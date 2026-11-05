#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "wm/arch/critical.hpp"
#include "wm/arch/regs.hpp"
#include "wm/hw/gpio.hpp"
#include "wm/logic/brr.hpp"

namespace wm::hw::uart {

#if WM_PCB_I6S
// FS-i6S: PF6 module power switch (H1), PB6 USART1_TX (AF0), PB7 USART1_RX (AF0) on J15.
inline constexpr Port kPowerPort = Port::F;
inline constexpr uint32_t kPowerPin = 6;
inline constexpr Port kTxPort = Port::B;
inline constexpr uint32_t kTxPin = 6;
inline constexpr Port kRxPort = Port::B;
inline constexpr uint32_t kRxPin = 7;
inline constexpr uint32_t kRxAf = 0;
inline constexpr uint32_t kGpioClocks = RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;
inline constexpr IRQn_Type kIrq = USART1_IRQn;
// USART1 uses the default DMA mapping: channel 3 RX (circular ring), channel 2 TX.
inline USART_TypeDef* usart() { return USART1; }
inline DMA_Channel_TypeDef* rx_dma() { return DMA1_Channel3; }
inline DMA_Channel_TypeDef* tx_dma() { return DMA1_Channel2; }
#else
// PC13: module power switch (GPIO output).
inline constexpr Port kPowerPort = Port::C;
inline constexpr uint32_t kPowerPin = 13;
// PD5: USART2_TX, PA15: USART2_RX.
inline constexpr Port kTxPort = Port::D;
inline constexpr uint32_t kTxPin = 5;
inline constexpr Port kRxPort = Port::A;
inline constexpr uint32_t kRxPin = 15;
inline constexpr uint32_t kRxAf = 1;
inline constexpr uint32_t kGpioClocks = RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
inline constexpr IRQn_Type kIrq = USART2_IRQn;
// DMA1 channel 6 carries USART2 RX (circular ring), channel 7 carries USART2 TX. On STM32F07x this is not the
// default (channels 5/4): it needs SYSCFG_CFGR1.USART2_DMA_RMP, set in set_uart_enabled().
inline USART_TypeDef* usart() { return USART2; }
inline DMA_Channel_TypeDef* rx_dma() { return DMA1_Channel6; }
inline DMA_Channel_TypeDef* tx_dma() { return DMA1_Channel7; }
#endif
inline constexpr std::size_t kRxBufSize = 256;
inline constexpr std::size_t kTxBufSize = 64;
static_assert((kRxBufSize & (kRxBufSize - 1)) == 0, "RX ring size must be a power of two");

// NVIC priority byte (0x40, Rust `set_priority`); only the top __NVIC_PRIO_BITS bits are implemented.
inline constexpr uint8_t kIrqPriorityRaw = 0x40;

inline constinit uint8_t g_rx_buf[kRxBufSize]{};
inline constinit uint8_t g_tx_buf[kTxBufSize]{};
inline constinit std::size_t g_rx_tail = 0;
inline constinit volatile bool g_tx_busy = false;
inline constinit volatile bool g_rx_idle_pending = false;
inline constinit volatile uint16_t g_rx_idle_pos = 0;
inline constinit bool g_active_high = true;
inline constinit bool g_power_on = false;
inline constinit bool g_half_duplex = false;

/// Write position of the RX DMA (the producer side of the ring).
inline std::size_t rx_head() {
    return kRxBufSize - rx_dma()->CNDTR;
}

inline void apply_power_pin() {
    const bool on = g_power_on;
    const bool high = g_active_high ? on : !on;
    if (high) {
        set_pins<kPowerPort>(1u << kPowerPin);
    } else {
        clear_pins<kPowerPort>(1u << kPowerPin);
    }
}

/// Enables the module power output (initially OFF according to polarity).
inline void init_power_switch(const bool active_high) {
    g_active_high = active_high;
    g_power_on = false;

    RCC->AHBENR |= kGpioClocks;
    set_mode_mask<kPowerPort>(1u << kPowerPin, Mode::Output);
    apply_power_pin();
}

inline void set_power_polarity(const bool active_high) {
    g_active_high = active_high;
    apply_power_pin();
}

inline void set_module_power(const bool power_on) {
    g_power_on = power_on;
    apply_power_pin();
}

inline bool is_half_duplex() {
    return g_half_duplex;
}

enum class Duplex : uint8_t { Full, Half };

/// Configures the CRSF USART (full-duplex) or single-wire half-duplex on the TX pin.
inline void set_uart_enabled(const bool enabled, const uint8_t baud_index, const Duplex duplex) {
    g_half_duplex = duplex == Duplex::Half;

    if (enabled) {
        RCC->AHBENR |= kGpioClocks;
#if WM_PCB_I6S
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
#else
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#endif

        set_mode_mask<kTxPort>(1u << kTxPin, Mode::Alternate);
        set_high_speed_mask<kTxPort>(1u << kTxPin);
        set_alt_function<kTxPort>({kTxPin}, {0});

        if (g_half_duplex) {
            set_open_drain_mask<kTxPort>(1u << kTxPin, true);
            set_pull_mask<kTxPort>(1u << kTxPin, Pull::Up);
            set_mode_mask<kRxPort>(1u << kRxPin, Mode::Input);
        } else {
            set_open_drain_mask<kTxPort>(1u << kTxPin, false);
            set_pull_mask<kTxPort>(1u << kTxPin, Pull::None);
            set_mode_mask<kRxPort>(1u << kRxPin, Mode::Alternate);
            set_pull_mask<kRxPort>(1u << kRxPin, Pull::Up);
            set_alt_function<kRxPort>({kRxPin}, {kRxAf});
        }

        USART_TypeDef* const u = usart();
        u->CR1 = 0;
        u->CR2 = 0;
        u->CR3 = (g_half_duplex ? USART_CR3_HDSEL : 0) | USART_CR3_DMAR | USART_CR3_DMAT;
        u->BRR = logic::brr_for_baud_index(baud_index);
        u->ICR = 0xFFFF'FFFFu;

        RCC->AHBENR |= RCC_AHBENR_DMAEN;
#if !WM_PCB_I6S
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART2_DMA_RMP;
#endif
        rx_dma()->CCR = 0;
        rx_dma()->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&u->RDR));
        rx_dma()->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_rx_buf));
        rx_dma()->CNDTR = static_cast<uint32_t>(kRxBufSize);
        rx_dma()->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
        g_rx_tail = rx_head();

        tx_dma()->CCR = 0;
        tx_dma()->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&u->TDR));
        g_tx_busy = false;
        g_rx_idle_pending = false;

        NVIC_ClearPendingIRQ(kIrq);
        // Cortex-M0 IPR registers are word-access only; NVIC_SetPriority does a word read-modify-write.
        NVIC_SetPriority(kIrq, kIrqPriorityRaw >> (8u - __NVIC_PRIO_BITS));
        NVIC_EnableIRQ(kIrq);

        u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_TCIE;
    } else {
        NVIC_DisableIRQ(kIrq);
        rx_dma()->CCR = 0;
        tx_dma()->CCR = 0;
        usart()->CR1 = 0;
        usart()->CR3 = 0;
        g_rx_tail = 0;
        g_tx_busy = false;

#if WM_PCB_I6S
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
#else
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
#endif
        set_mode_mask<kTxPort>(1u << kTxPin, Mode::Input);
        set_open_drain_mask<kTxPort>(1u << kTxPin, false);
        set_mode_mask<kRxPort>(1u << kRxPin, Mode::Input);
    }
}

/// Queues up to kTxBufSize bytes for DMA transmission. Waits a bounded time for the previous
/// frame to leave the wire (TC interrupt). Returns the number of bytes queued (0 if still busy).
inline std::size_t write_bytes(std::span<const uint8_t> bytes) {
    uint32_t timeout = 50'000;
    while (g_tx_busy) {
        if (--timeout == 0) {
            return 0;
        }
    }

    const std::size_t n = std::min(bytes.size(), kTxBufSize);
    if (n == 0) {
        return 0;
    }
    for (std::size_t i = 0; i < n; ++i) {
        g_tx_buf[i] = bytes[i];
    }
    // Half-duplex: the receiver stays off while we transmit so our own bytes are never echoed.
    // The TC interrupt turns it back on.
    if (g_half_duplex) {
        usart()->CR1 &= ~USART_CR1_RE;
    }

    usart()->ICR = USART_ICR_TCCF;
    g_tx_busy = true;
    tx_dma()->CCR = 0;
    tx_dma()->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_tx_buf));
    tx_dma()->CNDTR = static_cast<uint32_t>(n);
    tx_dma()->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
    return n;
}

/// Read position of the RX ring (the consumer side).
inline std::size_t rx_tail() {
    return g_rx_tail;
}

/// Once after the RX line went idle: the ring position where the burst ended. Bytes before that
/// position may still be unread, so a partial frame is only stale once rx_tail() reaches it.
inline std::optional<std::size_t> take_rx_idle() {
    arch::CriticalSection cs;
    if (!g_rx_idle_pending) {
        return std::nullopt;
    }
    g_rx_idle_pending = false;
    return g_rx_idle_pos;
}

/// CRSF USART interrupt body: TC frees the transmitter, IDLE marks the end of an RX burst.
inline void on_irq() {
    USART_TypeDef* const u = usart();
    const uint32_t isr = u->ISR;
    if ((isr & USART_ISR_TC) != 0) {
        u->ICR = USART_ICR_TCCF;
        u->CR1 |= USART_CR1_RE;
        g_tx_busy = false;
    }
    if ((isr & USART_ISR_IDLE) != 0) {
        u->ICR = USART_ICR_IDLECF;
        g_rx_idle_pos = static_cast<uint16_t>(rx_head() & (kRxBufSize - 1));
        g_rx_idle_pending = true;
    }
}

/// Returns the next received byte.
/// Bytes overwritten by the DMA before they are read (more than kRxBufSize unread) are lost.
inline std::optional<uint8_t> read_byte() {
    if ((usart()->ISR & USART_ISR_ORE) != 0) {
        usart()->ICR = USART_ICR_ORECF;
    }

    const std::size_t head = rx_head();
    if (g_rx_tail == head) {
        return std::nullopt;
    }
    std::atomic_signal_fence(std::memory_order_seq_cst);
    const uint8_t byte = g_rx_buf[g_rx_tail];
    std::atomic_signal_fence(std::memory_order_seq_cst);
    g_rx_tail = (g_rx_tail + 1) & (kRxBufSize - 1);
    return byte;
}

}  // namespace wm::hw::uart
