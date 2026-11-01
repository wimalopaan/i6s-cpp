#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ferrox/arch/critical.hpp"
#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/brr.hpp"

namespace ferrox::hw::uart {

// PC13: module power switch (GPIO output).
inline constexpr uint32_t kPowerPin = 13;
// PD5: USART2_TX, PA15: USART2_RX.
inline constexpr uint32_t kTxPin = 5;
inline constexpr uint32_t kRxPin = 15;
// DMA1 channel 6 carries USART2 RX (circular ring), channel 7 carries USART2 TX (fixed mapping on STM32F072).
inline constexpr std::size_t kRxBufSize = 256;
inline constexpr std::size_t kTxBufSize = 64;
static_assert((kRxBufSize & (kRxBufSize - 1)) == 0, "RX ring size must be a power of two");

// Raw NVIC priority byte (0x40). CMSIS NVIC_SetPriority would shift it by 6 on Cortex-M0.
inline constexpr uint8_t kIrqPriorityRaw = 0x40;

inline constinit uint8_t g_rx_buf[kRxBufSize]{};
inline constinit uint8_t g_tx_buf[kTxBufSize]{};
inline constinit std::size_t g_rx_tail = 0;
inline constinit volatile bool g_tx_busy = false;
inline constinit volatile bool g_rx_idle_pending = false;
inline constinit volatile bool g_active_high = true;
inline constinit volatile bool g_power_on = false;
inline constinit volatile bool g_half_duplex = false;

/// Write position of the RX DMA (the producer side of the ring).
inline std::size_t rx_head() {
    return kRxBufSize - DMA1_Channel6->CNDTR;
}

inline void apply_power_pin() {
    const bool on = g_power_on;
    const bool high = g_active_high ? on : !on;
    if (high) {
        set_pins<Port::C>(1u << kPowerPin);
    } else {
        clear_pins<Port::C>(1u << kPowerPin);
    }
}

/// Enables the PC13 output (initially OFF according to polarity).
inline void init_power_switch(bool active_high) {
    g_active_high = active_high;
    g_power_on = false;

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    set_mode_mask<Port::C>(1u << kPowerPin, Mode::Output);
    apply_power_pin();
}

inline void set_power_polarity(bool active_high) {
    g_active_high = active_high;
    apply_power_pin();
}

inline void set_module_power(bool power_on) {
    g_power_on = power_on;
    apply_power_pin();
}

inline bool is_half_duplex() {
    return g_half_duplex;
}

enum class Duplex : uint8_t { Full, Half };

/// Configures USART2 for CRSF (full-duplex) or single-wire half-duplex on PD5.
inline void set_uart_enabled(bool enabled, uint8_t baud_index, Duplex duplex) {
    g_half_duplex = duplex == Duplex::Half;

    if (enabled) {
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIODEN;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

        set_mode_mask<Port::D>(1u << kTxPin, Mode::Alternate);
        set_high_speed_mask<Port::D>(1u << kTxPin);
        set_alt_function<Port::D>({kTxPin}, {0});

        if (g_half_duplex) {
            set_open_drain_mask<Port::D>(1u << kTxPin, true);
            set_pull_mask<Port::D>(1u << kTxPin, Pull::Up);
            set_mode_mask<Port::A>(1u << kRxPin, Mode::Input);
        } else {
            set_open_drain_mask<Port::D>(1u << kTxPin, false);
            set_pull_mask<Port::D>(1u << kTxPin, Pull::None);
            set_mode_mask<Port::A>(1u << kRxPin, Mode::Alternate);
            set_pull_mask<Port::A>(1u << kRxPin, Pull::Up);
            set_alt_function<Port::A>({kRxPin}, {1});
        }

        USART2->CR1 = 0;
        USART2->CR2 = 0;
        USART2->CR3 = (g_half_duplex ? USART_CR3_HDSEL : 0) | USART_CR3_DMAR | USART_CR3_DMAT;
        USART2->BRR = logic::brr_for_baud_index(baud_index);
        USART2->ICR = 0xFFFF'FFFFu;

        RCC->AHBENR |= RCC_AHBENR_DMAEN;
        DMA1_Channel6->CCR = 0;
        DMA1_Channel6->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&USART2->RDR));
        DMA1_Channel6->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_rx_buf));
        DMA1_Channel6->CNDTR = static_cast<uint32_t>(kRxBufSize);
        DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
        g_rx_tail = rx_head();

        DMA1_Channel7->CCR = 0;
        DMA1_Channel7->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&USART2->TDR));
        g_tx_busy = false;
        g_rx_idle_pending = false;

        NVIC_ClearPendingIRQ(USART2_IRQn);
        reinterpret_cast<volatile uint8_t*>(&NVIC->IP[0])[USART2_IRQn] = kIrqPriorityRaw;
        NVIC_EnableIRQ(USART2_IRQn);

        USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_TCIE;
    } else {
        NVIC_DisableIRQ(USART2_IRQn);
        DMA1_Channel6->CCR = 0;
        DMA1_Channel7->CCR = 0;
        USART2->CR1 = 0;
        USART2->CR3 = 0;
        g_rx_tail = 0;
        g_tx_busy = false;

        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
        set_mode_mask<Port::D>(1u << kTxPin, Mode::Input);
        set_open_drain_mask<Port::D>(1u << kTxPin, false);
        set_mode_mask<Port::A>(1u << kRxPin, Mode::Input);
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
        USART2->CR1 &= ~USART_CR1_RE;
    }

    USART2->ICR = USART_ICR_TCCF;
    g_tx_busy = true;
    DMA1_Channel7->CCR = 0;
    DMA1_Channel7->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_tx_buf));
    DMA1_Channel7->CNDTR = static_cast<uint32_t>(n);
    DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
    return n;
}

/// Returns true once after the RX line went idle (end of a received burst).
inline bool take_rx_idle() {
    arch::CriticalSection cs;
    const bool pending = g_rx_idle_pending;
    g_rx_idle_pending = false;
    return pending;
}

/// USART2 interrupt body: TC frees the transmitter, IDLE marks the end of an RX burst.
inline void on_irq() {
    const uint32_t isr = USART2->ISR;
    if ((isr & USART_ISR_TC) != 0) {
        USART2->ICR = USART_ICR_TCCF;
        USART2->CR1 |= USART_CR1_RE;
        g_tx_busy = false;
    }
    if ((isr & USART_ISR_IDLE) != 0) {
        USART2->ICR = USART_ICR_IDLECF;
        g_rx_idle_pending = true;
    }
}

/// Returns the next received byte.
/// Bytes overwritten by the DMA before they are read (more than kRxBufSize unread) are lost.
inline std::optional<uint8_t> read_byte() {
    if ((USART2->ISR & USART_ISR_ORE) != 0) {
        USART2->ICR = USART_ICR_ORECF;
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

}  // namespace ferrox::hw::uart
