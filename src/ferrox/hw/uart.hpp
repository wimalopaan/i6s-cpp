#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ferrox/arch/critical.hpp"
#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/brr.hpp"
#include "ferrox/util/ring.hpp"

namespace ferrox::hw::uart {

// PC13: module power switch (GPIO output).
inline constexpr uint32_t kPowerPin = 13;
// PD5: USART2_TX, PA15: USART2_RX.
inline constexpr uint32_t kTxPin = 5;
inline constexpr uint32_t kRxPin = 15;
// Raw NVIC priority byte (0x40). CMSIS NVIC_SetPriority would shift it by 6 on Cortex-M0.
inline constexpr uint8_t kIrqPriorityRaw = 0x40;

inline constinit logic::SpscRing<128> g_rx{};
inline constinit volatile bool g_active_high = true;
inline constinit volatile bool g_power_on = false;
inline constinit volatile bool g_half_duplex = false;
// Half-duplex: bytes we transmitted are echoed back on RX and must be dropped.
inline constinit volatile uint32_t g_echo_skip = 0;

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
    g_echo_skip = 0;

    if (enabled) {
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIODEN;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

        set_mode_mask<Port::D>(1u << kTxPin, Mode::Alternate);
        set_high_speed_mask<Port::D>(1u << kTxPin);
        set_alt_function<Port::D>(kTxPin, 0);

        if (g_half_duplex) {
            set_open_drain_mask<Port::D>(1u << kTxPin, true);
            set_pull_mask<Port::D>(1u << kTxPin, Pull::Up);
            set_mode_mask<Port::A>(1u << kRxPin, Mode::Input);
        } else {
            set_open_drain_mask<Port::D>(1u << kTxPin, false);
            set_pull_mask<Port::D>(1u << kTxPin, Pull::None);
            set_mode_mask<Port::A>(1u << kRxPin, Mode::Alternate);
            set_pull_mask<Port::A>(1u << kRxPin, Pull::Up);
            set_alt_function<Port::A>(kRxPin, 1);
        }

        USART2->CR1 = 0;
        USART2->CR2 = 0;
        USART2->CR3 = g_half_duplex ? USART_CR3_HDSEL : 0;
        USART2->BRR = logic::brr_for_baud_index(baud_index);
        USART2->ICR = 0xFFFF'FFFFu;

        g_rx.clear();
        NVIC_ClearPendingIRQ(USART2_IRQn);
        reinterpret_cast<volatile uint8_t*>(&NVIC->IP[0])[USART2_IRQn] = kIrqPriorityRaw;
        NVIC_EnableIRQ(USART2_IRQn);

        USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    } else {
        NVIC_DisableIRQ(USART2_IRQn);
        USART2->CR1 = 0;
        g_rx.clear();

        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
        set_mode_mask<Port::D>(1u << kTxPin, Mode::Input);
        set_open_drain_mask<Port::D>(1u << kTxPin, false);
        set_mode_mask<Port::A>(1u << kRxPin, Mode::Input);
    }
}

/// Transmits bytes with a bounded wait per byte. Returns the number of bytes sent.
inline std::size_t write_bytes(std::span<const uint8_t> bytes) {
    const bool half_duplex = g_half_duplex;
    std::size_t sent = 0;
    for (const uint8_t b : bytes) {
        uint32_t timeout = 2500;
        while ((USART2->ISR & USART_ISR_TXE) == 0) {
            if (--timeout == 0) {
                return sent;
            }
        }
        if (half_duplex) {
            arch::CriticalSection cs;
            g_echo_skip = g_echo_skip + 1;
        }
        USART2->TDR = b;
        ++sent;
    }
    return sent;
}

inline std::optional<uint8_t> read_byte() {
    return g_rx.pop();
}

/// USART2 interrupt body: store received bytes, drop half-duplex echoes.
inline void on_irq() {
    const uint32_t isr = USART2->ISR;
    if ((isr & USART_ISR_ORE) != 0) {
        USART2->ICR = USART_ICR_ORECF;
    }

    if ((isr & USART_ISR_RXNE) != 0) {
        const auto byte = static_cast<uint8_t>(USART2->RDR & 0xFFu);
        if (g_half_duplex) {
            const uint32_t skip = g_echo_skip;
            if (skip > 0) {
                g_echo_skip = skip - 1;
                return;
            }
        }
        g_rx.push(byte);
    }
}

}  // namespace ferrox::hw::uart
