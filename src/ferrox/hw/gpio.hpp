#pragma once

#include <cstdint>

#include "ferrox/arch/regs.hpp"

namespace ferrox::hw {

enum class Port : uint8_t { A, B, C, D, E, F };
enum class Mode : uint32_t { Input = 0, Output = 1, Alternate = 2, Analog = 3 };
enum class Pull : uint32_t { None = 0, Up = 1, Down = 2 };

namespace detail {

/// Expands a 16-bit pin mask into a register mask with `bits` bits per pin, each set to `value`.
constexpr uint32_t spread(uint32_t pin_mask, uint32_t value, uint32_t bits) {
    uint32_t result = 0;
    for (uint32_t pin = 0; pin < 16; ++pin) {
        if ((pin_mask >> pin) & 1u) {
            result |= value << (pin * bits);
        }
    }
    return result;
}

template <Port P>
GPIO_TypeDef* port_regs() {
    if constexpr (P == Port::A) {
        return GPIOA;
    } else if constexpr (P == Port::B) {
        return GPIOB;
    } else if constexpr (P == Port::C) {
        return GPIOC;
    } else if constexpr (P == Port::D) {
        return GPIOD;
    } else if constexpr (P == Port::E) {
        return GPIOE;
    } else {
        return GPIOF;
    }
}

}  // namespace detail

template <Port P>
void set_mode_mask(uint32_t pin_mask, Mode mode) {
    GPIO_TypeDef* g = detail::port_regs<P>();
    g->MODER = (g->MODER & ~detail::spread(pin_mask, 3, 2)) |
               detail::spread(pin_mask, static_cast<uint32_t>(mode), 2);
}

template <Port P>
void set_pull_mask(uint32_t pin_mask, Pull pull) {
    GPIO_TypeDef* g = detail::port_regs<P>();
    g->PUPDR = (g->PUPDR & ~detail::spread(pin_mask, 3, 2)) |
               detail::spread(pin_mask, static_cast<uint32_t>(pull), 2);
}

template <Port P>
void set_open_drain_mask(uint32_t pin_mask, bool open_drain) {
    GPIO_TypeDef* g = detail::port_regs<P>();
    const uint32_t bits = detail::spread(pin_mask, 1, 1);
    g->OTYPER = open_drain ? (g->OTYPER | bits) : (g->OTYPER & ~bits);
}

template <Port P>
void set_high_speed_mask(uint32_t pin_mask) {
    GPIO_TypeDef* g = detail::port_regs<P>();
    g->OSPEEDR |= detail::spread(pin_mask, 3, 2);
}

/// Selects alternate function `af` (0..15) for a single pin.
template <Port P>
void set_alt_function(uint32_t pin, uint32_t af) {
    GPIO_TypeDef* g = detail::port_regs<P>();
    const uint32_t idx = pin / 8;
    const uint32_t shift = (pin % 8) * 4;
    g->AFR[idx] = (g->AFR[idx] & ~(0xFu << shift)) | ((af & 0xFu) << shift);
}

/// Drives the masked pins high (BSRR set bits).
template <Port P>
void set_pins(uint32_t pin_mask) {
    detail::port_regs<P>()->BSRR = pin_mask;
}

/// Drives the masked pins low (BSRR reset bits).
template <Port P>
void clear_pins(uint32_t pin_mask) {
    detail::port_regs<P>()->BSRR = pin_mask << 16;
}

template <Port P>
uint32_t read_pins() {
    return detail::port_regs<P>()->IDR;
}

template <Port P, uint32_t N>
    requires(N < 16)
struct Pin {
    static constexpr uint32_t mask = 1u << N;

    static void set_mode(Mode mode) { set_mode_mask<P>(mask, mode); }
    static void set_pull(Pull pull) { set_pull_mask<P>(mask, pull); }
    static void set_open_drain(bool open_drain) { set_open_drain_mask<P>(mask, open_drain); }
    static void set_high_speed() { set_high_speed_mask<P>(mask); }
    static void set_alt_function(uint32_t af) { hw::set_alt_function<P>(N, af); }
    static void write(bool high) { high ? set_pins<P>(mask) : clear_pins<P>(mask); }
    static bool read() { return (read_pins<P>() & mask) != 0; }
};

}  // namespace ferrox::hw
