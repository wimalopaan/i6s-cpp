#pragma once

#include <cstdint>

#include "wm/arch/regs.hpp"
#include "wm/hw/gpio.hpp"

// FS-i6S power latch (PB15), power button sense (PB14, active low) and the blue button LEDs (PD10/PD11)
// (flysky-i6s-rs hw/power.rs).
namespace wm::hw::power {

inline constexpr uint32_t kLatchPin = 1u << 15;   // PB15
inline constexpr uint32_t kButtonPin = 1u << 14;  // PB14
inline constexpr uint32_t kLedPins = (1u << 10) | (1u << 11);  // PD10, PD11

/// Latches the power rail before the user releases the power buttons. Must run first in main().
inline void init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIODEN;
    set_mode_mask<Port::B>(kLatchPin, Mode::Output);
    set_pins<Port::B>(kLatchPin);
    set_mode_mask<Port::B>(kButtonPin, Mode::Input);
    set_pull_mask<Port::B>(kButtonPin, Pull::Up);
    set_mode_mask<Port::D>(kLedPins, Mode::Output);
    set_pins<Port::D>(kLedPins);
}

/// Drives PB15 high again (before the DFU jump, so the radio stays on without USB power).
inline void latch_on() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    set_mode_mask<Port::B>(kLatchPin, Mode::Output);
    set_pins<Port::B>(kLatchPin);
}

inline void set_leds(const bool on) {
    if (on) {
        set_pins<Port::D>(kLedPins);
    } else {
        clear_pins<Port::D>(kLedPins);
    }
}

[[nodiscard]] inline bool is_power_button_pressed() {
    return (read_pins<Port::B>() & kButtonPin) == 0;
}

/// Turns the LEDs off, releases the latch and sleeps until the rail collapses.
[[noreturn]] inline void power_off() {
    set_leds(false);
    clear_pins<Port::B>(kLatchPin);
    for (;;) {
        __WFI();
    }
}

}  // namespace wm::hw::power
