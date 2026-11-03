#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "wm/arch/regs.hpp"
#include "wm/hw/gpio.hpp"
#include "wm/logic/touch_nav.hpp"

// FocalTech FT6236 capacitive touch controller on I2C1 (FS-i6S), 400 kHz:
// PB8 = SCL, PB9 = SDA (AF1, open-drain, high speed, pull-up), PA15 = RST (active low),
// PC12 = INT (active low; polling mode keeps it low for the whole touch).
// Frame: 7 bytes from register 0x00, decoded by logic::touch_nav::parse_packet.
namespace wm::hw::ft6236 {

using logic::touch_nav::TouchSample;

inline constexpr uint8_t kI2cAddr = 0x38;
inline constexpr uint32_t kRstPin = 1u << 15;  // PA15
inline constexpr uint32_t kIntPin = 1u << 12;  // PC12
inline constexpr uint32_t kI2cPins = (1u << 8) | (1u << 9);  // PB8, PB9

// PRESC 0, SCLDEL 0xB, SDADEL 0, SCLH 0x1A, SCLL 0x4B on the 48 MHz SYSCLK (Rust 0x00B0_1A4B).
inline constexpr uint32_t kTimingr = 0x00B0'1A4B;
inline constexpr uint32_t kSpinLimit = 10'000;
inline constexpr uint32_t kStopSpinLimit = 2'000;
inline constexpr uint8_t kMaxFailures = 5;

// FT6236 configuration registers.
inline constexpr uint8_t kRegGMode = 0xA4;       // 0x00 = polling mode
inline constexpr uint8_t kRegThreshold = 0x80;   // touch detection threshold
inline constexpr uint8_t kRegPeriodActive = 0x88;  // active report rate

namespace detail {

inline constexpr uint32_t kErrorMask = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;
inline constexpr uint32_t kStopErrorMask = I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;
inline constexpr uint32_t kAllFlags = I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF |
                                      I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_PECCF | I2C_ICR_TIMOUTCF |
                                      I2C_ICR_ALERTCF;
static_assert(kAllFlags == 0x3F38);

// Consecutive read failures; after kMaxFailures the controller is hard-reset.
inline constinit uint8_t g_fail_count = 0;

/// Busy-waits at least `n` core cycles, same loop as Rust `cortex_m::asm::delay` (2 n cycles on M0).
inline void delay_cycles(const uint32_t n) {
    uint32_t real = 1 + n / 2;
    __asm volatile("1: subs %0, #1\n\tbne 1b" : "+l"(real) : : "cc");
}

inline void delay_ms(const uint32_t ms) {
    delay_cycles(48'000 * ms);
}

inline void clear_flags() {
    I2C1->ICR = kAllFlags;
}

/// Waits for `ready` in ISR. False on any `errors` flag or after kSpinLimit polls.
inline bool wait_isr(const uint32_t ready, const uint32_t errors) {
    uint32_t timeout = kSpinLimit;
    while ((I2C1->ISR & ready) == 0 && timeout > 0) {
        if ((I2C1->ISR & errors) != 0) {
            return false;
        }
        --timeout;
    }
    return timeout != 0;
}

/// Ensures the GPIOB and I2C1 clocks, SYSCLK as I2C1 kernel clock, and the PB8/PB9 AF1 pin setup.
inline void configure_i2c1_pins() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    set_mode_mask<Port::B>(kI2cPins, Mode::Alternate);
    set_open_drain_mask<Port::B>(kI2cPins, true);
    set_high_speed_mask<Port::B>(kI2cPins);
    set_pull_mask<Port::B>(kI2cPins, Pull::Up);
    set_alt_function<Port::B>({8}, {1});
    set_alt_function<Port::B>({9}, {1});
}

/// Resets the I2C1 block, restores timing and re-enables it. GPIO configuration is preserved.
inline void reset_i2c1_peripheral() {
    I2C1->CR1 &= ~I2C_CR1_PE;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = kTimingr;
    clear_flags();
    I2C1->CR1 |= I2C_CR1_PE;
}

/// Aborts a failed transfer (bounded STOP wait) and returns I2C1 to a clean enabled state.
inline void recover_i2c1() {
    I2C1->CR2 |= I2C_CR2_STOP;
    for (uint32_t timeout = kStopSpinLimit; (I2C1->ISR & I2C_ISR_STOPF) == 0 && timeout > 0; --timeout) {
    }
    I2C1->CR1 &= ~I2C_CR1_PE;
    clear_flags();
    reset_i2c1_peripheral();
}

/// Pulses RST low for 20 ms and waits 50 ms for the controller to boot.
inline void pulse_reset() {
    clear_pins<Port::A>(kRstPin);
    delay_ms(20);
    set_pins<Port::A>(kRstPin);
    delay_ms(50);
}

}  // namespace detail

/// Writes one FT6236 register. Recovers the bus and returns false on NACK, bus error or timeout.
inline bool write_reg(const uint8_t reg, const uint8_t val) {
    I2C1->CR2 = (static_cast<uint32_t>(kI2cAddr) << 1) | (2u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START |
                I2C_CR2_AUTOEND;
    if (!detail::wait_isr(I2C_ISR_TXIS, detail::kErrorMask)) {
        detail::recover_i2c1();
        return false;
    }
    I2C1->TXDR = reg;
    if (!detail::wait_isr(I2C_ISR_TXIS, detail::kErrorMask)) {
        detail::recover_i2c1();
        return false;
    }
    I2C1->TXDR = val;
    if (!detail::wait_isr(I2C_ISR_STOPF, detail::kStopErrorMask)) {
        detail::recover_i2c1();
        return false;
    }
    I2C1->ICR = I2C_ICR_STOPCF;
    return true;
}

namespace detail {

/// Polling mode (INT stays low during a touch), threshold 22, ~70 Hz report rate.
inline void configure_controller() {
    delay_ms(10);
    write_reg(kRegGMode, 0x00);
    write_reg(kRegThreshold, 22);
    write_reg(kRegPeriodActive, 14);
}

}  // namespace detail

/// Brings up I2C1 on PB8/PB9 (clocks, kernel clock, pins, peripheral reset, 400 kHz timing).
/// Independent of the touch controller, so another I2C1 user can call it on its own.
inline void init_i2c1() {
    detail::configure_i2c1_pins();
    detail::reset_i2c1_peripheral();
}

/// Full touch bring-up: RST/INT pins, I2C1, 20 ms hardware reset, controller registers.
/// Blocks for about 160 ms (busy-wait, no watchdog feed), like the Rust driver.
inline void init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    set_mode_mask<Port::A>(kRstPin, Mode::Output);
    set_mode_mask<Port::C>(kIntPin, Mode::Input);
    set_pull_mask<Port::C>(kIntPin, Pull::Up);
    detail::configure_i2c1_pins();

    detail::pulse_reset();
    detail::reset_i2c1_peripheral();
    detail::configure_controller();
}

/// Hard-resets the controller: recovers I2C1, pulses RST and rewrites the configuration registers.
inline void reset_touch_controller() {
    detail::recover_i2c1();
    detail::pulse_reset();
    detail::configure_controller();
}

/// True while the INT pin (PC12) is low, i.e. a touch is in progress.
[[nodiscard]] inline bool is_touch_int_asserted() {
    return (read_pins<Port::C>() & kIntPin) == 0;
}

namespace detail {

inline void record_failure() {
    recover_i2c1();
    const auto fails = static_cast<uint8_t>(g_fail_count + 1);
    if (fails >= kMaxFailures) {
        g_fail_count = 0;
        reset_touch_controller();
    } else {
        g_fail_count = fails;
    }
}

}  // namespace detail

/// Reads and decodes the 7-byte touch frame. Returns nullopt on NACK, bus error or timeout; after
/// five consecutive failures the controller is hard-reset (~160 ms).
[[nodiscard]] inline std::optional<TouchSample> read_touch() {
    I2C1->CR2 = (static_cast<uint32_t>(kI2cAddr) << 1) | (1u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    if (!detail::wait_isr(I2C_ISR_TXIS, detail::kErrorMask)) {
        detail::record_failure();
        return std::nullopt;
    }
    I2C1->TXDR = 0x00;
    if (!detail::wait_isr(I2C_ISR_TC, detail::kErrorMask)) {
        detail::record_failure();
        return std::nullopt;
    }

    // Repeated START, read registers 0x00..0x06, STOP via AUTOEND.
    I2C1->CR2 = (static_cast<uint32_t>(kI2cAddr) << 1) | (7u << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
                I2C_CR2_START | I2C_CR2_AUTOEND;
    logic::touch_nav::Packet buf{};
    for (uint8_t& byte : buf) {
        if (!detail::wait_isr(I2C_ISR_RXNE, detail::kErrorMask)) {
            detail::record_failure();
            return std::nullopt;
        }
        byte = static_cast<uint8_t>(I2C1->RXDR & I2C_RXDR_RXDATA);
    }
    if (!detail::wait_isr(I2C_ISR_STOPF, detail::kStopErrorMask)) {
        detail::record_failure();
        return std::nullopt;
    }
    I2C1->ICR = I2C_ICR_STOPCF;

    detail::g_fail_count = 0;
    return logic::touch_nav::parse_packet(buf);
}

}  // namespace wm::hw::ft6236
