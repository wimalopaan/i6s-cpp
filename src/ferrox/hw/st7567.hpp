#pragma once

#include <algorithm>
#include <cstdint>

#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/framebuffer.hpp"

// ST7567 128x64 LCD on an 8-bit 6800-style parallel bus (PE0..PE7), with control on PB3..PB5 and PD2/PD7.
// Pin assignments and the init sequence follow the OpenI6X reference.
namespace ferrox::hw::lcd {

namespace cmd {
inline constexpr uint8_t kReset = 0xE2;
inline constexpr uint8_t kDisplayOff = 0xAE;
inline constexpr uint8_t kDisplayOn = 0xAF;
inline constexpr uint8_t kModeRam = 0xA4;
inline constexpr uint8_t kBias17 = 0xA3;
inline constexpr uint8_t kComNormal = 0xC0;
inline constexpr uint8_t kSegInverse = 0xA1;
inline constexpr uint8_t kPowerAllOn = 0x2F;
inline constexpr uint8_t kRegRatio = 0x23;
inline constexpr uint8_t kElectronicVolume = 0x81;
inline constexpr uint8_t kStartLine = 0x40;
inline constexpr uint8_t kPageStart = 0xB0;
// The controller has 132 columns, the LCD uses 128, so the window starts at column 4.
inline constexpr uint8_t kColLo = 0x04;
inline constexpr uint8_t kColHi = 0x10;
}  // namespace cmd

namespace detail {

using RS = Pin<Port::B, 3>;
using RST = Pin<Port::B, 4>;
using RW = Pin<Port::B, 5>;
using CS = Pin<Port::D, 2>;
using RD = Pin<Port::D, 7>;
using Backlight = Pin<Port::F, 3>;

inline constexpr uint32_t kDataMask = 0xFFu;
inline constexpr uint32_t kBacklightPwmMax = 999;

inline void delay_cycles(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        __asm volatile("nop");
    }
}

// Low byte of GPIOE->ODR is the 8-bit data bus (little-endian).
inline void put_byte(uint8_t byte) {
    *reinterpret_cast<volatile uint8_t*>(&GPIOE->ODR) = byte;
    RD::write(true);
    __asm volatile("nop");
    RD::write(false);
}

inline void write_byte(uint8_t byte, bool is_data) {
    RS::write(is_data);
    put_byte(byte);
}

inline void write_cmd(uint8_t c) { write_byte(c, false); }

}  // namespace detail

inline void init_hardware() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN |
                   RCC_AHBENR_GPIOFEN;

    set_mode_mask<Port::E>(detail::kDataMask, Mode::Output);
    set_mode_mask<Port::B>((1u << 3) | (1u << 4) | (1u << 5), Mode::Output);
    set_mode_mask<Port::D>((1u << 2) | (1u << 7), Mode::Output);
    set_mode_mask<Port::F>(1u << 3, Mode::Output);

    detail::CS::write(false);
    detail::RW::write(false);
    detail::RD::write(true);
    detail::RST::write(true);
    detail::RS::write(true);
}

inline void init_backlight_pwm() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    set_mode_mask<Port::C>(1u << 9, Mode::Alternate);
    set_alt_function<Port::C>(9, 0);

    // 48 MHz / (PSC + 1) = 1 MHz tick, ARR + 1 = 1000 ticks -> 1 kHz PWM on TIM3_CH4.
    TIM3->PSC = 47;
    TIM3->ARR = detail::kBacklightPwmMax;
    TIM3->CCR4 = detail::kBacklightPwmMax;
    // OC4M = PWM mode 1, OC4PE = preload enable.
    TIM3->CCMR2 = (TIM3->CCMR2 & ~(0x7Fu << 8)) | (6u << 12) | (1u << 11);
    TIM3->CCER |= (1u << 12);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

// Backlight: stock on/off on PF3 plus PWM duty on PC9 (0 = off, 1..100 = brightness).
inline void set_backlight_level(uint8_t pct) {
    if (pct == 0) {
        detail::Backlight::write(false);
        TIM3->CCR4 = 0;
    } else {
        detail::Backlight::write(true);
        TIM3->CCR4 = std::min<uint32_t>(pct, 100u) * detail::kBacklightPwmMax / 100u;
    }
}

inline void set_contrast(uint8_t ev) {
    detail::write_cmd(cmd::kElectronicVolume);
    detail::write_cmd(std::min<uint8_t>(ev, 63));
}

inline void init_controller() {
    detail::RST::write(false);
    detail::delay_cycles(500);
    detail::RST::write(true);
    detail::delay_cycles(500);

    detail::write_cmd(cmd::kReset);
    detail::delay_cycles(10000);

    detail::write_cmd(cmd::kDisplayOff);
    detail::write_cmd(cmd::kModeRam);
    detail::write_cmd(cmd::kBias17);
    detail::write_cmd(cmd::kComNormal);
    detail::write_cmd(cmd::kSegInverse);
    detail::write_cmd(cmd::kPowerAllOn);
    detail::write_cmd(cmd::kRegRatio);
    set_contrast(0x25);
    detail::write_cmd(cmd::kStartLine);
    detail::write_cmd(cmd::kPageStart);
    detail::write_cmd(cmd::kColLo);
    detail::write_cmd(cmd::kColHi);
    detail::write_cmd(cmd::kDisplayOn);
    detail::delay_cycles(10000);
}

inline void init() {
    init_hardware();
    init_controller();
    init_backlight_pwm();
}

// Sends all 8 pages; RS is set once per page, then each data byte is strobed.
inline void flush(const logic::display::Framebuffer& fb) {
    const auto& px = fb.data();
    for (uint8_t page = 0; page < 8; ++page) {
        detail::write_cmd(static_cast<uint8_t>(cmd::kPageStart | page));
        detail::write_cmd(cmd::kColLo);
        detail::write_cmd(cmd::kColHi);

        detail::RS::write(true);
        const std::size_t start = static_cast<std::size_t>(page) * logic::display::kWidth;
        for (std::size_t x = 0; x < static_cast<std::size_t>(logic::display::kWidth); ++x) {
            detail::put_byte(px[start + x]);
        }
    }
}

}  // namespace ferrox::hw::lcd
