#pragma once

#include <cstdint>

#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/buzzer.hpp"

// Piezo buzzer on PA8 (TIM1_CH1, alternate function 2). The timer counts 1 MHz ticks and
// drives a 50% duty square wave. The pure timing comes from logic::buzzer::pwm_timing.
namespace ferrox::hw::buzzer {

inline constexpr uint32_t kTimerPrescaler = 47;  // 48 MHz / (47 + 1) = 1 MHz tick

// Brings the timer up with the output off. Call once at boot before any sound is played.
inline void init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    set_mode_mask<Port::A>(1u << 8, Mode::Alternate);
    set_alt_function<Port::A>(PinNumber{8}, AlternateFunction{2});

    TIM1->CR1 = 0;
    TIM1->PSC = kTimerPrescaler;
    // OC1M = PWM mode 1, OC1PE = preload enable.
    TIM1->CCMR1 = (TIM1->CCMR1 & ~0x7Fu) | (6u << 4) | (1u << 3);
    // CC1E = output enable, CC1P = active high.
    TIM1->CCER |= (1u << 0) | (1u << 1);
    // MOE: the advanced timer drives its outputs only when this is set.
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;
}

inline void on(const logic::buzzer::FreqHz freq) {
    const auto timing = logic::buzzer::pwm_timing(freq);
    TIM1->ARR = timing.arr;
    TIM1->CCR1 = timing.ccr;
    if (TIM1->CNT > timing.arr) {
        TIM1->CNT = 0;
    }
    // ARPE = buffered period, CEN = counter running.
    TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

inline void off() {
    TIM1->CR1 = 0;
    TIM1->CCR1 = 0;
    TIM1->CNT = 0;
    TIM1->SR = 0;
}

// Tracks the frequency currently on the pin, so the timer is only touched when it changes.
struct Output {
    logic::buzzer::FreqHz applied{};

    // Applies the frequency from logic::buzzer::Player::output(). Zero means silent.
    inline void sync(const logic::buzzer::FreqHz target) {
        if (target == applied) {
            return;
        }
        if (target.value == 0) {
            off();
        } else {
            on(target);
        }
        applied = target;
    }
};

}  // namespace ferrox::hw::buzzer
