#pragma once

// Trainer glue: TIM15 with PF9 input capture (PPM in) and a compare interrupt that
// drives PF10 in software (PPM out), as in the Rust `trainer/mod.rs`. CMSIS registers only.
// Main-loop access to the shared decoder and generator goes through CriticalSection,
// which masks the ISR, so the ISR can use the same fields without a lock.

#include <array>
#include <cstdint>
#include <optional>

#include "ferrox/arch/critical.hpp"
#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/trainer.hpp"

namespace ferrox::hw::trainer {

using PpmOutPin = Pin<Port::F, 10>;
using PpmInPin = Pin<Port::F, 9>;

// Raw NVIC priority byte (0x90). CMSIS NVIC_SetPriority would shift it by 6 on Cortex-M0.
inline constexpr uint8_t kIrqPriorityRaw = 0x90;
// PSC=47 gives 1 us ticks. ARR=0xFFFF is a free-running 16-bit counter.
inline constexpr uint16_t kTimerPrescaler = 47;
inline constexpr uint16_t kTimerPeriod = 0xFFFF;
// CC1S = 01 (TI1) and IC1F = 0011 (8-sample filter). CC2 stays in compare mode and only raises its interrupt.
inline constexpr uint32_t kCcmr1 = 0x0031;
// CC1E (bit 0) enables capture, CC1P (bit 1) captures on the falling edge.
inline constexpr uint32_t kCcer = (1u << 0) | (1u << 1);
inline constexpr uint16_t kFirstCompareUs = 400;

// ISR and main share these. Main touches them only inside CriticalSection.
inline constinit logic::trainer::PpmDecoder g_decoder{};
inline constinit logic::trainer::PpmGenerator g_generator{};
inline constinit bool g_in_enabled = false;
inline constinit bool g_out_enabled = false;
// ISR-only state.
inline constinit uint16_t g_last_capture = 0;

inline void set_irq_priority(IRQn_Type irq) {
    reinterpret_cast<volatile uint8_t*>(&NVIC->IP[0])[irq] = kIrqPriorityRaw;
}

/// Configures the pins and TIM15 without starting the timer. Call set_enabled() to start it.
inline void init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    PpmOutPin::write(true);
    PpmOutPin::set_mode(Mode::Output);
    PpmInPin::set_pull(Pull::Up);
    PpmInPin::set_alt_function(AlternateFunction{0});
    PpmInPin::set_mode(Mode::Alternate);

    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->CR1 = 0;
    TIM15->PSC = kTimerPrescaler;
    TIM15->ARR = kTimerPeriod;
    TIM15->CCMR1 = kCcmr1;
    TIM15->CCER = kCcer;
    TIM15->CCR2 = kFirstCompareUs;
    TIM15->EGR = TIM_EGR_UG;
    TIM15->SR = 0;
    TIM15->DIER = 0;
    set_irq_priority(TIM15_IRQn);
}

/// Starts or stops PPM in and out. Stopping drives PF10 high (idle line).
inline void set_enabled(bool in, bool out) {
    arch::CriticalSection cs;
    // Restart the counter only from a stopped timer, so a running output schedule is not shifted.
    if ((in || out) && (TIM15->CR1 & TIM_CR1_CEN) == 0) {
        TIM15->SR = 0;
        TIM15->EGR = TIM_EGR_UG;
    }
    if (out && !g_out_enabled) {
        // Re-arm the compare so a stale CCR2 from an earlier run cannot stall the output.
        g_generator.reset_frame();
        TIM15->CCR2 = static_cast<uint16_t>(TIM15->CNT + kFirstCompareUs);
    }
    if (in && !g_in_enabled) {
        g_decoder.reset();
        g_last_capture = static_cast<uint16_t>(TIM15->CCR1);
    }
    g_in_enabled = in;
    g_out_enabled = out;

    uint32_t dier = 0;
    if (out) {
        dier |= TIM_DIER_CC2IE;
    }
    if (in) {
        dier |= TIM_DIER_CC1IE;
    }
    TIM15->DIER = dier;

    if (in || out) {
        NVIC_EnableIRQ(TIM15_IRQn);
        TIM15->CR1 |= TIM_CR1_CEN;
        return;
    }
    TIM15->CR1 = 0;
    NVIC_DisableIRQ(TIM15_IRQn);
    PpmOutPin::write(true);
}


/// Sets the 8 trainer output channels, clamped to 900..2100 us by the generator.
template <std::size_t N>
inline void publish_out(const std::array<uint16_t, N>& chs) {
    arch::CriticalSection cs;
    g_generator.update_channels(chs);
}

/// Advances the student signal-loss timer by dt_ms.
inline void tick(logic::trainer::DeltaMs dt) {
    arch::CriticalSection cs;
    g_decoder.tick(dt);
}

/// Latest valid student frame, or nullopt once the signal is lost.
inline std::optional<logic::trainer::InChannels> student_channels() {
    arch::CriticalSection cs;
    return g_decoder.channels();
}

// Called from TIM15_IRQHandler.
inline void on_tim15() {
    const uint32_t sr = TIM15->SR;

    if ((sr & TIM_SR_CC2IF) != 0) {
        TIM15->SR = ~TIM_SR_CC2IF;
        if (g_out_enabled) {
            const auto step = g_generator.next_interval();
            PpmOutPin::write(step.high);
            TIM15->CCR2 = static_cast<uint16_t>(TIM15->CCR2 + step.duration.value);
        } else {
            PpmOutPin::write(true);
        }
    }

    if ((sr & TIM_SR_CC1IF) != 0) {
        TIM15->SR = ~TIM_SR_CC1IF;
        const auto capture = static_cast<uint16_t>(TIM15->CCR1);
        const auto delta = static_cast<uint16_t>(capture - g_last_capture);
        g_last_capture = capture;
        if (g_in_enabled) {
            g_decoder.process_pulse(logic::trainer::PulseUs{delta});
        }
    }
}

}  // namespace ferrox::hw::trainer
