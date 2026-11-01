#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"

namespace ferrox::hw::adc {

inline constexpr std::size_t kStdChannels = 11;
inline constexpr std::size_t kChannels = 15;
inline constexpr uint16_t kAdcMin = 0;
inline constexpr uint16_t kAdcCenter = 2048;
inline constexpr uint16_t kAdcMax = 4095;

using Samples = std::array<uint16_t, kChannels>;

namespace detail {

inline constexpr uint32_t kChselrStd = 0x07FF;  // CH0..CH10
inline constexpr uint32_t kChselrP7 = 0xF7FF;   // CH0..CH10 + CH12..CH15
// DMA CCR: MINC | PSIZE=16 bit | MSIZE=16 bit | PL=High | EN
inline constexpr uint32_t kDmaCcr = (1u << 7) | (1u << 8) | (1u << 10) | (2u << 12) | 1u;

constexpr std::size_t count_for(const bool p7) {
    return p7 ? kChannels : kStdChannels;
}

constexpr uint32_t chselr_for(const bool p7) {
    return p7 ? kChselrP7 : kChselrStd;
}

// Written by DMA1 (raw) and read back as snapshot by the main loop.
inline constinit Samples g_raw{};
inline constinit Samples g_snapshot = [] {
    Samples s{};
    s.fill(kAdcCenter);
    return s;
}();
inline constinit std::size_t g_scan_count = kStdChannels;

inline void take_snapshot() {
    for (std::size_t i = 0; i < g_scan_count; ++i) {
        g_snapshot[i] = *static_cast<volatile uint16_t*>(&g_raw[i]);
    }
    for (std::size_t i = g_scan_count; i < kChannels; ++i) {
        g_snapshot[i] = kAdcCenter;
    }
}

/// Clears DMA/ADC flags and restarts DMA channel 1 plus the ADC sequence.
inline void rearm() {
    DMA1->IFCR = 0x0F;  // GIF1, TCIF1, HTIF1, TEIF1
    ADC1->ISR = ADC_ISR_EOSEQ | ADC_ISR_OVR;

    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CNDTR = static_cast<uint32_t>(g_scan_count);
    DMA1_Channel1->CCR = kDmaCcr;

    ADC1->CR |= ADC_CR_ADSTART;
}

}  // namespace detail

/// Configures PA0..PA7, PB0..PB1, PC0 and PC2..PC5 as analog inputs, ADC1 and DMA1 channel 1
/// for a continuous scan (11 channels, or 15 with the P7 header).
inline void init(const bool p7_enabled) {
    detail::g_scan_count = detail::count_for(p7_enabled);

    RCC->AHBENR |= RCC_AHBENR_DMAEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    set_mode_mask<Port::A>(0x00FF, Mode::Analog);
    set_pull_mask<Port::A>(0x00FF, Pull::None);
    set_mode_mask<Port::B>(0x0003, Mode::Analog);
    set_pull_mask<Port::B>(0x0003, Pull::None);
    set_mode_mask<Port::C>(0x003D, Mode::Analog);  // PC0, PC2..PC5
    set_pull_mask<Port::C>(0x003D, Pull::None);

    ADC1->CFGR2 = 2u << 30;  // CKMODE = PCLK/4 (synchronous)

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    for (uint32_t t = 100'000; t > 0 && (ADC1->ISR & ADC_ISR_ADRDY) == 0; --t) {
    }

    ADC1->CHSELR = detail::chselr_for(p7_enabled);
    ADC1->SMPR = 0b111;  // 239.5 ADC cycles
    ADC1->CFGR1 = ADC_CFGR1_DMAEN;

    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&ADC1->DR));
    DMA1_Channel1->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(detail::g_raw.data()));
    DMA1_Channel1->CNDTR = static_cast<uint32_t>(detail::g_scan_count);
    DMA1_Channel1->CCR = detail::kDmaCcr;

    ADC1->CR |= ADC_CR_ADSTART;
}

/// Switches between 11 and 15 scanned channels at runtime.
inline void set_p7_enabled(const bool enabled) {
    detail::g_scan_count = detail::count_for(enabled);

    DMA1_Channel1->CCR = 0;
    if ((ADC1->CR & ADC_CR_ADSTART) != 0) {
        ADC1->CR |= ADC_CR_ADSTP;
        for (uint32_t t = 10'000; t > 0 && (ADC1->CR & ADC_CR_ADSTART) != 0; --t) {
        }
    }

    ADC1->CHSELR = detail::chselr_for(enabled);
    detail::rearm();
}

inline bool is_p7_enabled() {
    return detail::g_scan_count == kChannels;
}

/// Blocks (bounded) until the first DMA sequence completes, then snapshots it.
inline void wait_first_conversion() {
    for (uint32_t t = 200'000; t > 0 && (DMA1->ISR & DMA_ISR_TCIF1) == 0; --t) {
    }
    detail::take_snapshot();
    detail::rearm();
}

/// Returns the latest complete scan. Re-arms DMA when a new sequence finished.
inline Samples read_raw() {
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) {
        detail::take_snapshot();
        detail::rearm();
    }
    return detail::g_snapshot;
}

}  // namespace ferrox::hw::adc
