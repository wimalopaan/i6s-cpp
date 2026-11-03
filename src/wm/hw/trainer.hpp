#pragma once

// Trainer glue without interrupts (CMSIS registers only):
// - PPM out: PF10 is TIM15_CH2 (AF0) in PWM mode 2. CCR2 holds the low sync pulse, and the TIM15 update
//   DMA request (DMA1 channel 5) writes the next period into the preloaded ARR from a circular buffer, so
//   the edges come from the timer hardware.
// - PPM in: PF9 is TIM15_CH1 (AF0) input capture. On every capture TIM15 sends a TRGO pulse (MMS = compare
//   pulse) to TIM2 (ITR1), which latches its free-running 1 us counter into CCR4 (IC4 = TRC). The TIM2 CC4
//   DMA request (DMA1 channel 4) stores the timestamps in a circular ring that tick() decodes.
// TIM15 has a single DMA channel on the STM32F072, hence the second timer for the input.
// Everything here runs in the main loop, so no locking is needed.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "wm/arch/regs.hpp"
#include "wm/hw/gpio.hpp"
#include "wm/logic/trainer.hpp"

namespace wm::hw::trainer {

using PpmOutPin = Pin<Port::F, 10>;
using PpmInPin = Pin<Port::F, 9>;

// PSC=47 gives 1 us ticks on both timers.
inline constexpr uint16_t kTimerPrescaler = 47;
// TIM15: CC1S = 01 (TI1) and IC1F = 0011 (8-sample filter); CC2 in PWM mode 2 (low below CCR2) with preload.
inline constexpr uint32_t kTim15Ccmr1 = 0x0031 | TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE;
// CC1E enables capture, CC1P captures on the falling edge. CC2E is set only while PPM out runs.
inline constexpr uint32_t kTim15Ccer = TIM_CCER_CC1E | TIM_CCER_CC1P;
// MMS = 011 (compare pulse): TRGO pulses on every CC1 capture.
inline constexpr uint32_t kTim15Cr2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1;
// TIM2: TS = 001 selects ITR1 (TIM15 TRGO, RM0091 internal trigger table); slave mode stays disabled.
inline constexpr uint32_t kTim2Smcr = TIM_SMCR_TS_0;
// CC4S = 11 maps IC4 on TRC, so CCR4 latches the counter on every TIM15 capture.
inline constexpr uint32_t kTim2Ccmr2 = TIM_CCMR2_CC4S;

// DMA1 channel 5 carries TIM15_UP, channel 4 TIM2_CH4 (default mapping, no SYSCFG remap).
inline DMA_Channel_TypeDef* out_dma() { return DMA1_Channel5; }
inline DMA_Channel_TypeDef* in_dma() { return DMA1_Channel4; }
// Half-word memory to TIM15->ARR, circular.
inline constexpr uint32_t kOutDmaCcr =
    DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_1;
// Word TIM2->CCR4 to memory, circular.
inline constexpr uint32_t kInDmaCcr = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL_0;

// 64 edges hold about 7 PPM frames (~150 ms). If the main loop stalls longer, the DMA laps the ring; the first
// delta then spans the lost edges, is far above kSyncMaxUs (32-bit timestamps, no aliasing) and resyncs the decoder.
inline constexpr std::size_t kInRingSize = 64;
static_assert((kInRingSize & (kInRingSize - 1)) == 0, "PPM in ring size must be a power of two");

// DMA buffers: ARR values (period - 1) of one output frame, and the input capture timestamps.
inline constinit volatile uint16_t g_out_arr[logic::trainer::kPpmOutPeriods]{};
inline constinit volatile uint32_t g_in_ring[kInRingSize]{};

inline constinit logic::trainer::PpmDecoder g_decoder{};
inline constinit bool g_in_enabled = false;
inline constinit bool g_out_enabled = false;
inline constinit std::size_t g_in_tail = 0;
inline constinit uint32_t g_last_capture = 0;

inline uint32_t dma_addr(const volatile void* p) {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

inline std::size_t in_head() {
    return (kInRingSize - in_dma()->CNDTR) & (kInRingSize - 1);
}

/// Sets the 8 trainer output channels, clamped to 900..2100 us. The DMA picks them up period by period.
template <std::size_t N>
inline void publish_out(const std::array<uint16_t, N>& chs) {
    const auto periods = logic::trainer::ppm_out_periods(chs);
    for (std::size_t i = 0; i < periods.size(); ++i) {
        g_out_arr[i] = static_cast<uint16_t>(periods[i] - 1);
    }
}

/// Configures the pins, TIM15, TIM2 and both DMA channels without starting anything. Call set_enabled() to start.
inline void init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN | RCC_AHBENR_DMAEN;
    PpmOutPin::write(true);
    PpmOutPin::set_mode(Mode::Output);
    PpmOutPin::set_alt_function(AlternateFunction{0});
    PpmInPin::set_pull(Pull::Up);
    PpmInPin::set_alt_function(AlternateFunction{0});
    PpmInPin::set_mode(Mode::Alternate);

    logic::trainer::OutChannels centre{};
    centre.fill(logic::trainer::kCenterUs);
    publish_out(centre);

    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->CR1 = 0;
    TIM15->DIER = 0;
    TIM15->PSC = kTimerPrescaler;
    TIM15->ARR = 0xFFFF;
    TIM15->CCMR1 = kTim15Ccmr1;
    TIM15->CCER = kTim15Ccer;
    TIM15->CCR2 = logic::trainer::kOutSyncUs;
    TIM15->CR2 = kTim15Cr2;
    TIM15->BDTR = TIM_BDTR_MOE;
    TIM15->EGR = TIM_EGR_UG;
    TIM15->SR = 0;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;
    TIM2->DIER = 0;
    TIM2->PSC = kTimerPrescaler;
    TIM2->ARR = 0xFFFF'FFFFu;
    TIM2->SMCR = kTim2Smcr;
    TIM2->CCMR2 = kTim2Ccmr2;
    TIM2->CCER = TIM_CCER_CC4E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;

    out_dma()->CCR = 0;
    out_dma()->CPAR = dma_addr(&TIM15->ARR);
    out_dma()->CMAR = dma_addr(g_out_arr);
    in_dma()->CCR = 0;
    in_dma()->CPAR = dma_addr(&TIM2->CCR4);
    in_dma()->CMAR = dma_addr(g_in_ring);
}

namespace detail {

// TIM15 keeps running, so a PF9 capture for PPM in is not lost while the output starts.
inline void start_out() {
    TIM15->DIER &= ~TIM_DIER_UDE;
    out_dma()->CCR = 0;
    // Load the sync gap into the active and the preload ARR, so the output starts with two gap periods (read as
    // frame separators) while the DMA fills the preload from the buffer at every update.
    TIM15->ARR = g_out_arr[logic::trainer::kPpmOutChannels];
    TIM15->CR1 |= TIM_CR1_ARPE;
    TIM15->EGR = TIM_EGR_UG;
    TIM15->SR = 0;
    out_dma()->CNDTR = static_cast<uint32_t>(logic::trainer::kPpmOutPeriods);
    out_dma()->CCR = kOutDmaCcr | DMA_CCR_EN;
    TIM15->DIER |= TIM_DIER_UDE;
    TIM15->CCER |= TIM_CCER_CC2E;
    PpmOutPin::set_mode(Mode::Alternate);
}

inline void stop_out() {
    PpmOutPin::write(true);
    PpmOutPin::set_mode(Mode::Output);
    TIM15->CCER &= ~TIM_CCER_CC2E;
    TIM15->DIER &= ~TIM_DIER_UDE;
    out_dma()->CCR = 0;
}

inline void start_in() {
    TIM2->CR1 = 0;
    TIM2->DIER = 0;
    in_dma()->CCR = 0;
    in_dma()->CNDTR = static_cast<uint32_t>(kInRingSize);
    in_dma()->CCR = kInDmaCcr | DMA_CCR_EN;
    g_decoder.reset();
    g_in_tail = in_head();
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_CC4DE;
    TIM2->CR1 = TIM_CR1_CEN;
    g_last_capture = TIM2->CNT;
}

inline void stop_in() {
    TIM2->DIER = 0;
    TIM2->CR1 = 0;
    in_dma()->CCR = 0;
}

inline void poll_in() {
    const std::size_t head = in_head();
    std::size_t tail = g_in_tail;
    while (tail != head) {
        const uint32_t capture = g_in_ring[tail];
        tail = (tail + 1) & (kInRingSize - 1);
        const uint32_t delta = capture - g_last_capture;
        g_last_capture = capture;
        g_decoder.process_pulse(logic::trainer::PulseUs{static_cast<uint16_t>(delta > 0xFFFF ? 0xFFFF : delta)});
    }
    g_in_tail = tail;
}

}  // namespace detail

/// Starts or stops PPM in and out. Stopping the output drives PF10 high (idle line).
inline void set_enabled(const bool in, const bool out) {
    if (out != g_out_enabled) {
        out ? detail::start_out() : detail::stop_out();
    }
    if (in != g_in_enabled) {
        in ? detail::start_in() : detail::stop_in();
    }
    g_in_enabled = in;
    g_out_enabled = out;
    // TIM15 runs for the output periods and for the PF9 capture that feeds TIM2.
    if (in || out) {
        TIM15->CR1 |= TIM_CR1_CEN;
    } else {
        TIM15->CR1 &= ~TIM_CR1_CEN;
    }
}

/// Decodes the edges captured since the last call, then advances the student signal-loss timer by dt_ms.
inline void tick(const logic::trainer::DeltaMs dt) {
    if (g_in_enabled) {
        detail::poll_in();
    }
    g_decoder.tick(dt);
}

/// Latest valid student frame, or nullopt once the signal is lost.
inline std::optional<logic::trainer::InChannels> student_channels() {
    return g_decoder.channels();
}

}  // namespace wm::hw::trainer
