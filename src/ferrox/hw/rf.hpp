#pragma once

// AFHDS 2A radio glue: owns the Radio state machine, TIM16 (3.85 ms frame
// tick) and the GIO2 EXTI line. Main-loop access goes through CriticalSection,
// which masks the ISRs, so the ISRs can read the shared fields without a lock.

#include <array>
#include <cstdint>
#include <optional>

#include "ferrox/arch/critical.hpp"
#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/a7105.hpp"
#include "ferrox/logic/afhds2a.hpp"
#include "ferrox/logic/afhds2a_radio.hpp"
#include "ferrox/time/systick.hpp"

namespace ferrox::hw::rf {

// Raw NVIC priority byte (0x80). CMSIS NVIC_SetPriority would shift it by 6 on Cortex-M0.
inline constexpr uint8_t kIrqPriorityRaw = 0x80;
// PSC=47 gives 1 us ticks; ARR=3849 gives the 3850 us frame period.
inline constexpr uint16_t kTimerPrescaler = 47;
inline constexpr uint16_t kTimerPeriodUs = 3849;
inline constexpr uint32_t kGio2Line = 2;
inline constexpr uint32_t kSettleCycles = 72000;

// ISR and main share these. Main writes them only inside CriticalSection.
inline constinit std::optional<logic::afhds2a::Radio> g_radio{};
inline constinit volatile bool g_silenced = true;
inline constinit volatile bool g_published = false;
inline constinit volatile uint16_t g_heartbeat = 0;
// ISR-only state.
inline constinit logic::afhds2a::Heartbeat g_gate{};
inline constinit bool g_antenna_state = false;

inline void gio2_irq_enable() { EXTI->IMR |= 1u << kGio2Line; }
inline void gio2_irq_disable() { EXTI->IMR &= ~(1u << kGio2Line); }

inline void set_irq_priority(const IRQn_Type irq) {
    reinterpret_cast<volatile uint8_t*>(&NVIC->IP[0])[irq] = kIrqPriorityRaw;
}

inline void init_gio2_exti() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // EXTI2 source = port B (value 1 in EXTICR[0] bits 11:8).
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xFu << 8)) | (1u << 8);
    EXTI->FTSR |= 1u << kGio2Line;
}

inline void init_timer() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->CR1 = 0;
    TIM16->PSC = kTimerPrescaler;
    TIM16->ARR = kTimerPeriodUs;
    TIM16->SR = 0;
    TIM16->DIER = TIM_DIER_UIE;
}

/// Brings up SPI, the A7105 and the frame timer. Radio stays silenced until set_silenced(false).
inline bool init(const logic::afhds2a::TxId tx) {
    g_radio.emplace(tx);
    a7105::init_bus();
    init_gio2_exti();
    if (!a7105::reset()) {
        return false;
    }
    a7105::init_regs();

    set_irq_priority(EXTI2_3_IRQn);
    set_irq_priority(TIM16_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(TIM16_IRQn);
    init_timer();
    TIM16->CR1 |= TIM_CR1_CEN;
    return true;
}

inline void set_silenced(const bool silenced) {
    if (silenced) {
        arch::CriticalSection cs;
        g_silenced = true;
        gio2_irq_disable();
        a7105::set_front_end(logic::afhds2a::FrontEnd::Off);
        a7105::strobe(logic::a7105::kStrobeSleep);
        return;
    }
    {
        arch::CriticalSection cs;
        a7105::strobe(logic::a7105::kStrobeStandby);
    }
    for (uint32_t i = 0; i < kSettleCycles; ++i) {
        __NOP();
    }
    arch::CriticalSection cs;
    g_silenced = false;
}

/// Publishes one channel frame and advances the main-loop heartbeat.
inline void publish_channels(const std::array<uint16_t, logic::afhds2a::kNumChannels>& ch) {
    arch::CriticalSection cs;
    g_radio->set_channels(ch);
    g_heartbeat = static_cast<uint16_t>(g_heartbeat + 1);
    g_published = true;
}

inline void set_bind_mode(const bool on) {
    arch::CriticalSection cs;
    if (auto power = g_radio->set_bind_mode(on)) {
        a7105::set_power(*power);
    }
}

inline void set_rf_power(const uint8_t idx) {
    arch::CriticalSection cs;
    if (auto power = g_radio->set_rf_power(idx)) {
        a7105::set_power(*power);
    }
}

inline void set_rx_id(const logic::afhds2a::RxId id) {
    arch::CriticalSection cs;
    g_radio->set_rx_id(id);
}

inline void set_rx_settings(const uint16_t rate_hz, const logic::afhds2a::PpmOut ppm_out,
                            const logic::afhds2a::SbusOut sbus) {
    arch::CriticalSection cs;
    g_radio->set_rx_settings(rate_hz, ppm_out, sbus);
}

inline bool is_bound() {
    arch::CriticalSection cs;
    return g_radio->link.bind_done;
}

inline bool is_binding() {
    arch::CriticalSection cs;
    return g_radio->binding();
}

inline logic::afhds2a::RxId rx_id() {
    arch::CriticalSection cs;
    return g_radio->link.rx_id;
}

inline std::optional<logic::afhds2a::RxId> take_pending_rx_save() {
    arch::CriticalSection cs;
    return g_radio->take_rx_save();
}

inline logic::afhds2a::Telemetry get_telemetry() {
    arch::CriticalSection cs;
    return g_radio->link.telemetry;
}

inline uint32_t packets_sent() {
    arch::CriticalSection cs;
    return g_radio->packets_sent;
}

/// TIM16 update: build and send the next frame.
inline void on_tim16() {
    TIM16->SR = 0;
    if (!g_radio.has_value() || g_silenced || !g_published) {
        return;
    }
    if (!g_gate.allow_tx(g_heartbeat)) {
        return;
    }

    gio2_irq_disable();
    const auto plan = g_radio->on_timer_tick();
    if (plan.toggle_antenna) {
        g_antenna_state = !g_antenna_state;
        a7105::switch_antenna(g_antenna_state);
    }
    if (plan.power_reg) {
        a7105::set_power(*plan.power_reg);
    }
    a7105::write_fifo(plan.packet, plan.channel);
    gio2_irq_enable();
}

/// GIO2 falling edge: TX finished, or an RX packet is ready.
inline void on_exti2_3() {
    if ((EXTI->PR & (1u << kGio2Line)) == 0) {
        return;
    }
    EXTI->PR = 1u << kGio2Line;
    gio2_irq_disable();

    const auto plan = g_radio->on_gio2_event();
    switch (plan.step) {
        case logic::afhds2a::GioStep::StartRx:
            a7105::set_front_end(plan.front);
            a7105::strobe(logic::a7105::kStrobeRx);
            gio2_irq_enable();
            break;
        case logic::afhds2a::GioStep::ReceiveDone:
            if (a7105::is_crc_ok()) {
                logic::afhds2a::Frame rx{};
                a7105::read_fifo(rx);
                g_radio->on_rx_frame(rx, a7105::read_raw_rssi(), time::millis());
            }
            a7105::strobe(logic::a7105::kStrobeStandby);
            break;
        case logic::afhds2a::GioStep::None:
            break;
    }
}

}  // namespace ferrox::hw::rf
