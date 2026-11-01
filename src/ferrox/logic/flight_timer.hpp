#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

#include "ferrox/logic/buzzer.hpp"
#include "ferrox/logic/flight.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/vario.hpp"

namespace ferrox::logic::flight_timer {

inline constexpr uint16_t kCountdownSecs = 10;
inline constexpr uint16_t kMinuteBeepSecs = 60;
inline constexpr uint32_t kMsPerSec = 1000;
inline constexpr uint32_t kCancelHoldMs = 1000;
// The progress modal stays hidden for the first 80 ms of a hold so a tap does not flash it.
inline constexpr uint32_t kCancelModalMinMs = 80;
inline constexpr uint32_t kToastMs = 800;
inline constexpr uint32_t kCooldownMs = 1500;
inline constexpr uint8_t kArmSwitchMax = 22;

struct Seconds {
    uint16_t value = 0;
    friend constexpr bool operator==(const Seconds&, const Seconds&) = default;
};

/// Model slot index; 0xFF means no model has been seen yet, so the first tick resets the timer.
struct ModelSlot {
    uint8_t value = 0xFF;
    friend constexpr bool operator==(const ModelSlot&, const ModelSlot&) = default;
};

struct TimerSource {
    uint8_t value = 0;
    friend constexpr bool operator==(const TimerSource&, const TimerSource&) = default;
};

struct ArmSwitch {
    uint8_t value = 0;
    friend constexpr bool operator==(const ArmSwitch&, const ArmSwitch&) = default;
};

/// Timer settings from the active model.
struct Config {
    Seconds limit{};
    TimerSource source{};
    ArmSwitch arm{};
    friend constexpr bool operator==(const Config&, const Config&) = default;
};

struct State {
    ModelSlot model{};
    Seconds remaining{};
    Seconds elapsed{};
    uint32_t acc_ms = 0;
    mixer::TimerLatch latched{};
    uint32_t toast_ms = 0;
    uint32_t cooldown_ms = 0;
    uint32_t cancel_hold_ms = 0;
    bool cancel_waiting_release = false;
};

struct Input {
    ModelSlot model{};
    Config config{};
    bool was_armed = false;
    bool menu_active = false;
    bool cancel_held = false;
    int16_t throttle = 0;
    Switches switches{};
    vario::Millis dt{};
};

struct Display {
    Seconds value{};
    bool expired = false;
    friend constexpr bool operator==(const Display&, const Display&) = default;
};

/// Reset progress overlay: `reset_done` shows the confirmation, otherwise `progress_pct` fills the bar.
struct Modal {
    bool reset_done = false;
    uint8_t progress_pct = 0;
    friend constexpr bool operator==(const Modal&, const Modal&) = default;
};

constexpr void reset_counters(State& s, Seconds limit) {
    s.remaining = limit;
    s.elapsed = Seconds{};
    s.acc_ms = 0;
    s.latched = {};
}

// Manual reset from the cancel hold also starts the confirmation toast and a cooldown.
constexpr void manual_reset(State& s, Seconds limit) {
    reset_counters(s, limit);
    s.cancel_hold_ms = 0;
    s.cancel_waiting_release = true;
    s.toast_ms = kToastMs;
    s.cooldown_ms = kCooldownMs;
}

constexpr uint32_t sat_sub(uint32_t a, uint32_t b) {
    return a > b ? a - b : 0;
}

// Advances the countdown by one second and returns the beep due for it.
constexpr buzzer::Cue count_second(State& s) {
    if (s.remaining.value == 0) {
        if (s.elapsed.value < UINT16_MAX) {
            ++s.elapsed.value;
        }
        return buzzer::Cue::None;
    }
    --s.remaining.value;
    if (s.remaining.value == 0) {
        return buzzer::Cue::TimerElapsed;
    }
    if (s.remaining.value <= kCountdownSecs) {
        return buzzer::Cue::TimerCountdown;
    }
    if (s.remaining.value % kMinuteBeepSecs == 0) {
        return buzzer::Cue::TimerMinute;
    }
    return buzzer::Cue::None;
}

/// One engine tick. Returns the buzzer cue to play, or None.
constexpr buzzer::Cue update(State& s, const Input& in) {
    buzzer::Cue cue = buzzer::Cue::None;

    if (s.model != in.model) {
        s.model = in.model;
        reset_counters(s, in.config.limit);
    }

    const bool assigned = in.config.arm.value >= 1 && in.config.arm.value <= kArmSwitchMax;
    const bool armed = flight::is_armed(in.config.arm.value, in.switches);
    if (assigned) {
        if (armed && !in.was_armed) {
            reset_counters(s, in.config.limit);
        } else if (!armed && in.was_armed) {
            s.latched = {};
        }
    }

    const uint32_t dt = in.dt.value;
    s.toast_ms = sat_sub(s.toast_ms, dt);
    s.cooldown_ms = sat_sub(s.cooldown_ms, dt);
    if (!in.cancel_held) {
        s.cancel_waiting_release = false;
    }

    if (!in.menu_active && in.cancel_held && !s.cancel_waiting_release && s.cooldown_ms == 0) {
        s.cancel_hold_ms += dt;
        if (s.cancel_hold_ms >= kCancelHoldMs) {
            manual_reset(s, in.config.limit);
            cue = buzzer::Cue::TimerReset;
        }
    } else {
        s.cancel_hold_ms = 0;
    }

    const bool running = mixer::is_timer_active(in.config.source.value, in.throttle, s.latched, in.switches,
                                                mixer::TimerArmed{armed});
    if (running && dt > 0) {
        s.acc_ms += dt;
        while (s.acc_ms >= kMsPerSec) {
            s.acc_ms -= kMsPerSec;
            const buzzer::Cue tick = count_second(s);
            if (cue == buzzer::Cue::None) {
                cue = tick;
            }
        }
    }

    return cue;
}

/// Footer value: nothing when the timer is off, otherwise remaining time, or elapsed time once it has run out.
constexpr std::optional<Display> display(const State& s, const Config& c) {
    if (c.source.value == 0) {
        return std::nullopt;
    }
    if (c.limit.value == 0) {
        return Display{s.elapsed, false};
    }
    if (s.remaining.value > 0) {
        return Display{s.remaining, false};
    }
    return Display{s.elapsed, true};
}

constexpr std::optional<Modal> modal(const State& s) {
    if (s.toast_ms > 0) {
        return Modal{true, 100};
    }
    if (s.cancel_hold_ms > kCancelModalMinMs) {
        const auto pct = static_cast<uint8_t>(std::min<uint32_t>(s.cancel_hold_ms * 100 / kCancelHoldMs, 100));
        return Modal{false, pct};
    }
    return std::nullopt;
}

}  // namespace ferrox::logic::flight_timer
