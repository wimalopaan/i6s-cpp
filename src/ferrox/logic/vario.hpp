#pragma once

// Pure variometer logic (Rust `vario.rs`): vertical speed filtering from
// telemetry or altitude differences, deadband gating, motor mute, and the
// tone frequency and cadence. The buzzer output is in the hardware layer and
// is not part of this header.

#include <algorithm>
#include <cstdint>

namespace ferrox::logic::vario {

// Strong types so altitude, vertical speed, time and throttle cannot be mixed up at a call site.
struct Millis {
    uint32_t value = 0;
    constexpr bool operator==(const Millis&) const = default;
};

struct AltitudeM {
    int16_t value = 0;
    constexpr bool operator==(const AltitudeM&) const = default;
};

struct AltitudeDm {
    int16_t value = 0;
    constexpr bool operator==(const AltitudeDm&) const = default;
};

struct VerticalSpeedCms {
    int16_t value = 0;
    constexpr bool operator==(const VerticalSpeedCms&) const = default;
};

struct ThrottleUs {
    uint16_t value = 0;
    constexpr bool operator==(const ThrottleUs&) const = default;
};

// Deadband thresholds in 0.1 m/s (10 cm/s) units.
struct DeadbandDmS {
    uint8_t value = 0;
    constexpr bool operator==(const DeadbandDmS&) const = default;
};

enum class SinkMode : uint8_t { Mute = 0, Pulse = 1, Continuous = 2 };

constexpr SinkMode sink_mode_from_u8(uint8_t val) {
    switch (val) {
        case 1:
            return SinkMode::Pulse;
        case 2:
            return SinkMode::Continuous;
        default:
            return SinkMode::Mute;
    }
}

struct Config {
    // 0: Off, 1: On (always active), 2..=23: switch conditions (Rust: 2..=23 map to switch_cond 1..=22). Matched by the caller.
    uint8_t switch_index = 0;
    DeadbandDmS climb_deadband{2};  // +0.2 m/s
    DeadbandDmS sink_deadband{5};   // -0.5 m/s
    SinkMode sink_mode = SinkMode::Pulse;
    bool motor_mute = true;
};

struct Tone {
    uint16_t freq_hz = 0;
    uint16_t pulse_ms = 0;
    uint16_t pause_ms = 0;

    constexpr bool is_silent() const { return freq_hz < 100 || pulse_ms == 0; }
    constexpr bool operator==(const Tone&) const = default;
};

inline constexpr Tone kSilent{};

// One telemetry snapshot plus the link and switch state for this tick.
struct Sample {
    Millis now{};
    AltitudeM alt_m{};
    AltitudeDm alt_dm{};
    VerticalSpeedCms vspeed{};
    bool has_native_vspeed = false;
    bool telem_connected = false;
    bool switch_active = false;
    ThrottleUs throttle{};
};

struct State {
    VerticalSpeedCms vspeed{};
    AltitudeM alt{};
    AltitudeDm last_alt_dm{};
    Millis last_alt_ms{};
    bool initialized = false;

    constexpr void reset() { *this = State{}; }

    constexpr Tone update(const Sample& s, const Config& cfg) {
        if (!s.telem_connected) {
            reset();
            return kSilent;
        }

        alt = s.alt_m;

        if (!initialized) {
            last_alt_dm = s.alt_dm;
            last_alt_ms = s.now;
            vspeed = s.has_native_vspeed ? s.vspeed : VerticalSpeedCms{0};
            initialized = true;
        }

        const uint32_t dt_ms = elapsed_ms(s.now, last_alt_ms);
        if (s.has_native_vspeed) {
            // Telemetry vspeed is filtered toward the reading, including 0 in level flight.
            vspeed.value = static_cast<int16_t>((static_cast<int32_t>(vspeed.value) * 3 + s.vspeed.value) / 4);
            last_alt_dm = s.alt_dm;
            last_alt_ms = s.now;
        } else if (s.alt_dm.value != last_alt_dm.value && s.now.value > last_alt_ms.value) {
            if (dt_ms >= 80) {
                const int32_t dh_dm = static_cast<int32_t>(s.alt_dm.value) - last_alt_dm.value;
                // dh_dm * 10 is cm, so (dh_dm * 10 * 1000) / dt_ms is cm/s.
                const int32_t derived_cms = (dh_dm * 10'000) / static_cast<int32_t>(dt_ms);
                vspeed.value = static_cast<int16_t>((static_cast<int32_t>(vspeed.value) * 3 + derived_cms) / 4);
                last_alt_dm = s.alt_dm;
                last_alt_ms = s.now;
            }
        } else if (dt_ms > 500) {
            vspeed.value = static_cast<int16_t>(static_cast<int32_t>(vspeed.value) * 3 / 4);
            last_alt_ms = s.now;
        }

        if (!s.switch_active) {
            return kSilent;
        }

        // Motor lockout above 10 % throttle (1100 µs).
        if (cfg.motor_mute && s.throttle.value > 1100) {
            return kSilent;
        }

        return compute_tone(cfg);
    }

    constexpr Tone compute_tone(const Config& cfg) const {
        const int32_t climb_threshold = static_cast<int32_t>(cfg.climb_deadband.value) * 10;
        const int32_t sink_threshold = static_cast<int32_t>(cfg.sink_deadband.value) * 10;
        const int32_t v = vspeed.value;

        if (v >= climb_threshold) {
            // Lift: 1000 Hz at the threshold up to 2800 Hz at +5 m/s; cadence 550 ms down to 110 ms.
            const int32_t delta = std::clamp(v - climb_threshold, 0, 500);
            const auto freq_hz = static_cast<uint16_t>(1000 + (delta * 1800) / 500);
            const auto period_ms = static_cast<uint16_t>(550 - (delta * 440) / 500);
            const auto pulse_ms = static_cast<uint16_t>(period_ms / 2);
            return Tone{freq_hz, pulse_ms, static_cast<uint16_t>(period_ms - pulse_ms)};
        }

        if (v <= -sink_threshold) {
            if (cfg.sink_mode == SinkMode::Mute) {
                return kSilent;
            }
            const int32_t sink_mag = std::clamp(-v - sink_threshold, 0, 500);
            const auto freq_hz = static_cast<uint16_t>(450 - (sink_mag * 100) / 500);
            if (cfg.sink_mode == SinkMode::Pulse) {
                return Tone{freq_hz, 160, 360};
            }
            return Tone{freq_hz, 1000, 0};
        }

        return kSilent;
    }

    static constexpr uint32_t elapsed_ms(Millis now, Millis since) {
        return now.value >= since.value ? now.value - since.value : 0;
    }
};

}  // namespace ferrox::logic::vario
