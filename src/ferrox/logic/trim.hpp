#pragma once

#include <algorithm>
#include <bit>
#include <cstdint>
#include <optional>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/vario.hpp"

namespace ferrox::logic::trim {

inline constexpr int8_t kTrimMin = -25;
inline constexpr int8_t kTrimMax = 25;
inline constexpr int16_t kTrimStepUs = 4;
inline constexpr int16_t kTrimMaxOffsetUs = kTrimMax * kTrimStepUs;

/// Trim values of the four primary flight channels.
struct TrimValues {
    int8_t roll{0};      // CH1 (AIL)
    int8_t pitch{0};     // CH2 (ELE)
    int8_t throttle{0};  // CH3 (THR)
    int8_t yaw{0};       // CH4 (RUD)
    constexpr bool operator==(const TrimValues&) const = default;
};

/// Adds the linear trim offset to a pulse width and clamps it to the channel range.
constexpr uint16_t apply(const uint16_t base_us, const int8_t trim) {
    const int32_t offset = static_cast<int32_t>(trim) * kTrimStepUs;
    return static_cast<uint16_t>(std::clamp<int32_t>(base_us + offset, kChannelMinUs, kChannelMaxUs));
}

/// Throttle trim: mode 0 = locked, 1 = idle trim (tapers to 0 at full stick), 2 = linear.
constexpr uint16_t apply_throttle(const uint16_t base_us, const int8_t trim, const uint8_t mode) {
    switch (mode) {
        case 1: {
            const int32_t max_offset = static_cast<int32_t>(trim) * kTrimStepUs;
            const int32_t travel = std::clamp<int32_t>(base_us - kChannelMinUs, 0, kChannelSpanUs);
            const int32_t effective = (max_offset * (kChannelSpanUs - travel)) / kChannelSpanUs;
            const int32_t lo = kChannelMinUs - kTrimMaxOffsetUs;
            const int32_t hi = kChannelMaxUs + kTrimMaxOffsetUs;
            return static_cast<uint16_t>(std::clamp<int32_t>(base_us + effective, lo, hi));
        }
        case 2:
            return apply(base_us, trim);
        default:
            return static_cast<uint16_t>(std::clamp<uint16_t>(base_us, kChannelMinUs, kChannelMaxUs));
    }
}

/// Trim axes in Rust `ActiveTrim` order (without None).
enum class Axis : uint8_t { Roll, Pitch, Throttle, Yaw };

/// Footer letter of an axis (Rust `format_trim`).
constexpr char axis_letter(const Axis axis) {
    switch (axis) {
        case Axis::Roll: return 'A';
        case Axis::Pitch: return 'E';
        case Axis::Throttle: return 'T';
        case Axis::Yaw: return 'R';
    }
    return ' ';
}

constexpr int8_t value_of(const TrimValues& values, const Axis axis) {
    switch (axis) {
        case Axis::Roll: return values.roll;
        case Axis::Pitch: return values.pitch;
        case Axis::Throttle: return values.throttle;
        case Axis::Yaw: return values.yaw;
    }
    return 0;
}

constexpr int8_t& value_of(TrimValues& values, const Axis axis) {
    switch (axis) {
        case Axis::Roll: return values.roll;
        case Axis::Pitch: return values.pitch;
        case Axis::Throttle: return values.throttle;
        case Axis::Yaw: return values.yaw;
    }
    return values.roll;
}

/// Audio result of one trim update. At most one per update.
struct Event {
    enum class Kind : uint8_t { None, Step, Center, Limit };
    Kind kind = Kind::None;
    // New trim value; set for Kind::Step only.
    int8_t value = 0;
    constexpr bool operator==(const Event&) const = default;
};

/// Stick deflection (-1000..+1000) of the three axes that instant trim reads.
struct InstantSticks {
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;
    constexpr bool operator==(const InstantSticks&) const = default;
};

/// Outcome of one instant trim press. Adjusted and Reset each have their own buzzer cue.
enum class InstantResult : uint8_t { None, Adjusted, Reset };

/// Trim steps for one stick deflection: 40 counts per step, rounded half away from zero (Rust `round_step`).
constexpr int8_t instant_step(const int16_t stick) {
    const int32_t v = stick;
    return static_cast<int8_t>(v >= 0 ? (v + 20) / 40 : (v - 20) / 40);
}

/// Raw reading at or above which an InstantTrim switch counts as pressed (Rust `raw_val >= 2048`).
inline constexpr uint16_t kInstantTrimRaw = 2048;

/// True while an auxiliary channel set to InstantTrim is switched down. Only the first 10 aux channels
/// are checked (6 without the external ADC), as in Rust.
constexpr bool instant_switch_down(const storage::RadioConfig& radio, const input::Raw& raw) {
    const std::size_t count = radio.ext_adc != 0 ? input::kAuxCount : 6;
    for (std::size_t ch = 0; ch < count; ++ch) {
        if (input::resolve_mode(ch, radio.adc_modes[ch]) == input::AdcMode::InstantTrim &&
            raw[input::aux_index_to_raw(ch)] >= kInstantTrimRaw) {
            return true;
        }
    }
    return false;
}

/// Rising-edge detector for the instant trim switches (Rust `prev_instant_trim_active`).
/// It runs on every tick, also while a menu is open, so the edge is not missed.
struct InstantEdge {
    bool prev = false;

    constexpr bool update(const bool active) {
        const bool rising = active && !prev;
        prev = active;
        return rising;
    }
};

/// Trim key state machine (Rust `TrimController`). Keys 0..7 of the key matrix are the
/// trim rockers: Roll +/-, Pitch +/-, Throttle +/-, Yaw +/-.
struct Controller {
    // Rust: hold 350 ms before auto-repeat, then one step every 90 ms.
    static constexpr uint32_t kHoldDelayMs = 350;
    static constexpr uint32_t kRepeatMs = 90;
    // Rust: the footer shows the last changed trim for 1.5 s.
    static constexpr uint32_t kActiveMs = 1500;

    TrimValues values{};
    std::optional<Axis> last_active{};
    uint32_t active_ms = 0;
    // Rust: throttle trim is locked until the radio setup enables it.
    bool throttle_enabled = false;
    uint8_t prev_keys = 0;
    uint32_t hold_ms = 0;
    uint32_t repeat_ms = 0;

    /// Runs one update from the raw key matrix. Not called while a menu is open (as in Rust).
    constexpr Event update(const uint16_t keys, const vario::Millis dt) {
        if (active_ms > 0) {
            active_ms = active_ms > dt.value ? active_ms - dt.value : 0;
            if (active_ms == 0) {
                last_active.reset();
            }
        }

        // Both horizontal trims inward is the DFU request, so trims are locked out.
        if (is_dfu_requested(keys)) {
            release();
            return {};
        }

        const auto trim_keys = static_cast<uint8_t>(keys & 0xFFu);
        if (trim_keys == 0) {
            release();
            return {};
        }

        std::optional<uint8_t> trigger{};
        if (trim_keys != prev_keys) {
            const auto newly = static_cast<uint8_t>(trim_keys & ~prev_keys);
            if (newly != 0) {
                trigger = static_cast<uint8_t>(std::countr_zero(newly));
            }
            prev_keys = trim_keys;
            hold_ms = 0;
            repeat_ms = 0;
        } else {
            hold_ms += dt.value;
            if (hold_ms >= kHoldDelayMs) {
                repeat_ms += dt.value;
                if (repeat_ms >= kRepeatMs) {
                    repeat_ms = 0;
                    trigger = static_cast<uint8_t>(std::countr_zero(trim_keys));
                }
            }
        }

        if (!trigger) {
            return {};
        }
        return step_key(*trigger);
    }

    /// Steps the trim of one key: even bits add a step, odd bits subtract one.
    constexpr Event step_key(const uint8_t key) {
        const auto axis = static_cast<Axis>(key / 2);
        const int8_t delta = (key % 2 == 0) ? 1 : -1;
        return step(axis, delta);
    }

    constexpr Event step(const Axis axis, const int8_t delta) {
        if (axis == Axis::Throttle && !throttle_enabled) {
            return {Event::Kind::Limit, 0};
        }

        int8_t& slot = value_of(values, axis);
        if ((delta > 0 && slot >= kTrimMax) || (delta < 0 && slot <= kTrimMin)) {
            return {Event::Kind::Limit, 0};
        }

        slot = static_cast<int8_t>(std::clamp<int>(slot + delta, kTrimMin, kTrimMax));
        last_active = axis;
        active_ms = kActiveMs;
        if (slot == 0) {
            return {Event::Kind::Center, 0};
        }
        return {Event::Kind::Step, slot};
    }

    /// Instant trim (Rust `main.rs` section 7), run on the rising edge of an InstantTrim switch outside menus.
    /// Sticks off centre add their rounded deflection to roll, pitch and yaw. Sticks centred reset those three
    /// trims if any is set. Throttle is not changed.
    constexpr InstantResult instant_trim(const InstantSticks& sticks) {
        const int8_t d_roll = instant_step(sticks.roll);
        const int8_t d_pitch = instant_step(sticks.pitch);
        const int8_t d_yaw = instant_step(sticks.yaw);
        if (d_roll != 0 || d_pitch != 0 || d_yaw != 0) {
            add_clamped(values.roll, d_roll);
            add_clamped(values.pitch, d_pitch);
            add_clamped(values.yaw, d_yaw);
            last_active = d_roll != 0 ? Axis::Roll : (d_pitch != 0 ? Axis::Pitch : Axis::Yaw);
            active_ms = kActiveMs;
            return InstantResult::Adjusted;
        }
        if (values.roll != 0 || values.pitch != 0 || values.yaw != 0) {
            values.roll = 0;
            values.pitch = 0;
            values.yaw = 0;
            last_active = Axis::Roll;
            active_ms = kActiveMs;
            return InstantResult::Reset;
        }
        return InstantResult::None;
    }

private:
    static constexpr void add_clamped(int8_t& slot, const int8_t delta) {
        slot = static_cast<int8_t>(std::clamp<int>(slot + delta, kTrimMin, kTrimMax));
    }

    constexpr void release() {
        prev_keys = 0;
        hold_ms = 0;
        repeat_ms = 0;
    }
};

/// Trim auto-save trigger (Rust `main.rs` section 8). Any trim change restarts a 2000 ms cooldown.
/// The model is written once the cooldown has run out, the radio is disarmed and the trims are dirty.
/// A failed write keeps them dirty, so the next tick retries, as in Rust.
struct AutoSave {
    // Rust: trim_save_cooldown_ms = 2000 on every change.
    static constexpr uint32_t kDelayMs = 2000;

    TrimValues last{};
    bool dirty = false;
    uint32_t cooldown_ms = 0;

    /// Marks `values` as already stored (boot and model switch), so loading does not count as a change.
    constexpr void reset(const TrimValues& values) {
        last = values;
        dirty = false;
        cooldown_ms = 0;
    }

    /// Runs one tick. Returns true when the caller should write the model now.
    /// `armed` is the arm state of the previous tick (Rust `pipeline.prev_armed`).
    constexpr bool tick(const TrimValues& values, const bool armed, const vario::Millis dt) {
        cooldown_ms -= std::min(cooldown_ms, static_cast<uint32_t>(dt.value));
        if (values != last) {
            last = values;
            dirty = true;
            cooldown_ms = kDelayMs;
        }
        return dirty && cooldown_ms == 0 && !armed;
    }

    /// Records the result of a write started because `tick` returned true.
    constexpr void saved(const bool ok) {
        if (ok) {
            dirty = false;
        }
    }
};

}  // namespace ferrox::logic::trim
