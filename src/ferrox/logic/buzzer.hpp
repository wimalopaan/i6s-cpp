#pragma once

// Pure buzzer logic (Rust `buzzer.rs` and its callers): tone presets in Simple and Rich
// styles, the foreground note and pattern player with the background vario stream, the
// alarm cadences for battery and RSSI, the inactivity and preflight timers, the arm chime
// decision, and the 1 MHz PWM timing. The TIM1 output is in the hardware layer.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "vario.hpp"

namespace ferrox::logic::buzzer {

// Strong types so frequencies, durations and counters cannot be mixed up at a call site.
struct FreqHz {
    uint16_t value = 0;
    constexpr bool operator==(const FreqHz&) const = default;
};

struct DurationMs {
    uint16_t value = 0;
    constexpr bool operator==(const DurationMs&) const = default;
};

struct PauseMs {
    uint16_t value = 0;
    constexpr bool operator==(const PauseMs&) const = default;
};

struct Repeats {
    uint8_t value = 0;
    constexpr bool operator==(const Repeats&) const = default;
};

struct ElapsedMs {
    uint32_t value = 0;
    constexpr bool operator==(const ElapsedMs&) const = default;
};

struct TrimStep {
    int8_t value = 0;
    constexpr bool operator==(const TrimStep&) const = default;
};

struct BatteryMv {
    uint16_t value = 0;
    constexpr bool operator==(const BatteryMv&) const = default;
};

// Battery warning threshold in 0.1 V units, as stored in the radio config.
struct WarnDeci {
    uint8_t value = 0;
    constexpr bool operator==(const WarnDeci&) const = default;
};

struct Rssi {
    uint8_t value = 0;
    constexpr bool operator==(const Rssi&) const = default;
};

// Arm switch condition index: 1..=22 are real switch conditions, anything else is not armable.
struct SwitchIndex {
    uint8_t value = 0;
    constexpr bool operator==(const SwitchIndex&) const = default;
};

enum class ToneStyle : uint8_t { Simple = 0, Rich = 1 };

constexpr ToneStyle tone_style_from_u8(uint8_t val) { return val == 0 ? ToneStyle::Simple : ToneStyle::Rich; }

struct Note {
    FreqHz freq{};
    DurationMs duration{};
    PauseMs pause{};
    constexpr bool operator==(const Note&) const = default;
};

// Plays `notes` in order, then repeats the whole list `repeats` more times.
struct Pattern {
    std::span<const Note> notes{};
    Repeats repeats{};
};

inline constexpr uint16_t kBeepDefaultHz = 2250;
inline constexpr uint16_t kBeepCenterHz = 2800;
inline constexpr uint16_t kBeepLimitHz = 1100;
inline constexpr uint32_t kMinAudibleHz = 100;

enum class Cue : uint8_t {
    None,
    Click,
    Welcome,
    Armed,
    Disarmed,
    TrimCenter,
    TrimLimit,
    WarnBattery,
    WarnCritical,
    WarnPreflight,
    WarnInactivity,
    WarnRssiLow,
    WarnRssiCritical,
    CalibStart,
    CalibAdvance,
    CalibReject,
    CalibSuccess,
    PotCenter,
    TimerMinute,
    TimerCountdown,
    TimerElapsed,
    NavStep,
};

namespace tables {

inline constexpr std::array<Note, 1> kClick{{{FreqHz{kBeepDefaultHz}, DurationMs{15}, PauseMs{0}}}};
inline constexpr std::array<Note, 4> kWelcome{{
    {FreqHz{1046}, DurationMs{110}, PauseMs{25}},
    {FreqHz{1318}, DurationMs{110}, PauseMs{25}},
    {FreqHz{1568}, DurationMs{120}, PauseMs{30}},
    {FreqHz{2093}, DurationMs{240}, PauseMs{0}},
}};
inline constexpr std::array<Note, 2> kArmed{{{FreqHz{1800}, DurationMs{45}, PauseMs{10}}, {FreqHz{2600}, DurationMs{75}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kDisarmed{{{FreqHz{2600}, DurationMs{45}, PauseMs{10}}, {FreqHz{1800}, DurationMs{75}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kArmedSimple{{{FreqHz{2400}, DurationMs{80}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kDisarmedSimple{{{FreqHz{1800}, DurationMs{80}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kTrimCenterSimple{{{FreqHz{kBeepCenterHz}, DurationMs{60}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kTrimCenter{{{FreqHz{2400}, DurationMs{30}, PauseMs{10}}, {FreqHz{3000}, DurationMs{50}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kTrimLimitSimple{{{FreqHz{kBeepLimitHz}, DurationMs{45}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kTrimLimit{{{FreqHz{1100}, DurationMs{40}, PauseMs{15}}, {FreqHz{900}, DurationMs{60}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kWarnBatterySimple{{{FreqHz{2400}, DurationMs{70}, PauseMs{50}}}};
inline constexpr std::array<Note, 3> kWarnBattery{{
    {FreqHz{2000}, DurationMs{60}, PauseMs{30}},
    {FreqHz{2300}, DurationMs{60}, PauseMs{30}},
    {FreqHz{2600}, DurationMs{90}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kWarnCriticalSimple{{{FreqHz{2800}, DurationMs{80}, PauseMs{40}}}};
inline constexpr std::array<Note, 4> kWarnCritical{{
    {FreqHz{2800}, DurationMs{70}, PauseMs{20}},
    {FreqHz{1800}, DurationMs{70}, PauseMs{20}},
    {FreqHz{2800}, DurationMs{70}, PauseMs{20}},
    {FreqHz{1800}, DurationMs{70}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kWarnPreflightSimple{{{FreqHz{2600}, DurationMs{80}, PauseMs{50}}}};
inline constexpr std::array<Note, 3> kWarnPreflight{{
    {FreqHz{2600}, DurationMs{70}, PauseMs{25}},
    {FreqHz{2600}, DurationMs{70}, PauseMs{25}},
    {FreqHz{2900}, DurationMs{90}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kWarnInactivitySimple{{{FreqHz{1800}, DurationMs{250}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kWarnInactivity{{{FreqHz{1600}, DurationMs{70}, PauseMs{30}}, {FreqHz{2000}, DurationMs{100}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kWarnRssiLowSimple{{{FreqHz{2000}, DurationMs{80}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kWarnRssiLow{{{FreqHz{1900}, DurationMs{50}, PauseMs{20}}, {FreqHz{1700}, DurationMs{70}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kWarnRssiCriticalSimple{{{FreqHz{2800}, DurationMs{50}, PauseMs{40}}}};
inline constexpr std::array<Note, 4> kWarnRssiCritical{{
    {FreqHz{2600}, DurationMs{40}, PauseMs{20}},
    {FreqHz{1800}, DurationMs{40}, PauseMs{20}},
    {FreqHz{2600}, DurationMs{40}, PauseMs{20}},
    {FreqHz{1800}, DurationMs{60}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kCalibStartSimple{{{FreqHz{2200}, DurationMs{100}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kCalibStart{{{FreqHz{2000}, DurationMs{50}, PauseMs{15}}, {FreqHz{2500}, DurationMs{80}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kCalibAdvance{{{FreqHz{2400}, DurationMs{50}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kCalibReject{{{FreqHz{1100}, DurationMs{100}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kCalibSuccessSimple{{{FreqHz{2800}, DurationMs{150}, PauseMs{0}}}};
inline constexpr std::array<Note, 3> kCalibSuccess{{
    {FreqHz{2000}, DurationMs{40}, PauseMs{15}},
    {FreqHz{2500}, DurationMs{40}, PauseMs{15}},
    {FreqHz{3100}, DurationMs{100}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kPotCenter{{{FreqHz{3200}, DurationMs{10}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kTimerMinuteSimple{{{FreqHz{2400}, DurationMs{60}, PauseMs{0}}}};
inline constexpr std::array<Note, 2> kTimerMinute{{{FreqHz{2200}, DurationMs{40}, PauseMs{15}}, {FreqHz{2800}, DurationMs{60}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kTimerCountdown{{{FreqHz{2600}, DurationMs{35}, PauseMs{0}}}};
inline constexpr std::array<Note, 1> kTimerElapsedSimple{{{FreqHz{2800}, DurationMs{150}, PauseMs{0}}}};
inline constexpr std::array<Note, 3> kTimerElapsed{{
    {FreqHz{3000}, DurationMs{70}, PauseMs{20}},
    {FreqHz{2200}, DurationMs{70}, PauseMs{20}},
    {FreqHz{3000}, DurationMs{120}, PauseMs{0}},
}};
inline constexpr std::array<Note, 1> kNavStep{{{FreqHz{2200}, DurationMs{30}, PauseMs{0}}}};

}  // namespace tables

constexpr Pattern single(std::span<const Note> notes) { return Pattern{notes, Repeats{0}}; }

// Simple style replaces the melody with a tone pattern, Rich plays the melody.
constexpr Pattern pattern_for(Cue cue, ToneStyle style) {
    const bool rich = style == ToneStyle::Rich;
    switch (cue) {
        case Cue::None: return {};
        case Cue::Click: return single(tables::kClick);
        case Cue::Welcome: return rich ? single(tables::kWelcome) : single(tables::kClick);
        case Cue::Armed: return rich ? single(tables::kArmed) : single(tables::kArmedSimple);
        case Cue::Disarmed: return rich ? single(tables::kDisarmed) : single(tables::kDisarmedSimple);
        case Cue::TrimCenter: return rich ? single(tables::kTrimCenter) : single(tables::kTrimCenterSimple);
        case Cue::TrimLimit: return rich ? single(tables::kTrimLimit) : single(tables::kTrimLimitSimple);
        case Cue::WarnBattery: return rich ? single(tables::kWarnBattery) : Pattern{tables::kWarnBatterySimple, Repeats{1}};
        case Cue::WarnCritical: return rich ? single(tables::kWarnCritical) : Pattern{tables::kWarnCriticalSimple, Repeats{2}};
        case Cue::WarnPreflight: return rich ? single(tables::kWarnPreflight) : Pattern{tables::kWarnPreflightSimple, Repeats{2}};
        case Cue::WarnInactivity: return rich ? single(tables::kWarnInactivity) : single(tables::kWarnInactivitySimple);
        case Cue::WarnRssiLow: return rich ? single(tables::kWarnRssiLow) : single(tables::kWarnRssiLowSimple);
        case Cue::WarnRssiCritical: return rich ? single(tables::kWarnRssiCritical) : Pattern{tables::kWarnRssiCriticalSimple, Repeats{2}};
        case Cue::CalibStart: return rich ? single(tables::kCalibStart) : single(tables::kCalibStartSimple);
        case Cue::CalibAdvance: return single(tables::kCalibAdvance);
        case Cue::CalibReject: return single(tables::kCalibReject);
        case Cue::CalibSuccess: return rich ? single(tables::kCalibSuccess) : single(tables::kCalibSuccessSimple);
        case Cue::PotCenter: return single(tables::kPotCenter);
        case Cue::TimerMinute: return rich ? single(tables::kTimerMinute) : single(tables::kTimerMinuteSimple);
        case Cue::TimerCountdown: return single(tables::kTimerCountdown);
        case Cue::TimerElapsed: return rich ? single(tables::kTimerElapsed) : single(tables::kTimerElapsedSimple);
        case Cue::NavStep: return single(tables::kNavStep);
    }
    return {};
}

// Trim feedback: pitch rises 20 Hz per step around 2000 Hz, clamped to 1500..=2500 Hz.
constexpr Note trim_note(TrimStep step) {
    const int32_t raw = 2000 + static_cast<int32_t>(step.value) * 20;
    const auto clamped = static_cast<uint16_t>(raw < 1500 ? 1500 : (raw > 2500 ? 2500 : raw));
    return Note{FreqHz{clamped}, DurationMs{25}, PauseMs{0}};
}

// Shared with the vario logic so both modules agree on time and tone types.
using vario::Millis;
using vario::Tone;

// PWM period in 1 MHz timer ticks, with ARR = period - 1 and 50% duty.
struct PwmTiming {
    uint16_t arr = 0;
    uint16_t ccr = 0;
    constexpr bool operator==(const PwmTiming&) const = default;
};

inline constexpr uint32_t kTimerTickHz = 1'000'000;

constexpr PwmTiming pwm_timing(FreqHz freq) {
    constexpr uint32_t kMaxPeriod = 65535;
    constexpr uint32_t kMinPeriod = 10;
    const uint32_t period = freq.value == 0 ? kMaxPeriod : std::clamp(kTimerTickHz / freq.value, kMinPeriod, kMaxPeriod);
    return PwmTiming{static_cast<uint16_t>(period - 1), static_cast<uint16_t>(period / 2)};
}

// Foreground note/pattern player plus the background vario stream. The output frequency is
// a pure function of the state, so the hardware layer only needs to apply it.
class Player {
public:
    constexpr void set_enabled(bool enabled) {
        enabled_ = enabled;
        if (!enabled_) {
            stop();
        }
    }

    constexpr bool enabled() const { return enabled_; }

    constexpr void play(Pattern pattern) {
        if (!enabled_ || pattern.notes.empty()) {
            return;
        }
        notes_ = pattern.notes;
        use_trim_ = false;
        start(pattern.repeats);
    }

    constexpr void play_cue(Cue cue, ToneStyle style) { play(pattern_for(cue, style)); }

    constexpr void play_trim(TrimStep step) {
        if (!enabled_) {
            return;
        }
        trim_ = trim_note(step);
        notes_ = {};
        use_trim_ = true;
        start(Repeats{0});
    }

    constexpr void stop() {
        playing_ = false;
        in_pause_ = false;
        remaining_ = ElapsedMs{};
        vario_ = vario::kSilent;
        vario_pause_ = false;
        vario_remaining_ = ElapsedMs{};
    }

    // Background vario tone. Changing the parameters restarts the pulse, as in the Rust code.
    constexpr void set_vario(Tone tone) {
        if (tone.is_silent()) {
            vario_ = vario::kSilent;
            vario_pause_ = false;
            vario_remaining_ = ElapsedMs{};
            return;
        }
        if (!(tone == vario_)) {
            vario_ = tone;
            vario_pause_ = false;
            vario_remaining_ = ElapsedMs{tone.pulse_ms};
        }
    }

    constexpr void tick(Millis dt) {
        uint32_t left = dt.value;
        while (playing_ && left >= remaining_.value) {
            left -= remaining_.value;
            if (in_pause_) {
                in_pause_ = false;
                advance_note();
            } else {
                end_tone();
            }
        }
        if (playing_) {
            remaining_.value -= left;
            return;
        }
        tick_vario(left);
    }

    constexpr FreqHz output() const {
        if (!enabled_) {
            return FreqHz{};
        }
        if (playing_) {
            if (in_pause_) {
                return FreqHz{};
            }
            return audible(note_at(index_).freq);
        }
        if (vario_.is_silent() || vario_pause_) {
            return FreqHz{};
        }
        return audible(FreqHz{vario_.freq_hz});
    }

    constexpr bool playing() const { return playing_; }

    constexpr bool is_vario_active() const { return !vario_.is_silent(); }

private:
    static constexpr FreqHz audible(FreqHz f) { return f.value >= kMinAudibleHz ? f : FreqHz{}; }

    constexpr void start(Repeats repeats) {
        index_ = 0;
        repeats_left_ = repeats.value;
        in_pause_ = false;
        remaining_ = ElapsedMs{note_at(0).duration.value};
        playing_ = true;
    }

    constexpr std::size_t note_count() const { return use_trim_ ? 1 : notes_.size(); }

    constexpr const Note& note_at(std::size_t i) const { return use_trim_ ? trim_ : notes_[i]; }

    constexpr void end_tone() {
        const bool more = index_ + 1 < note_count() || repeats_left_ > 0;
        if (!more) {
            playing_ = false;
            return;
        }
        const PauseMs pause = note_at(index_).pause;
        // A zero pause advances at once. The Rust player would otherwise stall on it.
        if (pause.value > 0) {
            in_pause_ = true;
            remaining_ = ElapsedMs{pause.value};
        } else {
            advance_note();
        }
    }

    constexpr void advance_note() {
        if (index_ + 1 < note_count()) {
            ++index_;
        } else {
            --repeats_left_;
            index_ = 0;
        }
        remaining_ = ElapsedMs{note_at(index_).duration.value};
    }

    constexpr void tick_vario(uint32_t left) {
        if (vario_.is_silent()) {
            return;
        }
        uint32_t rem = vario_remaining_.value;
        while (left >= rem) {
            left -= rem;
            if (!vario_pause_) {
                if (vario_.pause_ms > 0) {
                    vario_pause_ = true;
                    rem = vario_.pause_ms;
                } else {
                    // Continuous drone: the pulse length is only a cadence, the tone stays on.
                    rem = std::max<uint32_t>(vario_.pulse_ms, 100);
                }
            } else {
                vario_pause_ = false;
                rem = vario_.pulse_ms;
            }
        }
        vario_remaining_ = ElapsedMs{rem - left};
    }

    bool enabled_ = true;
    std::span<const Note> notes_{};
    Note trim_{};
    bool use_trim_ = false;
    std::size_t index_ = 0;
    uint8_t repeats_left_ = 0;
    bool playing_ = false;
    bool in_pause_ = false;
    ElapsedMs remaining_{};
    Tone vario_ = vario::kSilent;
    bool vario_pause_ = false;
    ElapsedMs vario_remaining_{};
};

// Battery alarm: while low, fire every 8 s; the timer rests at 7 s so the first warning comes after about 1 s.
inline constexpr uint32_t kBatteryRearmMs = 7000;
inline constexpr uint32_t kBatteryPeriodMs = 8000;

constexpr bool battery_low(BatteryMv mv, WarnDeci warn) {
    return mv.value < static_cast<uint16_t>(warn.value * 100);
}

struct BatteryWarnTimer {
    ElapsedMs elapsed{};

    constexpr bool step(bool low, Millis dt) {
        if (!low) {
            elapsed = ElapsedMs{kBatteryRearmMs};
            return false;
        }
        if (elapsed.value >= kBatteryPeriodMs) {
            elapsed = ElapsedMs{};
            return true;
        }
        elapsed.value += dt.value;
        return false;
    }
};

enum class RssiAlarm : uint8_t { None, Low, Critical };

constexpr RssiAlarm rssi_alarm(Rssi rssi, bool telemetry_connected) {
    if (!telemetry_connected || rssi.value >= 40) {
        return RssiAlarm::None;
    }
    return rssi.value < 20 ? RssiAlarm::Critical : RssiAlarm::Low;
}

struct RssiWarnTimer {
    ElapsedMs elapsed{};

    constexpr bool step(RssiAlarm level, Millis dt) {
        if (level == RssiAlarm::None) {
            elapsed = ElapsedMs{};
            return false;
        }
        const uint32_t period = level == RssiAlarm::Critical ? 3000 : 6000;
        if (elapsed.value >= period) {
            elapsed = ElapsedMs{};
            return true;
        }
        elapsed.value += dt.value;
        return false;
    }
};

// Inactivity beep: after 10 minutes without user input, beep every 30 s.
inline constexpr uint32_t kInactivityLimitMs = 600'000;
inline constexpr uint32_t kInactivityRepeatMs = 30'000;

struct InactivityTimer {
    ElapsedMs idle{};
    ElapsedMs since_beep{};

    constexpr bool step(bool user_active, Millis dt) {
        if (user_active) {
            idle = ElapsedMs{};
            since_beep = ElapsedMs{};
            return false;
        }
        const uint32_t sum = idle.value + dt.value;
        idle.value = sum < idle.value ? UINT32_MAX : sum;
        if (idle.value < kInactivityLimitMs) {
            return false;
        }
        if (since_beep.value >= kInactivityRepeatMs) {
            since_beep = ElapsedMs{};
            return true;
        }
        since_beep.value += dt.value;
        return false;
    }
};

// Preflight warning cadence: every 800 ms while the preflight check is active.
inline constexpr uint32_t kPreflightPeriodMs = 800;

struct PreflightTimer {
    Millis last{};

    constexpr bool step(Millis now) {
        if (now.value - last.value < kPreflightPeriodMs) {
            return false;
        }
        last = now;
        return true;
    }
};

struct ArmEdge {
    SwitchIndex arm_switch{};
    bool is_armed = false;
    bool was_armed = false;
    bool menu_active = false;
};

// The arm chime fires on a real arm-state change, and not while a menu is open.
constexpr Cue arm_cue(ArmEdge edge) {
    const bool armable = edge.arm_switch.value >= 1 && edge.arm_switch.value <= 22;
    if (!armable || edge.menu_active || edge.is_armed == edge.was_armed) {
        return Cue::None;
    }
    return edge.is_armed ? Cue::Armed : Cue::Disarmed;
}

}  // namespace ferrox::logic::buzzer
