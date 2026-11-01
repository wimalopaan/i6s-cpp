#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "ferrox/logic/power.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::input {

inline constexpr std::size_t kAuxCount = 10;
inline constexpr std::size_t kRawCount = 15;
inline constexpr uint16_t kDetentDeadband = 16;
inline constexpr uint16_t kHallDeadband = 3;
inline constexpr uint16_t kBatteryRawIndex = 10;  // PC0

using Raw = std::array<uint16_t, kRawCount>;

struct AdcReading {
    uint16_t value;
    friend constexpr bool operator==(const AdcReading&, const AdcReading&) = default;
};

struct Deadband {
    uint16_t value;
    friend constexpr bool operator==(const Deadband&, const Deadband&) = default;
};

/// Normalized physical gimbals (-1000..+1000): rh=PA0, rv=PA1, lv=PA2, lh=PA3.
struct Gimbals {
    int16_t rh = 0;
    int16_t rv = 0;
    int16_t lv = 0;
    int16_t lh = 0;
};

/// Normalized potentiometers VRA..VRF (-1000..+1000).
struct Pots {
    int16_t vr1 = 0;
    int16_t vr2 = 0;
    int16_t vr3 = 0;
    int16_t vr4 = 0;
    int16_t vr5 = 0;
    int16_t vr6 = 0;
};

struct InputState {
    Gimbals gimbals{};
    Pots pots{};
    Switches switches{};
    uint16_t battery_mv = 0;
    std::array<int16_t, kAuxCount> aux_pots{};
    Raw raw{};
};

enum class AdcMode : uint8_t { Default = 0, TwoPos, ThreePos, SixPos, Pot, PotDetent, InstantTrim };

constexpr AdcMode mode_from_u8(const uint8_t val) {
    switch (val) {
        case 1: return AdcMode::TwoPos;
        case 2: return AdcMode::ThreePos;
        case 3: return AdcMode::SixPos;
        case 4: return AdcMode::Pot;
        case 5: return AdcMode::PotDetent;
        case 6: return AdcMode::InstantTrim;
        default: return AdcMode::Default;
    }
}

/// Effective mode for aux index 0..9 (0..3: SA..SD, 4..5: VRA..VRB, 6..9: VRC..VRF).
constexpr AdcMode resolve_mode(const std::size_t idx, const uint8_t val) {
    const AdcMode mode = mode_from_u8(val);
    if (mode != AdcMode::Default) {
        return mode;
    }
    switch (idx) {
        case 0: return AdcMode::TwoPos;     // SA
        case 1: return AdcMode::TwoPos;     // SB (i6X default)
        case 2: return AdcMode::ThreePos;   // SC
        case 3: return AdcMode::TwoPos;     // SD
        default: return AdcMode::Pot;
    }
}

/// Raw ADC index for aux index 0..9.
constexpr std::size_t aux_index_to_raw(const std::size_t ch) {
    switch (ch) {
        case 0: return 4;   // SA: PA4
        case 1: return 5;   // SB: PA5
        case 2: return 8;   // SC: PB0
        case 3: return 9;   // SD: PB1
        case 4: return 6;   // VRA: PA6
        case 5: return 7;   // VRB: PA7
        case 6: return 11;  // VRC: ext ADC 1
        case 7: return 12;  // VRD: ext ADC 2
        case 8: return 13;  // VRE: ext ADC 3
        default: return 14; // VRF: ext ADC 4
    }
}

constexpr uint8_t decode_switch_6pos(const uint16_t raw) {
    if (raw < 682) return 1;
    if (raw < 1365) return 2;
    if (raw < 2048) return 3;
    if (raw < 2730) return 4;
    if (raw < 3413) return 5;
    return 6;
}

constexpr SwitchPos decode_switch_3pos(const uint16_t raw) {
    if (raw < 1365) return SwitchPos::Up;
    if (raw < 2730) return SwitchPos::Mid;
    return SwitchPos::Down;
}

constexpr SwitchPos decode_switch_2pos(const uint16_t raw) {
    return raw < 2048 ? SwitchPos::Up : SwitchPos::Down;
}

/// One calibrated analog axis. Holds the jitter filter state (no heap, no globals).
struct AxisCalib {
    uint16_t min = storage::kAdcMin;
    uint16_t center = storage::kAdcCenter;
    uint16_t max = storage::kAdcMax;
    uint32_t filtered_raw = 0;
    bool invert = false;
    uint16_t deadband = 0;

    /// Passes changes of 6 counts or more immediately; averages small jitter.
    constexpr uint16_t filter_raw(const uint16_t raw) {
        if (filtered_raw == 0) {
            filtered_raw = static_cast<uint32_t>(raw) * 4;
        }
        const auto previous = static_cast<uint16_t>(filtered_raw / 4);
        const int32_t diff = std::abs(static_cast<int32_t>(raw) - static_cast<int32_t>(previous));
        if (diff < 6) {
            filtered_raw = (filtered_raw - previous) + raw;
        } else {
            filtered_raw = static_cast<uint32_t>(raw) * 4;
        }
        return static_cast<uint16_t>(filtered_raw / 4);
    }

    constexpr int16_t finish(const int32_t val) const {
        return static_cast<int16_t>(invert ? -val : val);
    }

    /// Normalizes around the center with the configured deadband.
    constexpr int16_t normalize(const uint16_t raw) {
        const uint16_t s = filter_raw(raw);
        int32_t val = 0;
        if (deadband > 0 && static_cast<uint16_t>(std::abs(int32_t{s} - center)) <= deadband) {
            val = 0;
        } else if (s <= center) {
            const uint16_t active_center = sat_sub(center, deadband);
            const int32_t span = std::max<int32_t>(sat_sub(active_center, min), 100);
            const int32_t delta = int32_t{s} - active_center;
            val = std::clamp((delta * 1000) / span, -1000, 0);
        } else {
            const uint16_t active_center = sat_add(center, deadband);
            const int32_t span = std::max<int32_t>(sat_sub(max, active_center), 100);
            const int32_t delta = int32_t{s} - active_center;
            val = std::clamp((delta * 1000) / span, 0, 1000);
        }
        return finish(val);
    }

    /// Normalizes a center-detented pot: solid zero inside the detent window.
    constexpr int16_t normalize_detent(const AdcReading raw, const Deadband window) {
        const uint16_t s = filter_raw(raw.value);
        int32_t val = 0;
        if (static_cast<uint16_t>(std::abs(int32_t{s} - center)) <= window.value) {
            val = 0;
        } else if (s < center) {
            const uint16_t active_center = sat_sub(center, window.value);
            const int32_t span = std::max<int32_t>(sat_sub(active_center, min), 100);
            const int32_t delta = int32_t{s} - active_center;
            val = std::clamp((delta * 1000) / span, -1000, 0);
        } else {
            const uint16_t active_center = sat_add(center, window.value);
            const int32_t span = std::max<int32_t>(sat_sub(max, active_center), 100);
            const int32_t delta = int32_t{s} - active_center;
            val = std::clamp((delta * 1000) / span, 0, 1000);
        }
        return finish(val);
    }

private:
    static constexpr uint16_t sat_sub(const uint16_t a, const uint16_t b) {
        return a > b ? static_cast<uint16_t>(a - b) : uint16_t{0};
    }
    static constexpr uint16_t sat_add(const uint16_t a, const uint16_t b) {
        const uint32_t s = uint32_t{a} + b;
        return s > 0xFFFF ? uint16_t{0xFFFF} : static_cast<uint16_t>(s);
    }
};

/// Board switches SE..SWL. SE/SF are the PC12/PC15 pin levels (low = Down).
struct BoardSwitches {
    bool se_high = true;
    bool sf_high = true;
};

struct Calibration {
    AxisCalib rh{};
    AxisCalib rv{};
    AxisCalib lv{};
    AxisCalib lh{};
    std::array<AxisCalib, kAuxCount> aux{};
    std::array<uint8_t, kAuxCount> adc_modes{};
    uint32_t filtered_battery_mv = 0;
    bool ext_switches = false;
    bool ext_adc = false;
};

/// Applies stick and pot endpoints, inversion, deadbands and modes from the radio config.
constexpr void apply_calibration(const storage::RadioConfig& cfg, Calibration& c) {
    c.ext_switches = cfg.ext_switches != 0;
    c.ext_adc = cfg.ext_adc != 0;
    for (std::size_t i = 0; i < kAuxCount; ++i) {
        c.adc_modes[i] = cfg.adc_modes[i];
    }

    const uint16_t stick_deadband = cfg.gimbal_type == 1 ? kHallDeadband : 0;
    auto apply_stick = [&](AxisCalib& axis, const std::size_t idx) {
        axis.invert = ((cfg.stick_invert_mask >> (idx & 3)) & 1u) != 0;
        axis.deadband = stick_deadband;
        axis.min = cfg.sticks[idx].min;
        axis.center = cfg.sticks[idx].center;
        axis.max = cfg.sticks[idx].max;
    };
    apply_stick(c.rh, 0);
    apply_stick(c.rv, 1);
    apply_stick(c.lv, 2);
    apply_stick(c.lh, 3);

    for (std::size_t i = 0; i < kAuxCount; ++i) {
        const AdcMode mode = resolve_mode(i, c.adc_modes[i]);
        auto& a = c.aux[i];
        a.invert = false;
        a.min = cfg.aux_pots[i].min;
        a.max = cfg.aux_pots[i].max;
        a.center = mode == AdcMode::PotDetent
                       ? cfg.aux_pots[i].center
                       : static_cast<uint16_t>((uint32_t{cfg.aux_pots[i].min} + cfg.aux_pots[i].max) / 2);
    }
}

constexpr SwitchPos decode_switch_for(const AdcMode mode, const uint16_t raw) {
    return mode == AdcMode::ThreePos ? decode_switch_3pos(raw) : decode_switch_2pos(raw);
}

constexpr Switches decode_switches(const Raw& raw, const BoardSwitches& board, const Calibration& c) {
    Switches s{};
    const auto mode = [&](const std::size_t i) { return resolve_mode(i, c.adc_modes[i]); };

    s.sa = mode(0) == AdcMode::ThreePos ? decode_switch_3pos(raw[4]) : decode_switch_2pos(raw[4]);
    s.sb = mode(1) == AdcMode::TwoPos ? decode_switch_2pos(raw[5]) : decode_switch_3pos(raw[5]);
    s.sc = mode(2) == AdcMode::TwoPos ? decode_switch_2pos(raw[8]) : decode_switch_3pos(raw[8]);
    s.sd = mode(3) == AdcMode::ThreePos ? decode_switch_3pos(raw[9]) : decode_switch_2pos(raw[9]);

    if (c.ext_switches) {
        s.se = board.se_high ? SwitchPos::Up : SwitchPos::Down;
        s.sf = board.sf_high ? SwitchPos::Up : SwitchPos::Down;
    }

    if (c.ext_adc) {
        const auto ext = [&](const std::size_t aux_idx) {
            const AdcMode m = mode(aux_idx);
            return (m == AdcMode::ThreePos || m == AdcMode::TwoPos) ? decode_switch_for(m, raw[aux_idx + 5])
                                                                    : SwitchPos::Up;
        };
        s.swi = ext(6);
        s.swj = ext(7);
        s.swk = ext(8);
        s.swl = ext(9);
    }
    return s;
}

/// Full input snapshot from one raw ADC frame. Updates the filter state in `c`.
constexpr InputState process(const Raw& raw, const BoardSwitches& board, Calibration& c) {
    InputState st{};
    st.raw = raw;

    st.gimbals = Gimbals{
        .rh = c.rh.normalize(raw[0]),
        .rv = c.rv.normalize(raw[1]),
        .lv = c.lv.normalize(raw[2]),
        .lh = c.lh.normalize(raw[3]),
    };

    const std::size_t max_aux = c.ext_adc ? kAuxCount : 6;
    for (std::size_t i = 0; i < max_aux; ++i) {
        const uint16_t r = raw[aux_index_to_raw(i)];
        const AdcMode mode = resolve_mode(i, c.adc_modes[i]);
        int16_t v = 0;
        switch (mode) {
            case AdcMode::PotDetent: v = c.aux[i].normalize_detent({r}, {kDetentDeadband}); break;
            case AdcMode::TwoPos:
            case AdcMode::InstantTrim: v = r < 2048 ? -1000 : 1000; break;
            case AdcMode::ThreePos: {
                const SwitchPos p = decode_switch_3pos(r);
                v = p == SwitchPos::Up ? -1000 : (p == SwitchPos::Mid ? 0 : 1000);
                break;
            }
            case AdcMode::SixPos: v = static_cast<int16_t>(-1000 + (decode_switch_6pos(r) - 1) * 400); break;
            case AdcMode::Pot:
            case AdcMode::Default: v = c.aux[i].normalize(r); break;
        }
        st.aux_pots[i] = v;
    }

    st.pots = Pots{
        .vr1 = st.aux_pots[4],
        .vr2 = st.aux_pots[5],
        .vr3 = st.aux_pots[6],
        .vr4 = st.aux_pots[7],
        .vr5 = st.aux_pots[8],
        .vr6 = st.aux_pots[9],
    };

    st.switches = decode_switches(raw, board, c);

    const uint16_t instant_mv = battery_mv(raw[kBatteryRawIndex]);
    if (c.filtered_battery_mv == 0) {
        c.filtered_battery_mv = static_cast<uint32_t>(instant_mv) << 8;
        st.battery_mv = instant_mv;
    } else {
        c.filtered_battery_mv = c.filtered_battery_mv - (c.filtered_battery_mv >> 5) +
                                (static_cast<uint32_t>(instant_mv) << 3);
        st.battery_mv = static_cast<uint16_t>(c.filtered_battery_mv >> 8);
    }
    return st;
}

}  // namespace ferrox::logic::input
