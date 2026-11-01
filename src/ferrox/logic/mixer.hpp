#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/trim.hpp"

namespace ferrox::logic::mixer {

enum class ModelType : uint8_t { Airplane = 0, Glider = 1, Heli = 2, Quad = 3, General = 4 };

enum class WingTail : uint8_t { Normal = 0, Elevon = 1, VTail = 2, Flaperon = 3 };

constexpr WingTail wing_tail_from(const uint8_t val) {
    switch (val) {
        case 1: return WingTail::Elevon;
        case 2: return WingTail::VTail;
        case 3: return WingTail::Flaperon;
        default: return WingTail::Normal;
    }
}

using ChannelValues = std::array<int32_t, kNumChannels>;
using Sticks = std::array<int16_t, 4>;  // Roll, Pitch, Throttle, Yaw (conditioned)

struct StickInputs {
    int16_t roll = 0;
    int16_t pitch = 0;
    uint16_t throttle = 0;
    int16_t yaw = 0;
    friend constexpr bool operator==(const StickInputs&, const StickInputs&) = default;
};

struct TimerLatch {
    bool value = false;
    friend constexpr bool operator==(const TimerLatch&, const TimerLatch&) = default;
};

struct TimerArmed {
    bool value = false;
    friend constexpr bool operator==(const TimerArmed&, const TimerArmed&) = default;
};
using GVars = std::array<int16_t, 5>;

constexpr int32_t clamp_mix(const int32_t v) {
    return std::clamp<int32_t>(v, kMixerMin, kMixerMax);
}

/// Attenuates the down (negative) side by diff% when diff > 0, the up side when diff < 0.
constexpr int32_t apply_differential(const int32_t val, const int8_t diff) {
    if (diff == 0) {
        return val;
    }
    const int32_t d = std::clamp<int32_t>(diff, -100, 100);
    if (d > 0 && val < 0) {
        return (val * (100 - d)) / 100;
    }
    if (d < 0 && val > 0) {
        return (val * (100 + d)) / 100;
    }
    return val;
}

/// Dual rate and integer cubic expo on a normalized stick input.
constexpr int16_t apply_dr_expo(const int16_t input, const uint8_t rate, const int8_t expo) {
    const int32_t rate_clamped = std::clamp<int32_t>(rate, 30, 100);
    const int32_t x = (static_cast<int32_t>(input) * rate_clamped) / 100;

    if (expo == 0) {
        return static_cast<int16_t>(clamp_mix(x));
    }

    const int32_t expo_val = std::clamp<int32_t>(expo, -100, 100);
    // |x| <= 1000, so x^3 <= 1e9 fits into int32.
    const int32_t x_cubic = (x * x * x) / 1'000'000;

    int32_t result;
    if (expo_val > 0) {
        result = (x * (100 - expo_val) + x_cubic * expo_val) / 100;
    } else {
        const int32_t abs_expo = -expo_val;
        result = x + ((x - x_cubic) * abs_expo) / 100;
    }
    return static_cast<int16_t>(clamp_mix(result));
}

constexpr int32_t three_pos_value(const SwitchPos p) {
    switch (p) {
        case SwitchPos::Up: return kMixerMin;
        case SwitchPos::Mid: return kMixerCenter;
        case SwitchPos::Down: return kMixerMax;
    }
    return kMixerCenter;
}

/// Switch condition codes used by mix lines and the arm switch (0 = always active).
constexpr bool is_switch_active(const uint8_t condition, const Switches& s) {
    switch (condition) {
        case 0: return true;
        case 1: return s.sa == SwitchPos::Up;
        case 2: return s.sa == SwitchPos::Down;
        case 3: return s.sb == SwitchPos::Up;
        case 4: return s.sb == SwitchPos::Mid;
        case 5: return s.sb == SwitchPos::Down;
        case 6: return s.sc == SwitchPos::Up;
        case 7: return s.sc == SwitchPos::Mid;
        case 8: return s.sc == SwitchPos::Down;
        case 9: return s.sd == SwitchPos::Up;
        case 10: return s.sd == SwitchPos::Down;
        case 11: return s.se == SwitchPos::Up;
        case 12: return s.se == SwitchPos::Down;
        case 13: return s.sf == SwitchPos::Up;
        case 14: return s.sf == SwitchPos::Down;
        case 15: return s.sg == SwitchPos::Up;
        case 16: return s.sg == SwitchPos::Down;
        case 17: return s.sh == SwitchPos::Up;
        case 18: return s.sh == SwitchPos::Down;
        case 19: return s.swi == SwitchPos::Up;
        case 20: return s.swi == SwitchPos::Down;
        case 21: return s.swj == SwitchPos::Up;
        case 22: return s.swj == SwitchPos::Down;
        default: return true;
    }
}

/// Arm switch is valid only for condition codes 1..=22; 0 or out-of-range means not armed.
constexpr bool eval_arm_switch(const uint8_t arm_switch, const Switches& s) {
    return arm_switch > 0 && arm_switch <= 22 && is_switch_active(arm_switch, s);
}

/// True for high rates, false for low rates.
constexpr bool is_dr_high(const uint8_t dr_switch, const Switches& s) {
    switch (dr_switch) {
        case 1: return s.sa == SwitchPos::Up;
        case 2: return s.sb == SwitchPos::Up;
        case 3: return s.sc == SwitchPos::Up;
        case 4: return s.sd == SwitchPos::Up;
        case 5: return s.se == SwitchPos::Up;
        case 6: return s.sf == SwitchPos::Up;
        default: return true;
    }
}

/// Flight timer trigger. `latched` is updated for the THt mode.
constexpr bool is_timer_active(const uint8_t timer_source, const int16_t throttle, TimerLatch& latched,
                               const Switches& s, const TimerArmed is_armed) {
    if (!is_armed.value) {
        return false;
    }
    switch (timer_source) {
        case 0: return false;
        case 1: return throttle > -900;
        case 2:
            if (throttle > -900) {
                latched.value = true;
            }
            return latched.value;
        case 3: return true;
        default:
            if (timer_source >= 4 && timer_source <= 17) {
                return is_switch_active(static_cast<uint8_t>(timer_source - 3), s);
            }
            return false;
    }
}

constexpr int32_t pot_at(std::span<const int16_t> pots, const std::size_t i) {
    return i < pots.size() ? pots[i] : 0;
}

/// Maps a mixer source identifier to a normalized value (-1000..+1000).
constexpr int32_t evaluate_source(const uint8_t src, const Sticks& sticks, std::span<const int16_t> pots,
                                  const Switches& s, const ChannelValues& channels,
                                  const GVars& gvars) {
    switch (src) {
        case 0: return kMixerCenter;
        case 1: return sticks[0];
        case 2: return sticks[1];
        case 3: return sticks[2];
        case 4: return sticks[3];
        case 5: return pot_at(pots, 0);
        case 6: return pot_at(pots, 1);
        case 7: return three_pos_value(s.sa);
        case 8: return three_pos_value(s.sb);
        case 9: return three_pos_value(s.sc);
        case 10: return three_pos_value(s.sd);
        case 11: return kMixerMax;
        case 30: return std::clamp<int32_t>((sticks[2] + kMixerMax) / 2, 0, kMixerMax);
        case 31: return three_pos_value(s.se);
        case 32: return three_pos_value(s.sf);
        case 33: return three_pos_value(s.sg);
        case 34: return three_pos_value(s.sh);
        case 35: return three_pos_value(s.swi);
        case 36: return three_pos_value(s.swj);
        case 37: return pot_at(pots, 2);
        case 38: return pot_at(pots, 3);
        case 39: return pot_at(pots, 4);
        case 40: return pot_at(pots, 5);
        default:
            if (src >= 12 && src <= 29) {
                return channels[src - 12];
            }
            if (src >= 41 && src <= 45) {
                return std::clamp<int32_t>(gvars[src - 41], kMixerMin, kMixerMax);
            }
            return kMixerCenter;
    }
}

/// Full mixer pipeline: conditioning, wing/tail templates, aux mapping, matrix mixes,
/// output scaling, trims, safety interlocks and channel reversing.
constexpr std::array<uint16_t, kNumChannels> compute_channels(
    const StickInputs inputs, std::span<const int16_t> pots, const Switches& sw,
    const storage::ModelConfig& model, const trim::TrimValues& trims, const uint8_t throttle_trim_mode) {
    const auto [raw_roll, raw_pitch, curved_throttle, raw_yaw] = inputs;
    // Stage 1 and 2: dual rates and expo.
    const bool high_rate = is_dr_high(model.dr_switch, sw);
    const auto& rates = high_rate ? model.dr_high : model.dr_low;
    const auto& expos = high_rate ? model.expo_high : model.expo_low;

    const int16_t cond_roll = apply_dr_expo(raw_roll, rates[0], expos[0]);
    const int16_t cond_pitch = apply_dr_expo(raw_pitch, rates[1], expos[1]);
    const int16_t cond_yaw = apply_dr_expo(raw_yaw, rates[2], expos[2]);
    const int16_t cond_thr = static_cast<int16_t>(
        std::clamp<int32_t>(static_cast<int32_t>(curved_throttle) * 2 - kMixerMax, kMixerMin, kMixerMax));

    const Sticks sticks{cond_roll, cond_pitch, cond_thr, cond_yaw};
    ChannelValues ch{};

    // Stage 3: wing/tail templates on the primary channels.
    const WingTail tmpl = wing_tail_from(model.wing_tail_mix);
    const int32_t p = cond_pitch;
    const int32_t r = cond_roll;
    const int32_t y = cond_yaw;
    switch (tmpl) {
        case WingTail::Normal:
            ch[0] = r;
            ch[1] = p;
            ch[2] = cond_thr;
            ch[3] = y;
            break;
        case WingTail::Elevon:
            ch[0] = clamp_mix((p + apply_differential(-r, model.template_diff)) / 2);
            ch[1] = clamp_mix((p + apply_differential(r, model.template_diff)) / 2);
            ch[2] = cond_thr;
            ch[3] = y;
            break;
        case WingTail::VTail:
            ch[0] = r;
            ch[1] = clamp_mix((p + y) / 2);
            ch[2] = cond_thr;
            ch[3] = clamp_mix((p - y) / 2);
            break;
        case WingTail::Flaperon: {
            const int32_t left_r = apply_differential(r, model.template_diff);
            const int32_t right_r = apply_differential(-r, model.template_diff);
            const int32_t flap = evaluate_source(model.aux_channels[1], sticks, pots, sw, ch, model.gvars);
            ch[0] = clamp_mix(left_r + flap / 2);
            ch[1] = p;
            ch[2] = cond_thr;
            ch[3] = y;
            ch[5] = clamp_mix(right_r + flap / 2);
            break;
        }
    }

    // Auxiliary channels CH5..CH18. CH6 is owned by the template in Flaperon mode.
    for (std::size_t i = 0; i < model.aux_channels.size(); ++i) {
        const std::size_t ch_idx = 4 + i;
        if (ch_idx >= kNumChannels) {
            break;
        }
        if (tmpl == WingTail::Flaperon && ch_idx == 5) {
            continue;
        }
        ch[ch_idx] = evaluate_source(model.aux_channels[i], sticks, pots, sw, ch, model.gvars);
    }

    // Stage 3b: freeform matrix mixer lines, executed in order.
    for (const auto& mix : model.mixes) {
        if (mix.target_ch == 0 || mix.target_ch > kNumChannels) {
            continue;
        }
        if (!is_switch_active(mix.switch_sel, sw)) {
            continue;
        }

        const std::size_t target = mix.target_ch - 1;
        const int32_t src_val = evaluate_source(mix.source, sticks, pots, sw, ch, model.gvars);

        int32_t weight = mix.weight;
        if (mix.weight_src >= 1 && mix.weight_src <= 18) {
            weight = (static_cast<int32_t>(mix.weight) * ch[mix.weight_src - 1]) / kMixerMax;
        } else if (mix.weight_src >= 19 && mix.weight_src <= 36) {
            const int32_t unipolar = std::clamp<int32_t>((ch[mix.weight_src - 19] + kMixerMax) / 2, 0, kMixerMax);
            weight = (static_cast<int32_t>(mix.weight) * unipolar) / kMixerMax;
        } else if (mix.weight_src >= 37 && mix.weight_src <= 41) {
            const int32_t gv = std::clamp<int32_t>(model.gvars[mix.weight_src - 37], kMixerMin, kMixerMax);
            weight = (static_cast<int32_t>(mix.weight) * gv) / kMixerMax;
        }

        const int32_t term = (src_val * weight) / 100 + static_cast<int32_t>(mix.offset) * 10;

        switch (mix.mode) {
            case 0: ch[target] = clamp_mix(ch[target] + term); break;                // ADD
            case 1: ch[target] = clamp_mix((ch[target] * term) / kMixerMax); break;  // MULTIPLY
            case 2: ch[target] = clamp_mix(term); break;                             // REPLACE
            default: break;
        }
    }

    // Stage 4: scale to pulse widths.
    std::array<uint16_t, kNumChannels> rf{};
    for (std::size_t i = 0; i < kNumChannels; ++i) {
        const int32_t us = ((ch[i] * kChannelHalfSpanUs) / kMixerMax) + kChannelCenterUs;
        rf[i] = static_cast<uint16_t>(std::clamp<int32_t>(us, kChannelMinUs, kChannelMaxUs));
    }

    rf[0] = trim::apply(rf[0], trims.roll);
    rf[1] = trim::apply(rf[1], trims.pitch);
    rf[2] = trim::apply_throttle(rf[2], trims.throttle, throttle_trim_mode);
    rf[3] = trim::apply(rf[3], trims.yaw);

    // Safety: heli throttle hold (SA Down) and disarmed models pin the throttle to idle.
    const bool is_heli_hold = static_cast<ModelType>(model.model_type) == ModelType::Heli &&
                              sw.sa == SwitchPos::Down;
    const bool is_disarmed = model.arm_switch > 0 && model.arm_switch <= 22 &&
                             !eval_arm_switch(model.arm_switch, sw);
    if (is_heli_hold || is_disarmed) {
        rf[2] = kChannelMinUs;
    }

    for (std::size_t ch_idx = 0; ch_idx < kNumChannels; ++ch_idx) {
        if ((model.channel_reverse & (1u << ch_idx)) != 0) {
            rf[ch_idx] = static_cast<uint16_t>(kChannelReverseSum - rf[ch_idx]);
        }
    }

    return rf;
}

}  // namespace ferrox::logic::mixer
