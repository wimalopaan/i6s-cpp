#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/curve.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/trim.hpp"

namespace ferrox::logic::flight {

using Switches = ferrox::logic::Switches;

struct FlightControls {
    int16_t aileron = 0;
    int16_t elevator = 0;
    int16_t throttle = 0;
    int16_t rudder = 0;
};

/// Assigns the physical gimbals to roll, pitch, throttle and yaw for the stick mode.
constexpr FlightControls flight_controls(const input::Gimbals& g, safety::StickMode m) {
    using safety::StickMode;
    switch (m) {
        case StickMode::Mode1: return {g.rh, g.lv, g.rv, g.lh};
        case StickMode::Mode2: return {g.rh, g.rv, g.lv, g.lh};
        case StickMode::Mode3: return {g.lh, g.lv, g.rv, g.rh};
        case StickMode::Mode4: return {g.lh, g.rv, g.lv, g.rh};
    }
    return {};
}

/// Armed when no arm switch is configured or the configured switch is active (same rule as compute_channels).
constexpr bool is_armed(uint8_t arm_switch, const Switches& sw) {
    return arm_switch == 0 || arm_switch > 22 || mixer::eval_arm_switch(arm_switch, sw);
}

/// Mixer output for one model: 18 channels in µs, ready for the RF output.
/// Trims are passed in by the caller; the trim key state machine lives in logic/trim.hpp.
constexpr std::array<uint16_t, kNumChannels> rf_channels(const input::InputState& in,
                                                         safety::StickMode sm,
                                                         const storage::ModelConfig& model,
                                                         const trim::TrimValues& trims,
                                                         uint8_t throttle_trim_mode) {
    const FlightControls c = flight_controls(in.gimbals, sm);
    const auto thr_in = static_cast<uint16_t>(std::clamp<int32_t>((c.throttle + kMixerMax) / 2, 0, kMixerMax));
    const uint16_t thr = curve::evaluate(thr_in, model.thr_curve_pts, model.thr_curve_smooth != 0, model.thr_curve);
    return mixer::compute_channels({c.aileron, c.elevator, thr, c.rudder},
                                   std::span<const int16_t>(in.aux_pots).subspan<4, 6>(), in.switches, model,
                                   trims, throttle_trim_mode);
}

}  // namespace ferrox::logic::flight
