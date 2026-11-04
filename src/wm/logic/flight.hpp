#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "wm/logic/channels.hpp"
#include "wm/logic/curve.hpp"
#include "wm/logic/input.hpp"
#include "wm/logic/mixer.hpp"
#include "wm/logic/safety.hpp"
#include "wm/logic/storage.hpp"
#include "wm/logic/trim.hpp"

namespace wm::logic::flight {

using Switches = wm::logic::Switches;

struct FlightControls {
    int16_t aileron = 0;
    int16_t elevator = 0;
    int16_t throttle = 0;
    int16_t rudder = 0;
};

/// Assigns the physical gimbals to roll, pitch, throttle and yaw for the stick mode.
constexpr FlightControls flight_controls(const input::Gimbals& g, const safety::StickMode m) {
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
constexpr bool is_armed(const uint8_t arm_switch, const Switches& sw) {
    return arm_switch == 0 || arm_switch > board::kMaxSwitchCond || mixer::eval_arm_switch(arm_switch, sw);
}

constexpr mixer::StickInputs stick_inputs(const input::InputState& in, const safety::StickMode sm,
                                          const storage::ModelConfig& model) {
    const FlightControls c = flight_controls(in.gimbals, sm);
    const auto thr_in = static_cast<uint16_t>(std::clamp<int32_t>((c.throttle + kMixerMax) / 2, 0, kMixerMax));
    const uint16_t thr = mixer::is_tank(model) ? thr_in :
        curve::evaluate(thr_in, model.thr_curve_pts, model.thr_curve_smooth != 0, model.thr_curve);
    return {c.aileron, c.elevator, thr, c.rudder};
}

constexpr safety::Preflight tank_preflight(const input::InputState& in, const safety::StickMode sm,
                                           const storage::ModelConfig& model) {
    const auto pots = std::span<const int16_t>(in.aux_pots).subspan<4, input::kAuxCount - 4>();
    const auto prepared = mixer::prepare_channels(stick_inputs(in, sm, model), pots, in.switches, model);
    return mixer::evaluate_tank_sources(prepared, pots, in.switches, model).centered()
        ? safety::Preflight::Ok : safety::Preflight::TankNotCentered;
}

/// Mixer output for one model: 18 channels in us, ready for the RF output.
constexpr std::array<uint16_t, kNumChannels> rf_channels(const input::InputState& in,
                                                         const safety::StickMode sm,
                                                         const storage::ModelConfig& model,
                                                         const trim::TrimValues& trims,
                                                         const uint8_t throttle_trim_mode,
                                                         mixer::TankState& tank_state) {
    return mixer::compute_channels(stick_inputs(in, sm, model),
                                   std::span<const int16_t>(in.aux_pots).subspan<4, input::kAuxCount - 4>(), in.switches, model,
                                   trims, throttle_trim_mode, tank_state);
}

}  // namespace wm::logic::flight
