#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>

#include "ferrox/logic/input.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::buzzer_inputs {

// Physical activity for the inactivity alarm: any key, a stick or pot deflection, or a switch change.
// Raw indices are the four sticks (0..3) and the two pots (6, 7), with the thresholds used by the Rust firmware.
struct ActivityTracker {
    std::array<uint16_t, 6> sticks{};
    Switches switches{};

    constexpr void prime(const input::Raw& raw, const Switches& sw) {
        sticks = {raw[0], raw[1], raw[2], raw[3], raw[6], raw[7]};
        switches = sw;
    }

    constexpr bool step(const input::Raw& raw, const Switches& sw, const uint16_t keys) {
        constexpr std::array<uint16_t, 6> kThreshold{30, 30, 30, 30, 40, 40};
        const std::array<uint16_t, 6> now{raw[0], raw[1], raw[2], raw[3], raw[6], raw[7]};
        bool moved = false;
        for (std::size_t i = 0; i < now.size(); ++i) {
            const int diff = int{now[i]} - int{sticks[i]};
            const int mag = diff < 0 ? -diff : diff;
            moved = moved || mag > kThreshold[i];
        }
        const bool sw_changed = !(sw == switches);
        sticks = now;
        switches = sw;
        return keys != 0 || moved || sw_changed;
    }
};

// Vario setting: 0 is off, 1 is always on, and 2..=23 select arm condition 1..=22 (as in Rust).
constexpr bool vario_switch_active(const uint8_t setting, const Switches& sw) {
    if (setting == 1) {
        return true;
    }
    if (setting >= 2 && setting <= 23) {
        return mixer::eval_arm_switch(static_cast<uint8_t>(setting - 1), sw);
    }
    return false;
}

// Pot centre click: fires when an aux pot leaves the ±25 deadband around centre (|prev| >= 25 to |curr| < 25), as in Rust.
template <std::size_t N>
struct PotCenterTracker {
    std::array<int16_t, N> prev{};

    constexpr bool step(std::span<const int16_t> pots) {
        bool crossed = false;
        for (std::size_t i = 0; i < pots.size() && i < N; ++i) {
            const int curr = pots[i];
            const int p = prev[i];
            crossed = crossed || (std::abs(p) >= 25 && std::abs(curr) < 25);
            prev[i] = static_cast<int16_t>(curr);
        }
        return crossed;
    }
};

}  // namespace ferrox::logic::buzzer_inputs
