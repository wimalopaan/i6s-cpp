#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "ferrox/logic/input.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::auto_select {

// Normalized pot travel between two ticks that counts as a deliberate move (Rust `pot_change`).
inline constexpr int kPotMoveCounts = 300;

// Switch and pot readings of one main-loop tick.
struct Sample {
    Switches switches{};
    int16_t vr1{0};
    int16_t vr2{0};
};

// What moved since the previous tick. `sw_cond` is a condition code 1..26 (Rust `sw_change`),
// `dr_switch` is a dual-rate switch code 1..6 (Rust `dr_change`).
struct Changes {
    std::optional<uint8_t> sw_cond{};
    std::optional<uint8_t> dr_switch{};
    bool vr1{false};
    bool vr2{false};
};

inline constexpr std::size_t kTwoPosFieldCount = 12;  // SA..SWL in Rust detection order
inline constexpr std::size_t kDrFieldCount = 6;       // SA..SF
inline constexpr std::array<SwitchPos Switches::*, kTwoPosFieldCount> kFields{
    &Switches::sa,  &Switches::sb,  &Switches::sc,  &Switches::sd,  &Switches::se,  &Switches::sf,
    &Switches::sg,  &Switches::sh,  &Switches::swi, &Switches::swj, &Switches::swk, &Switches::swl};

// First condition code of each field. SB and SC are 3-pos (Up, Mid, Down); the others are 2-pos.
inline constexpr std::array<uint8_t, kTwoPosFieldCount> kCondBase{1, 3, 6, 9, 11, 13, 15, 17, 19, 21, 23, 25};

constexpr std::optional<uint8_t> condition_change(const Switches& now, const Switches& prev) {
    for (std::size_t i = 0; i < kFields.size(); ++i) {
        const auto field = kFields[i];
        if (now.*field == prev.*field) {
            continue;
        }
        const auto pos = static_cast<uint8_t>(now.*field);
        const bool three_pos = i == 1 || i == 2;
        return static_cast<uint8_t>(kCondBase[i] + (three_pos ? pos : (pos != 0 ? 1 : 0)));
    }
    return std::nullopt;
}

constexpr std::optional<uint8_t> dr_switch_change(const Switches& now, const Switches& prev) {
    for (std::size_t i = 0; i < kDrFieldCount; ++i) {
        if (now.*kFields[i] != prev.*kFields[i]) {
            return static_cast<uint8_t>(i + 1);
        }
    }
    return std::nullopt;
}

constexpr bool pot_moved(const int16_t now, const int16_t prev) {
    const int delta = int{now} - int{prev};
    return delta >= kPotMoveCounts || delta <= -kPotMoveCounts;
}

constexpr Changes detect(const Sample& now, const Sample& prev) {
    return Changes{
        .sw_cond = condition_change(now.switches, prev.switches),
        .dr_switch = dr_switch_change(now.switches, prev.switches),
        .vr1 = pot_moved(now.vr1, prev.vr1),
        .vr2 = pot_moved(now.vr2, prev.vr2),
    };
}

// Keeps the previous tick's readings. Must be stepped every main-loop tick, menu open or not.
struct Tracker {
    Sample prev{};

    constexpr Changes step(const Sample& now) {
        const Changes changes = detect(now, prev);
        prev = now;
        return changes;
    }
};

static_assert(condition_change(Switches{.sa = SwitchPos::Down}, Switches{}) == 2);
static_assert(condition_change(Switches{.sb = SwitchPos::Mid}, Switches{}) == 4);
static_assert(condition_change(Switches{.sb = SwitchPos::Down}, Switches{}) == 5);
static_assert(condition_change(Switches{.swl = SwitchPos::Down}, Switches{}) == 26);
static_assert(condition_change(Switches{}, Switches{}) == std::nullopt);
static_assert(dr_switch_change(Switches{.sf = SwitchPos::Down}, Switches{}) == 6);
static_assert(dr_switch_change(Switches{.swi = SwitchPos::Down}, Switches{}) == std::nullopt);
static_assert(pot_moved(300, 0) && pot_moved(-300, 0) && !pot_moved(299, 0));

}  // namespace ferrox::logic::auto_select
