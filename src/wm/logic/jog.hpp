#pragma once

#include <cstdint>

#include "wm/logic/auto_select.hpp"
#include "wm/logic/input.hpp"

// Menu navigation with the rear pots of the FS-i6S (flysky-i6s-rs `HAS_JOG_WHEELS`, ui/menu/mod.rs).
namespace wm::logic::jog {

inline constexpr int kDeflection = 300;

enum class Step : int8_t { None = 0, Up = -1, Down = 1 };

// Only spring wheels (Default) and pots step the menu; switch modes would hijack the list.
constexpr bool steps_menu(const input::AdcMode raw_mode) {
    return raw_mode == input::AdcMode::Default || raw_mode == input::AdcMode::Pot ||
           raw_mode == input::AdcMode::PotDetent;
}

constexpr Step deflection(const input::AdcMode raw_mode, const int16_t value) {
    if (!steps_menu(raw_mode)) {
        return Step::None;
    }
    if (value > kDeflection) {
        return Step::Down;
    }
    return value < -kDeflection ? Step::Up : Step::None;
}

// One step per deflection; the wheel must come back into [-300, 300] to re-arm. VRB counts only while VRA is
// centred. `raw_vra`/`raw_vrb` are the stored (unresolved) modes, as Rust `get_raw_adc_mode`.
struct Wheel {
    Step held{Step::None};

    constexpr Step update(const bool active, const input::AdcMode raw_vra, const int16_t vr1,
                          const input::AdcMode raw_vrb, const int16_t vr2) {
        // While inactive the held direction is kept (Rust skips the whole block), so a wheel still deflected
        // when navigation resumes does not step until it has been centred.
        if (!active) {
            return Step::None;
        }
        Step cur = deflection(raw_vra, vr1);
        if (cur == Step::None) {
            cur = deflection(raw_vrb, vr2);
        }
        if (cur == Step::None) {
            held = Step::None;
            return Step::None;
        }
        if (held != Step::None) {
            return Step::None;
        }
        held = cur;
        return cur;
    }
};

// Source picking: a rear pot in Pot mode held off centre selects VRA/VRB even without movement, ahead of the
// relative pot-change detection.
constexpr void prefer_deflected_pot(auto_select::Changes& c, const input::AdcMode raw_vra, const int16_t vr1,
                                    const input::AdcMode raw_vrb, const int16_t vr2) {
    const auto is_pot = [](const input::AdcMode m) {
        return m == input::AdcMode::Pot || m == input::AdcMode::PotDetent;
    };
    const auto deflected = [](const int16_t v) { return v >= kDeflection || v <= -kDeflection; };
    if (is_pot(raw_vra) && deflected(vr1)) {
        c.vr1 = true;
        c.vr2 = false;
    } else if (is_pot(raw_vrb) && deflected(vr2)) {
        c.vr1 = false;
        c.vr2 = true;
    }
}

}  // namespace wm::logic::jog
