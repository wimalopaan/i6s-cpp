#pragma once

#include <cstdint>
#include <string_view>

#include "wm/logic/format.hpp"
#include "wm/logic/framebuffer.hpp"
#include "wm/logic/mixer.hpp"
#include "wm/logic/safety.hpp"
#include "wm/logic/switches.hpp"

// Startup safety warning screen (Rust `run_preflight_check`).
namespace wm::logic::preflight {

using display::Framebuffer;

inline constexpr uint32_t kRenderIntervalMs = 33;

struct UnsafeSwitches {
    bool sa = false;
    bool sb = false;
    bool sc = false;
    bool sd = false;

    constexpr bool any() const { return sa || sb || sc || sd; }
};

/// All switches must be up, except SA which is Down (throttle hold) for heli.
constexpr UnsafeSwitches unsafe_switches(const Switches& sw, const mixer::ModelType mt) {
    const bool heli = mt == mixer::ModelType::Heli;
    return {heli ? sw.sa != SwitchPos::Down : sw.sa != SwitchPos::Up, sw.sb != SwitchPos::Up,
            sw.sc != SwitchPos::Up, sw.sd != SwitchPos::Up};
}

constexpr std::string_view throttle_message(const safety::Preflight p) {
    switch (p) {
        case safety::Preflight::ThrottleHigh: return "THROTTLE NOT AT IDLE!";
        case safety::Preflight::ThrottleNotCentered: return "CENTER THROTTLE STICK!";
        case safety::Preflight::ThrottleHoldDisengaged: return "THROTTLE HOLD OFF!";
        case safety::Preflight::Ok: break;
    }
    return {};
}

constexpr std::string_view footer(const safety::Preflight p) {
    switch (p) {
        case safety::Preflight::ThrottleNotCentered: return "Center Thr/Safe SW [ESC]Skip";
        case safety::Preflight::ThrottleHoldDisengaged: return "Hold Thr/Safe SW   [ESC]Skip";
        default: return "Lower Thr/Safe SW  [ESC]Skip";
    }
}

/// "[SA] [SC]" etc. Deliberate deviation: Rust cuts the list at 16 characters, which garbles the
/// fourth label ("[SA] [SB] [SC] ["); all four (19 characters) fit on the display.
constexpr format::Text<20> switch_list(const UnsafeSwitches u) {
    format::Text<20> t;
    const auto add = [&t](const bool flag, const std::string_view label) {
        if (!flag) {
            return;
        }
        if (!t.empty()) {
            t.push(' ');
        }
        for (const char c : label) {
            t.push(c);
        }
    };
    add(u.sa, "[SA]");
    add(u.sb, "[SB]");
    add(u.sc, "[SC]");
    add(u.sd, "[SD]");
    return t;
}

constexpr void render(Framebuffer& fb, const safety::Preflight thr, const UnsafeSwitches sw) {
    fb.clear();
    fb.draw_str_6x10({16, 2}, "SAFETY WARNING!", false);
    fb.draw_hline({0, 11}, 128, true);

    if (thr != safety::Preflight::Ok) {
        fb.draw_str_6x10({2, 16}, throttle_message(thr), false);
    }
    if (sw.any()) {
        fb.draw_str_6x10({2, 27}, "SWITCH WARNING:", false);
        const auto list = switch_list(sw);
        fb.draw_str_6x10({2, 37}, list.view(), false);
    }

    fb.draw_hline({0, 55}, 128, true);
    fb.draw_str_4x6({2, 57}, footer(thr), false);
}

}  // namespace wm::logic::preflight
