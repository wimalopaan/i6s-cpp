#pragma once

#include <algorithm>
#include <cstdint>

#include "ferrox/logic/buzzer.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/vario.hpp"

namespace ferrox::logic::bind {

/// Holding BIND this long on the dashboard starts binding (Rust `main.rs` section 5).
inline constexpr uint32_t kHoldMs = 1000;

/// AFHDS 2A link state seen by the dashboard. `afhds` is false for CRSF models and when the radio is not ready.
struct Link {
    bool afhds = false;
    bool radio_binding = false;
    friend constexpr bool operator==(const Link&, const Link&) = default;
};

/// Binding is shown only on AFHDS 2A (Rust `is_binding = !is_crsf && rf::is_binding()`).
constexpr bool binding_shown(const Link link) { return link.afhds && link.radio_binding; }

struct Input {
    Link link{};
    bool menu_active = false;
    uint16_t keys = 0;     // held keys
    uint16_t pressed = 0;  // newly pressed keys
    vario::Millis dt{};
};

enum class Action : uint8_t { None, Start, Stop };

struct Event {
    Action action = Action::None;
    buzzer::Cue cue = buzzer::Cue::None;
    friend constexpr bool operator==(const Event&, const Event&) = default;
};

/// Hold timer for the BIND key. `was_held` stops a long hold from starting binding twice.
struct Controller {
    uint32_t hold_ms = 0;
    bool was_held = false;
    friend constexpr bool operator==(const Controller&, const Controller&) = default;
};

/// While binding, CANCEL or a new BIND press ends it (click). Otherwise a 1 s BIND hold starts it.
/// The hold is ignored and reset while a menu is open, as in Rust.
constexpr Event update(Controller& c, const Input& in) {
    if (binding_shown(in.link)) {
        if ((in.keys & safety::kKeyCancel) != 0 || (in.pressed & kKeyBind) != 0) {
            return {Action::Stop, buzzer::Cue::Click};
        }
        return {};
    }
    if (in.menu_active || !in.link.afhds || (in.keys & kKeyBind) == 0) {
        c = {};
        return {};
    }
    c.hold_ms = std::min(c.hold_ms + in.dt.value, kHoldMs);
    if (c.hold_ms >= kHoldMs && !c.was_held) {
        c.was_held = true;
        return {Action::Start, buzzer::Cue::BindHold};
    }
    return {};
}

}  // namespace ferrox::logic::bind
