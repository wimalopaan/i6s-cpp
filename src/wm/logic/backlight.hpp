#pragma once

#include <cstdint>
#include <optional>

#include "wm/logic/storage.hpp"

namespace wm::logic::backlight {

// Radio Setup "Backlight" codes: 0 always on, 1/2/3 = 15/30/60 s.
constexpr uint32_t timeout_ms(const uint8_t code) {
    switch (code) {
        case 1: return 15'000;
        case 2: return 30'000;
        case 3: return 60'000;
        default: return 0;
    }
}

constexpr uint8_t on_level(const storage::RadioConfig& r) { return static_cast<uint8_t>(r.backlight_brightness * 10); }

// Auto-dim as in Rust main.rs: activity restores the brightness and restarts the timeout, expiry turns
// the light off. Returns the level to apply this tick, or nullopt to leave it unchanged.
struct Dimmer {
    uint32_t remaining_ms = 30'000;

    constexpr std::optional<uint8_t> step(const bool user_active, const storage::RadioConfig& r, const uint16_t dt_ms) {
        if (user_active) {
            remaining_ms = timeout_ms(r.backlight_timeout);
            return on_level(r);
        }
        if (r.backlight_timeout == 0 || dt_ms == 0) {
            return std::nullopt;
        }
        if (remaining_ms > dt_ms) {
            remaining_ms -= dt_ms;
            return std::nullopt;
        }
        remaining_ms = 0;
        return uint8_t{0};
    }
};

}  // namespace wm::logic::backlight
