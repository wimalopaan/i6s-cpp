#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

#include "wm/logic/framebuffer.hpp"
#include "wm/logic/touch_nav.hpp"

// FS-i6S background tasks of the dashboard loop (flysky-i6s-rs hw::BoardIdleState and hw::power::PowerManager):
// power button shutdown, rear button clicks, the touch/right-button menu hold and the blinking button LEDs.
namespace wm::logic::board_idle {

inline constexpr uint16_t kShutdownHoldMs = 1500;
inline constexpr uint16_t kLedBlinkMs = 250;
inline constexpr uint32_t kCenterTouchHoldMs = 150;
inline constexpr uint8_t kRearFuncTimerReset = 1;
inline constexpr uint8_t kRearFuncInstantTrim = 1;

// Holding the power button (PB14) for 1.5 s shuts the radio down. The button that switched the radio on
// must be released first.
struct PowerManager {
    bool boot_release_detected{false};
    uint16_t hold_ms{0};
    bool shutting_down{false};

    // True exactly once, when the shutdown starts.
    constexpr bool update(const uint16_t dt_ms, const bool pressed) {
        if (shutting_down) {
            return false;
        }
        if (!boot_release_detected) {
            boot_release_detected = !pressed;
            return false;
        }
        if (!pressed) {
            hold_ms = 0;
            return false;
        }
        hold_ms = static_cast<uint16_t>(std::min<uint32_t>(uint32_t{hold_ms} + dt_ms, UINT16_MAX));
        if (hold_ms >= kShutdownHoldMs) {
            shutting_down = true;
            return true;
        }
        return false;
    }

    [[nodiscard]] constexpr uint8_t progress_pct() const {
        if (!boot_release_detected) {
            return 0;
        }
        return static_cast<uint8_t>(std::min<uint32_t>(uint32_t{hold_ms} * 100u / kShutdownHoldMs, 100u));
    }
};

struct Buttons {
    bool left{false};   // PA9
    bool right{false};  // PA10
};

struct Input {
    uint32_t now_ms{0};
    uint16_t dt_ms{0};
    uint16_t keys{0};  // touch/button key bits of this poll
    bool menu_active{false};
    Buttons buttons{};
    bool power_pressed{false};
    uint8_t rear_left_func{0};
    uint8_t rear_right_func{0};
};

struct Result {
    bool ok_held{false};               // centre long-tap or right button held on the dashboard
    bool timer_reset_clicked{false};   // left button, function TIMER RESET
    bool instant_trim_clicked{false};  // right button, function INSTANT TRIM
    bool shutdown{false};              // caller saves, beeps and cuts the power latch
    std::optional<bool> leds{};        // new level of the front button LEDs
};

struct State {
    PowerManager power{};
    bool prev_left{false};
    bool prev_right{false};
    uint32_t last_center_touch_ms{0};
    bool led_phase{true};
    uint16_t led_timer_ms{0};

    constexpr Result update(const Input& in) {
        Result r{};
        r.shutdown = power.update(in.dt_ms, in.power_pressed);

        if (in.menu_active) {
            led_timer_ms = static_cast<uint16_t>(std::min<uint32_t>(uint32_t{led_timer_ms} + in.dt_ms, UINT16_MAX));
            if (led_timer_ms >= kLedBlinkMs) {
                led_timer_ms = 0;
                led_phase = !led_phase;
                r.leds = led_phase;
            }
        } else if (!led_phase) {
            led_phase = true;
            led_timer_ms = 0;
            r.leds = true;
        }

        const bool left_clicked = in.buttons.left && !prev_left;
        const bool right_clicked = in.buttons.right && !prev_right;
        prev_left = in.buttons.left;
        prev_right = in.buttons.right;

        if (!in.menu_active) {
            if ((in.keys & touch_nav::kKeyMenuOpen) != 0) {
                last_center_touch_ms = in.now_ms;
            }
            const bool touch_active =
                last_center_touch_ms != 0 && in.now_ms - last_center_touch_ms < kCenterTouchHoldMs;
            r.ok_held = touch_active || in.buttons.right;
            r.timer_reset_clicked = left_clicked && in.rear_left_func == kRearFuncTimerReset;
            r.instant_trim_clicked = right_clicked && in.rear_right_func == kRearFuncInstantTrim;
        } else {
            last_center_touch_ms = 0;
        }
        return r;
    }

    constexpr void clear_menu_open_state() { last_center_touch_ms = 0; }
};

// In menus and the calibration wizard a swipe (Up/Down) must not also confirm (Rust take_frame_keys).
constexpr uint16_t menu_keys(const uint16_t keys, const bool menu_or_calib_active) {
    if (menu_or_calib_active && (keys & (touch_nav::kKeyNavUp | touch_nav::kKeyNavDown)) != 0) {
        return static_cast<uint16_t>(keys & ~touch_nav::kKeyNavOk);
    }
    return keys;
}

// "SHUTDOWN..." box with a progress bar while the power button is held.
constexpr void render_overlay(display::Framebuffer& fb, const uint8_t progress_pct) {
    if (progress_pct == 0) {
        return;
    }
    fb.fill_rect({{18, 19}, {92, 26}}, false);
    fb.draw_rect({{18, 19}, {92, 26}}, true);
    fb.draw_str_6x10({28, 21}, "SHUTDOWN...", false);
    fb.draw_rect({{27, 33}, {74, 6}}, true);
    const int fill_w = std::min(static_cast<int>(progress_pct) * 70 / 100, 70);
    if (fill_w > 0) {
        fb.fill_rect({{29, 35}, {fill_w, 2}}, true);
    }
}

}  // namespace wm::logic::board_idle
