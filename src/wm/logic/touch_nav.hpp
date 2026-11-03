#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>

#include "wm/logic/input.hpp"

// FS-i6S touch navigation and digital trims (Rust `hw::touch::nav` + FT6236 packet decoding).
//
// Navigation: swipe/tap up -> Up (bit 9), down -> Down (bit 8), right -> OK (bit 10),
// left -> Cancel (bit 11); right front button (PA10) -> OK, left front button (PA9) -> Cancel.
// Digital trims: front button held + stick deflection (modifier mode) or taps on the dashboard edges.
namespace wm::logic::touch_nav {

/// Stick deflection threshold required to engage modifier trim mode.
inline constexpr int16_t kTrimModifierStickThreshold = 350;

// Bitfield matching the board key scan.
inline constexpr uint16_t kKeyTrimRollR = 1u << 0;
inline constexpr uint16_t kKeyTrimRollL = 1u << 1;
inline constexpr uint16_t kKeyTrimPitchU = 1u << 2;
inline constexpr uint16_t kKeyTrimPitchD = 1u << 3;
inline constexpr uint16_t kKeyTrimThrU = 1u << 4;
inline constexpr uint16_t kKeyTrimThrD = 1u << 5;
inline constexpr uint16_t kKeyTrimYawR = 1u << 6;
inline constexpr uint16_t kKeyTrimYawL = 1u << 7;
inline constexpr uint16_t kKeyNavDown = 1u << 8;
inline constexpr uint16_t kKeyNavUp = 1u << 9;
inline constexpr uint16_t kKeyNavOk = 1u << 10;
inline constexpr uint16_t kKeyNavCancel = 1u << 11;
inline constexpr uint16_t kKeyBind = 1u << 12;
inline constexpr uint16_t kKeyMenuOpen = 1u << 13;

/// Release timeout: an active touch without packets for this long is treated as lifted.
inline constexpr uint32_t kReleaseTimeoutMs = 40;

/// FT6236 hardware gestures.
enum class Gesture : uint8_t { None, SwipeUp, SwipeRight, SwipeDown, SwipeLeft };

enum class TouchEventKind : uint8_t { PressDown, LiftUp, Contact, NoEvent };

/// Single touch point in 128x64 LCD space.
struct TouchPoint {
    uint8_t x = 0;  // 0..127
    uint8_t y = 0;  // 0..63
    uint16_t raw_x = 0;
    uint16_t raw_y = 0;
    TouchEventKind event = TouchEventKind::NoEvent;
    friend constexpr bool operator==(const TouchPoint&, const TouchPoint&) = default;
};

/// Touch sample decoded from a 7-byte FT6236 frame.
struct TouchSample {
    Gesture gesture = Gesture::None;
    std::optional<TouchPoint> point{};
    friend constexpr bool operator==(const TouchSample&, const TouchSample&) = default;
};

using Packet = std::array<uint8_t, 7>;

/// Decodes FT6236 registers 0x00..0x06. The sensor axes are swapped and scaled to the LCD:
/// LCD y = min(raw_x >> 1, 63), LCD x = min(128 - (raw_y >> 1), 127) (both truncated to 8 bits first).
[[nodiscard]] constexpr TouchSample parse_packet(const Packet& buf) {
    Gesture gesture = Gesture::None;
    switch (buf[1]) {
        case 0x10: gesture = Gesture::SwipeUp; break;
        case 0x14: gesture = Gesture::SwipeRight; break;
        case 0x18: gesture = Gesture::SwipeDown; break;
        case 0x1C: gesture = Gesture::SwipeLeft; break;
        default: break;
    }

    std::optional<TouchPoint> point{};
    if ((buf[2] & 0x0F) > 0) {
        const auto raw_x = static_cast<uint16_t>(((buf[3] & 0x0Fu) << 8) | buf[4]);
        const auto raw_y = static_cast<uint16_t>(((buf[5] & 0x0Fu) << 8) | buf[6]);

        TouchEventKind event = TouchEventKind::NoEvent;
        switch ((buf[3] >> 6) & 0x03) {
            case 0: event = TouchEventKind::PressDown; break;
            case 1: event = TouchEventKind::LiftUp; break;
            case 2: event = TouchEventKind::Contact; break;
            default: break;
        }

        const uint8_t lcd_y = std::min<uint8_t>(static_cast<uint8_t>(raw_x >> 1), 63);
        const auto y_scaled = static_cast<uint8_t>(raw_y >> 1);
        const uint8_t lcd_x = std::min<uint8_t>(y_scaled >= 128 ? 0 : static_cast<uint8_t>(128 - y_scaled), 127);

        point = TouchPoint{.x = lcd_x, .y = lcd_y, .raw_x = raw_x, .raw_y = raw_y, .event = event};
    }
    return TouchSample{.gesture = gesture, .point = point};
}

/// Software drag tracking across consecutive touch frames. Owned by the caller (Rust global
/// `GLOBAL_SWIPE_TRACKER`) and persisted between polls.
struct SwipeTracker {
    uint8_t start_x = 0;
    uint8_t start_y = 0;
    uint8_t last_x = 0;
    uint8_t last_y = 0;
    uint8_t anchor_y = 0;
    uint8_t max_y = 0;
    uint8_t min_y = 0;
    int8_t scroll_dir = 0;  // 0 = neutral, 1 = down, -1 = up
    bool active = false;
    bool swiped = false;
    uint32_t last_packet_ms = 0;
    uint32_t last_scroll_ms = 0;
    uint32_t last_move_ms = 0;

    constexpr void reset_at(const uint8_t x, const uint8_t y) {
        start_x = x;
        start_y = y;
        last_x = x;
        last_y = y;
        anchor_y = y;
        max_y = y;
        min_y = y;
        scroll_dir = 0;
        active = true;
        swiped = false;
        last_scroll_ms = 0;
        last_move_ms = 0;
    }

    constexpr void clear() {
        active = false;
        swiped = false;
        scroll_dir = 0;
        last_scroll_ms = 0;
        last_move_ms = 0;
    }
};

namespace detail {

constexpr bool in_range(const uint8_t v, const uint8_t lo, const uint8_t hi) {
    return v >= lo && v <= hi;
}

constexpr uint8_t sat_sub(const uint8_t a, const uint8_t b) {
    return a > b ? static_cast<uint8_t>(a - b) : 0;
}

constexpr uint8_t sat_add(const uint8_t a, const uint8_t b) {
    const unsigned sum = static_cast<unsigned>(a) + b;
    return sum > 0xFF ? 0xFF : static_cast<uint8_t>(sum);
}

}  // namespace detail

/// Maps a static tap position to navigation (menu) or trim / page actions (dashboard).
[[nodiscard]] constexpr uint16_t evaluate_tap_zones(const uint8_t x, const uint8_t y, const bool menu_active) {
    uint16_t keys = 0;
    if (menu_active) {
        // Footer: right half "[ESC] Back", left half "[OK] Select". Header -> Up.
        // The content body (18..=45) deliberately emits nothing to avoid accidental selection.
        if (y >= 46) {
            keys |= x >= 64 ? kKeyNavCancel : kKeyNavOk;
        } else if (y < 18) {
            keys |= kKeyNavUp;
        }
    } else {
        if (y < 14) {
            keys |= kKeyNavDown;  // status bar: next dashboard page
        } else if (x < 16) {
            keys |= y < 32 ? kKeyTrimThrU : kKeyTrimThrD;
        } else if (x > 112) {
            keys |= y < 32 ? kKeyTrimPitchU : kKeyTrimPitchD;
        } else if (y > 52) {
            if (x < 36) {
                keys |= kKeyTrimYawL;
            } else if (x < 64) {
                keys |= kKeyTrimYawR;
            } else if (x < 92) {
                keys |= kKeyTrimRollL;
            } else {
                keys |= kKeyTrimRollR;
            }
        } else if (detail::in_range(x, 32, 96) && detail::in_range(y, 20, 44)) {
            keys |= kKeyMenuOpen;  // accumulates towards the 1.2 s menu-open hold
        }
    }
    return keys;
}

/// Evaluates one touch sample with an explicit tracker. `now_ms` is the current system time
/// (Rust `time::millis()`); it only paces menu scrolling.
[[nodiscard]] constexpr uint16_t touch_to_nav_keys_with_tracker(const TouchSample& sample, const bool menu_active,
                                                               SwipeTracker& tracker, const uint32_t now_ms) {
    uint16_t keys = 0;

    // Hardware gestures are suppressed in menus (software drag paces scrolling there) and after a
    // software swipe, so the controller cannot add a spurious step on release.
    if (!menu_active && !tracker.swiped) {
        switch (sample.gesture) {
            case Gesture::SwipeUp: keys |= kKeyNavUp; break;
            case Gesture::SwipeDown: keys |= kKeyNavDown; break;
            case Gesture::SwipeRight: keys |= kKeyNavOk; break;
            case Gesture::SwipeLeft: keys |= kKeyNavCancel; break;
            case Gesture::None: break;
        }
    }

    if (!sample.point) {
        // No contact: finger lifted.
        if (tracker.active && !tracker.swiped) {
            keys |= evaluate_tap_zones(tracker.start_x, tracker.start_y, menu_active);
        }
        tracker.clear();
        return keys;
    }

    const TouchPoint pt = *sample.point;
    switch (pt.event) {
        case TouchEventKind::PressDown:
            tracker.reset_at(pt.x, pt.y);
            break;
        case TouchEventKind::Contact: {
            if (!tracker.active) {
                tracker.reset_at(pt.x, pt.y);
            }

            if (menu_active) {
                // Only vertical displacement scrolls; horizontal motion never selects or cancels.
                const uint32_t now = now_ms;
                const uint32_t now_nz = std::max<uint32_t>(now, 1);

                // Stationary for >= 100 ms (movement within +-1 px): settle the anchor so lifting
                // the finger cannot overshoot.
                const int dmove = static_cast<int>(pt.y) - static_cast<int>(tracker.last_y);
                if (dmove >= 2 || dmove <= -2) {
                    tracker.last_move_ms = now_nz;
                } else if (tracker.last_move_ms != 0 && now - tracker.last_move_ms >= 100) {
                    tracker.anchor_y = pt.y;
                    tracker.max_y = pt.y;
                    tracker.min_y = pt.y;
                }

                tracker.max_y = std::max(tracker.max_y, pt.y);
                tracker.min_y = std::min(tracker.min_y, pt.y);

                // Deliberate reversal (>= 4 px against the previous scroll direction) re-anchors at the turning point.
                if (tracker.scroll_dir > 0 && pt.y <= detail::sat_sub(tracker.max_y, 4)) {
                    tracker.anchor_y = tracker.max_y;
                    tracker.min_y = pt.y;
                    tracker.scroll_dir = 0;
                } else if (tracker.scroll_dir < 0 && pt.y >= detail::sat_add(tracker.min_y, 4)) {
                    tracker.anchor_y = tracker.min_y;
                    tracker.max_y = pt.y;
                    tracker.scroll_dir = 0;
                }

                const int dy = static_cast<int>(pt.y) - static_cast<int>(tracker.anchor_y);
                const uint32_t elapsed =
                    tracker.last_scroll_ms == 0 ? std::numeric_limits<uint32_t>::max() : now - tracker.last_scroll_ms;

                // 12 px threshold; repeats in the same direction need >= 200 ms, a fresh step or reversal fires at once.
                const auto scroll = [&](const uint16_t key, const int8_t dir) {
                    if (tracker.scroll_dir != dir || elapsed >= 200) {
                        keys |= key;
                        tracker.anchor_y = pt.y;
                        tracker.max_y = pt.y;
                        tracker.min_y = pt.y;
                        tracker.scroll_dir = dir;
                        tracker.last_scroll_ms = now_nz;
                        tracker.last_move_ms = now_nz;
                        tracker.swiped = true;
                    }
                };
                if (dy <= -12) {
                    scroll(kKeyNavUp, -1);
                } else if (dy >= 12) {
                    scroll(kKeyNavDown, 1);
                }
            } else {
                // Dashboard: center hold area (28..=100, 16..=48).
                const bool started_in_center =
                    detail::in_range(tracker.start_x, 28, 100) && detail::in_range(tracker.start_y, 16, 48);
                const int dx = static_cast<int>(pt.x) - static_cast<int>(tracker.start_x);
                const int dy = static_cast<int>(pt.y) - static_cast<int>(tracker.start_y);

                if (started_in_center) {
                    if (dx <= -16 || dy >= 16) {
                        if (!tracker.swiped) {
                            keys |= kKeyNavDown;
                            tracker.swiped = true;
                        }
                    } else if (dx >= 16 || dy <= -16) {
                        if (!tracker.swiped) {
                            keys |= kKeyNavUp;
                            tracker.swiped = true;
                        }
                    } else {
                        keys |= kKeyMenuOpen;
                    }
                } else if (!tracker.swiped) {
                    if (dx <= -14 || dy >= 14) {
                        keys |= kKeyNavDown;
                        tracker.swiped = true;
                    } else if (dx >= 14 || dy <= -14) {
                        keys |= kKeyNavUp;
                        tracker.swiped = true;
                    }
                }
            }

            tracker.last_x = pt.x;
            tracker.last_y = pt.y;
            break;
        }
        case TouchEventKind::LiftUp:
            // Lift without a swipe is a tap at the press position.
            if (tracker.active && !tracker.swiped) {
                keys |= evaluate_tap_zones(tracker.start_x, tracker.start_y, menu_active);
            }
            tracker.clear();
            break;
        case TouchEventKind::NoEvent:
            break;
    }

    // Dashboard only: virtual edge trims fire immediately during press/hold.
    if (!menu_active && !tracker.swiped && keys == 0) {
        if (pt.x < 16 || pt.x > 112 || pt.y > 52) {
            keys |= evaluate_tap_zones(pt.x, pt.y, false);
        } else if (detail::in_range(pt.x, 28, 100) && detail::in_range(pt.y, 16, 48)) {
            keys |= kKeyMenuOpen;
        }
    }

    return keys;
}

/// Evaluates a sample read after a touch interrupt; records `now_ms` as the last packet time.
[[nodiscard]] constexpr uint16_t touch_to_nav_keys(const TouchSample& sample, const bool menu_active,
                                                  SwipeTracker& tracker, const uint32_t now_ms) {
    tracker.last_packet_ms = now_ms;
    return touch_to_nav_keys_with_tracker(sample, menu_active, tracker, now_ms);
}

/// Touch release when no packet / interrupt is asserted. Emits a tap if the touch never swiped.
[[nodiscard]] constexpr uint16_t process_touch_release(const bool menu_active, SwipeTracker& tracker) {
    uint16_t keys = 0;
    if (tracker.active && !tracker.swiped) {
        keys |= evaluate_tap_zones(tracker.start_x, tracker.start_y, menu_active);
    }
    tracker.active = false;
    tracker.swiped = false;
    return keys;
}

/// Result of modifier trim evaluation (Rust tuple `(trim_keys, suppress_left, suppress_right)`).
struct ModifierTrims {
    uint16_t keys = 0;
    bool suppress_left = false;
    bool suppress_right = false;
    friend constexpr bool operator==(const ModifierTrims&, const ModifierTrims&) = default;
};

/// Front button held + stick deflection: left -> yaw (lh) and, if enabled, throttle (lv) trims;
/// right -> roll (rh) and pitch (rv) trims.
[[nodiscard]] constexpr ModifierTrims process_modifier_trims_cfg(const bool left_held, const bool right_held,
                                                                 const input::Gimbals& sticks,
                                                                 const bool throttle_trim_enabled) {
    constexpr int16_t t = kTrimModifierStickThreshold;
    ModifierTrims r{};

    if (left_held) {
        if (sticks.lh < -t) {
            r.keys |= kKeyTrimYawL;
            r.suppress_left = true;
        } else if (sticks.lh > t) {
            r.keys |= kKeyTrimYawR;
            r.suppress_left = true;
        }

        if (throttle_trim_enabled) {
            if (sticks.lv < -t) {
                r.keys |= kKeyTrimThrD;
                r.suppress_left = true;
            } else if (sticks.lv > t) {
                r.keys |= kKeyTrimThrU;
                r.suppress_left = true;
            }
        }
    }

    if (right_held) {
        if (sticks.rh < -t) {
            r.keys |= kKeyTrimRollL;
            r.suppress_right = true;
        } else if (sticks.rh > t) {
            r.keys |= kKeyTrimRollR;
            r.suppress_right = true;
        }

        if (sticks.rv < -t) {
            r.keys |= kKeyTrimPitchD;
            r.suppress_right = true;
        } else if (sticks.rv > t) {
            r.keys |= kKeyTrimPitchU;
            r.suppress_right = true;
        }
    }

    return r;
}

[[nodiscard]] constexpr ModifierTrims process_modifier_trims(const bool left_held, const bool right_held,
                                                             const input::Gimbals& sticks) {
    return process_modifier_trims_cfg(left_held, right_held, sticks, false);
}

/// Front tactile buttons (Rust `read_front_buttons()`): left = PA9, right = PA10, true = pressed.
struct FrontButtons {
    bool left = false;
    bool right = false;
};

/// Rear button function code for "trims modifier".
inline constexpr uint8_t kRearFuncTrimModifier = 2;

/// Combined touch + front-button tick (Rust `update_inputs`). The caller reads the buttons and must
/// publish the result as the touch key state (Rust `set_touch_keys`).
[[nodiscard]] constexpr uint16_t update_inputs(const std::optional<TouchSample>& touch_sample,
                                               const input::Gimbals& sticks, const bool menu_active,
                                               const uint8_t rear_left_func, const uint8_t rear_right_func,
                                               const bool throttle_trim_enabled, const FrontButtons buttons,
                                               SwipeTracker& tracker, const uint32_t now_ms) {
    uint16_t combined = 0;

    if (menu_active) {
        // In menus and wizards the front buttons always navigate.
        if (buttons.left) {
            combined |= kKeyNavCancel;
        }
        if (buttons.right) {
            combined |= kKeyNavOk;
        }
    } else {
        const bool left_mod = rear_left_func == kRearFuncTrimModifier;
        const bool right_mod = rear_right_func == kRearFuncTrimModifier;
        if ((left_mod && buttons.left) || (right_mod && buttons.right)) {
            combined |= process_modifier_trims_cfg(buttons.left && left_mod, buttons.right && right_mod, sticks,
                                                   throttle_trim_enabled)
                            .keys;
        }
    }

    uint16_t touch_keys = 0;
    if (touch_sample) {
        touch_keys = touch_to_nav_keys(*touch_sample, menu_active, tracker, now_ms);
    } else if (tracker.active && now_ms - tracker.last_packet_ms >= kReleaseTimeoutMs) {
        if (!tracker.swiped) {
            touch_keys |= evaluate_tap_zones(tracker.start_x, tracker.start_y, menu_active);
        }
        tracker.clear();
    }
    combined |= touch_keys;

    return combined;
}

}  // namespace wm::logic::touch_nav
