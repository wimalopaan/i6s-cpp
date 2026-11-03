#pragma once

// Host tests for wm::logic::touch_nav (Rust `hw::touch::nav` and `hw::touch::ft6236` unit tests).
// Included from test_logic.cpp after `check(bool, const char*)` is defined.

#include <optional>

#include "wm/logic/input.hpp"
#include "wm/logic/touch_nav.hpp"

inline void touch_nav_tests() {
    namespace tn = wm::logic::touch_nav;
    using tn::Gesture;
    using tn::SwipeTracker;
    using tn::TouchEventKind;
    using tn::TouchPoint;
    using tn::TouchSample;

    const auto contact = [](const uint8_t x, const uint8_t y, const uint16_t raw_x, const uint16_t raw_y,
                            const TouchEventKind ev) {
        return TouchSample{.gesture = Gesture::None,
                           .point = TouchPoint{.x = x, .y = y, .raw_x = raw_x, .raw_y = raw_y, .event = ev}};
    };

    // ft6236: test_parse_gestures
    {
        tn::Packet buf{};
        buf[1] = 0x10;
        check(tn::parse_packet(buf).gesture == Gesture::SwipeUp, "ft6236 parse: 0x10 swipe up");
        buf[1] = 0x14;
        check(tn::parse_packet(buf).gesture == Gesture::SwipeRight, "ft6236 parse: 0x14 swipe right");
        buf[1] = 0x18;
        check(tn::parse_packet(buf).gesture == Gesture::SwipeDown, "ft6236 parse: 0x18 swipe down");
        buf[1] = 0x1C;
        check(tn::parse_packet(buf).gesture == Gesture::SwipeLeft, "ft6236 parse: 0x1C swipe left");
        buf[1] = 0x00;
        check(tn::parse_packet(buf).gesture == Gesture::None, "ft6236 parse: 0x00 no gesture");
        check(!tn::parse_packet(buf).point.has_value(), "ft6236 parse: no touches -> no point");
    }

    // ft6236: test_parse_coordinates_and_axis_swap
    {
        tn::Packet buf{};
        buf[2] = 1;
        buf[3] = 0x00;
        buf[4] = 40;
        buf[5] = 0x00;
        buf[6] = 100;
        const auto pt = tn::parse_packet(buf).point;
        check(pt.has_value(), "ft6236 parse: point present");
        if (pt) {
            check(pt->raw_x == 40 && pt->raw_y == 100, "ft6236 parse: raw coordinates");
            check(pt->y == 20, "ft6236 parse: lcd y = raw_x >> 1");
            check(pt->x == 128 - 50, "ft6236 parse: lcd x = 128 - (raw_y >> 1)");
            check(pt->event == TouchEventKind::PressDown, "ft6236 parse: event 0 = press down");
        }
    }

    // ft6236: test_coordinate_clamping
    {
        tn::Packet buf{};
        buf[2] = 1;
        buf[3] = 0x8F;
        buf[4] = 0xFF;
        buf[5] = 0x0F;
        buf[6] = 0xFF;
        const auto pt = tn::parse_packet(buf).point;
        check(pt.has_value(), "ft6236 clamp: point present");
        if (pt) {
            check(pt->y == 63, "ft6236 clamp: y clamped to 63");
            check(pt->x == 0, "ft6236 clamp: x saturates to 0");
            check(pt->event == TouchEventKind::Contact, "ft6236 clamp: event 2 = contact");
        }
    }
    static_assert(tn::parse_packet({0, 0, 1, 0x40, 0, 0, 0}).point->event == TouchEventKind::LiftUp);
    static_assert(tn::parse_packet({0, 0, 1, 0xC0, 0, 0, 0}).point->event == TouchEventKind::NoEvent);

    // nav: test_gesture_navigation
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys(TouchSample{.gesture = Gesture::SwipeUp}, false, tracker, 0) == tn::kKeyNavUp,
              "touch nav: swipe up -> Up");
        check(tn::touch_to_nav_keys(TouchSample{.gesture = Gesture::SwipeDown}, false, tracker, 0) ==
                  tn::kKeyNavDown,
              "touch nav: swipe down -> Down");
        check(tn::touch_to_nav_keys(TouchSample{.gesture = Gesture::SwipeRight}, false, tracker, 0) ==
                  tn::kKeyNavOk,
              "touch nav: swipe right -> OK");
        check(tn::touch_to_nav_keys(TouchSample{.gesture = Gesture::SwipeLeft}, false, tracker, 0) ==
                  tn::kKeyNavCancel,
              "touch nav: swipe left -> Cancel");
    }

    // nav: test_menu_tap_zones
    {
        check((tn::evaluate_tap_zones(64, 10, true) & tn::kKeyNavUp) != 0, "touch nav: menu header tap -> Up");
        check(tn::evaluate_tap_zones(80, 55, true) == tn::kKeyNavCancel, "touch nav: menu footer right -> Cancel");
        check(tn::evaluate_tap_zones(40, 55, true) == tn::kKeyNavOk, "touch nav: menu footer left -> OK");
    }

    // nav: test_menu_tap_zones_contact_and_liftup
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys_with_tracker(contact(80, 55, 110, 96, TouchEventKind::PressDown), true, tracker,
                                                 0) == 0,
              "touch nav: menu press down emits nothing");
        check(tracker.active, "touch nav: press down activates tracker");
        check(tn::touch_to_nav_keys_with_tracker(contact(80, 55, 110, 96, TouchEventKind::LiftUp), true, tracker,
                                                 0) == tn::kKeyNavCancel,
              "touch nav: menu lift up on footer right -> Cancel");
        check(!tracker.active, "touch nav: lift up clears tracker");
        SwipeTracker fresh{};
        check(tn::touch_to_nav_keys_with_tracker(contact(110, 30, 60, 36, TouchEventKind::NoEvent), true, fresh, 0) ==
                  0,
              "touch nav: NoEvent emits nothing");
    }

    // nav: test_software_swipe_drag
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 35, 70, 136, TouchEventKind::PressDown), true, tracker,
                                                 0) == 0,
              "touch nav: drag press emits nothing");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 55, 110, 136, TouchEventKind::Contact), true, tracker,
                                                 0) == tn::kKeyNavDown,
              "touch nav: 20 px drag down -> Down");
        check(tracker.swiped, "touch nav: drag marks swiped");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 55, 110, 136, TouchEventKind::LiftUp), true, tracker,
                                                 0) == 0,
              "touch nav: release after swipe is not a tap");
    }

    // nav: test_virtual_trims_on_dashboard
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys(contact(5, 15, 30, 246, TouchEventKind::PressDown), false, tracker, 0) ==
                  tn::kKeyTrimThrU,
              "touch nav: left border tap -> throttle trim up");
        check(tn::touch_to_nav_keys(contact(120, 45, 90, 16, TouchEventKind::PressDown), false, tracker, 0) ==
                  tn::kKeyTrimPitchD,
              "touch nav: right border tap -> pitch trim down");
    }

    // nav: test_modifier_trims
    {
        wm::logic::input::Gimbals sticks{.rh = 0, .rv = 0, .lv = -1000, .lh = 0};
        auto r = tn::process_modifier_trims(false, false, sticks);
        check(r.keys == 0 && !r.suppress_left && !r.suppress_right, "modifier trims: no button -> nothing");

        r = tn::process_modifier_trims(true, false, sticks);
        check(r.keys == 0 && !r.suppress_left, "modifier trims: idle throttle without throttle trim -> nothing");

        sticks.lh = -500;
        r = tn::process_modifier_trims(true, false, sticks);
        check(r.keys == tn::kKeyTrimYawL && r.suppress_left, "modifier trims: left + lh left -> yaw L");

        sticks.lh = 0;
        sticks.rh = 600;
        r = tn::process_modifier_trims(false, true, sticks);
        check(r.keys == tn::kKeyTrimRollR && r.suppress_right, "modifier trims: right + rh right -> roll R");
    }

    // nav: test_menu_touch_scrolling_and_instant_reversal
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 30, 60, 120, TouchEventKind::PressDown), true, tracker,
                                                 0) == 0,
              "touch scroll: press emits nothing");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 42, 84, 120, TouchEventKind::Contact), true, tracker,
                                                 0) == tn::kKeyNavDown,
              "touch scroll: 12 px down -> Down");
        check(tracker.scroll_dir == 1, "touch scroll: direction down");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 46, 92, 120, TouchEventKind::Contact), true, tracker,
                                                 0) == 0,
              "touch scroll: 4 px further emits nothing");
        check(tracker.max_y == 46, "touch scroll: peak tracked");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 42, 84, 120, TouchEventKind::Contact), true, tracker,
                                                 0) == 0,
              "touch scroll: reversal emits nothing yet");
        check(tracker.anchor_y == 46, "touch scroll: reversal re-anchors at peak");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 34, 68, 120, TouchEventKind::Contact), true, tracker,
                                                 0) == tn::kKeyNavUp,
              "touch scroll: 12 px up from peak -> Up immediately");
        check(tracker.scroll_dir == -1, "touch scroll: direction up");
    }

    // nav: test_dashboard_center_hold_and_swipe
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys_with_tracker(contact(64, 30, 60, 128, TouchEventKind::Contact), false, tracker,
                                                 0) == tn::kKeyMenuOpen,
              "touch dashboard: center hold -> menu open");
        check(tn::touch_to_nav_keys_with_tracker(contact(66, 32, 64, 124, TouchEventKind::Contact), false, tracker,
                                                 0) == tn::kKeyMenuOpen,
              "touch dashboard: tremor keeps menu open");
        check(tn::touch_to_nav_keys_with_tracker(contact(45, 30, 60, 166, TouchEventKind::Contact), false, tracker,
                                                 0) == tn::kKeyNavDown,
              "touch dashboard: swipe left -> next page");
        check(tracker.swiped, "touch dashboard: swipe marks swiped");
    }

    // nav: test_menu_touch_stationary_hold_and_liftup_stability
    {
        SwipeTracker tracker{};
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 30, 60, 120, TouchEventKind::PressDown), true, tracker,
                                                 0) == 0,
              "touch hold: press emits nothing");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 42, 84, 120, TouchEventKind::Contact), true, tracker,
                                                 0) == tn::kKeyNavDown,
              "touch hold: 12 px down -> Down");
        check(tracker.anchor_y == 42, "touch hold: anchor moved to scroll position");
        check(tn::touch_to_nav_keys_with_tracker(contact(60, 45, 90, 120, TouchEventKind::LiftUp), true, tracker,
                                                 0) == 0,
              "touch hold: lift with tremor emits nothing");
        check(!tracker.active, "touch hold: lift clears tracker");
    }

    // C++-only: explicit time and caller-owned state that replaced Rust globals.
    {
        SwipeTracker tracker{};
        const wm::logic::input::Gimbals sticks{};
        const tn::FrontButtons none{};
        const auto press = contact(5, 40, 0, 0, TouchEventKind::PressDown);
        check(tn::update_inputs(press, sticks, true, 0, 0, false, none, tracker, 1000) == 0,
              "touch update: menu press on body emits nothing");
        check(tracker.active && tracker.last_packet_ms == 1000, "touch update: packet time recorded");
        check(tn::update_inputs(std::nullopt, sticks, true, 0, 0, false, none, tracker, 1039) == 0 && tracker.active,
              "touch update: no release before 40 ms");
        check(tn::update_inputs(std::nullopt, sticks, true, 0, 0, false, none, tracker, 1040) == 0 && !tracker.active,
              "touch update: release after 40 ms (body tap emits nothing)");

        static_cast<void>(tn::update_inputs(contact(80, 55, 0, 0, TouchEventKind::PressDown), sticks, true, 0, 0,
                                            false, none, tracker, 2000));
        check(tn::update_inputs(std::nullopt, sticks, true, 0, 0, false, none, tracker, 2040) == tn::kKeyNavCancel,
              "touch update: timed-out release taps footer Cancel");

        check(tn::update_inputs(std::nullopt, sticks, true, 2, 2, true, {.left = true, .right = true}, tracker,
                                3000) == (tn::kKeyNavCancel | tn::kKeyNavOk),
              "touch update: menu front buttons navigate");
        const wm::logic::input::Gimbals deflected{.rh = -400, .rv = 400, .lv = 400, .lh = 400};
        check(tn::update_inputs(std::nullopt, deflected, false, 2, 2, true, {.left = true, .right = true}, tracker,
                                3000) == (tn::kKeyTrimRollL | tn::kKeyTrimPitchU | tn::kKeyTrimThrU | tn::kKeyTrimYawR),
              "touch update: modifier trims in flight");
        check(tn::update_inputs(std::nullopt, deflected, false, 2, 0, true, {.left = false, .right = true}, tracker,
                                3000) == 0,
              "touch update: right button without modifier func emits nothing");

        static_cast<void>(tn::touch_to_nav_keys(contact(40, 55, 0, 0, TouchEventKind::PressDown), true, tracker, 4000));
        check(tn::process_touch_release(true, tracker) == tn::kKeyNavOk && !tracker.active,
              "touch release: tap on footer left -> OK");
        check(tn::process_touch_release(true, tracker) == 0, "touch release: inactive tracker emits nothing");

        // Same-direction repeat needs 200 ms; settling after 100 ms stationary re-anchors.
        SwipeTracker t{};
        static_cast<void>(tn::touch_to_nav_keys(contact(60, 10, 0, 0, TouchEventKind::PressDown), true, t, 5000));
        check(tn::touch_to_nav_keys(contact(60, 22, 0, 0, TouchEventKind::Contact), true, t, 5010) == tn::kKeyNavDown,
              "touch pacing: first step fires");
        check(tn::touch_to_nav_keys(contact(60, 34, 0, 0, TouchEventKind::Contact), true, t, 5100) == 0,
              "touch pacing: repeat within 200 ms suppressed");
        check(tn::touch_to_nav_keys(contact(60, 36, 0, 0, TouchEventKind::Contact), true, t, 5210) == tn::kKeyNavDown,
              "touch pacing: repeat after 200 ms fires");
        check(tn::touch_to_nav_keys(contact(60, 45, 0, 0, TouchEventKind::Contact), true, t, 5220) == 0 &&
                  tn::touch_to_nav_keys(contact(60, 45, 0, 0, TouchEventKind::Contact), true, t, 5330) == 0 &&
                  t.anchor_y == 45,
              "touch pacing: stationary 100 ms settles anchor");
    }
}
