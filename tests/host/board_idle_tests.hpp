#pragma once

// FS-i6S board idle logic (flysky-i6s-rs hw::BoardIdleState / hw::power::PowerManager). Board independent,
// so it runs in both host builds. Included from test_logic.cpp after `check(bool, const char*)`.

#include <cstdint>

#include "wm/logic/board_idle.hpp"
#include "wm/logic/flight_timer.hpp"
#include "wm/logic/framebuffer.hpp"

inline void board_idle_tests() {
    namespace bi = wm::logic::board_idle;
    namespace tn = wm::logic::touch_nav;

    {
        bi::PowerManager pm{};
        check(!pm.update(1000, true) && !pm.update(1000, true) && pm.progress_pct() == 0,
              "power: button held since power-on is ignored until released");
        check(!pm.update(10, false) && pm.progress_pct() == 0, "power: boot release arms the button");
        check(!pm.update(750, true) && pm.progress_pct() == 50, "power: 750 ms hold shows 50 %");
        check(!pm.update(10, false) && pm.hold_ms == 0, "power: release resets the hold");
        check(!pm.update(1499, true), "power: 1499 ms is not enough");
        check(pm.update(1, true) && pm.shutting_down && pm.progress_pct() == 100, "power: 1500 ms shuts down");
        check(!pm.update(10, true), "power: shutdown fires once");
    }
    {
        bi::State s{};
        auto r = s.update({.now_ms = 10, .dt_ms = 1, .keys = tn::kKeyMenuOpen});
        check(r.ok_held, "board idle: centre tap counts as OK held");
        r = s.update({.now_ms = 159, .dt_ms = 1});
        check(r.ok_held, "board idle: centre tap stays held for 150 ms after the last report");
        r = s.update({.now_ms = 160, .dt_ms = 1});
        check(!r.ok_held, "board idle: centre tap released after 150 ms");
        r = s.update({.now_ms = 170, .dt_ms = 1, .buttons = {.right = true}});
        check(r.ok_held, "board idle: right button held counts as OK held");
        s.update({.now_ms = 180, .dt_ms = 1, .keys = tn::kKeyMenuOpen});
        s.clear_menu_open_state();
        check(!s.update({.now_ms = 181, .dt_ms = 1}).ok_held, "board idle: clear_menu_open_state drops the tap");
        s.update({.now_ms = 190, .dt_ms = 1, .keys = tn::kKeyMenuOpen});
        check(!s.update({.now_ms = 191, .dt_ms = 1, .keys = tn::kKeyMenuOpen, .menu_active = true}).ok_held,
              "board idle: no OK hold inside menus");
    }
    {
        bi::State s{};
        auto r = s.update({.now_ms = 1, .buttons = {.left = true}, .rear_left_func = 1});
        check(r.timer_reset_clicked && !r.instant_trim_clicked, "board idle: left click resets the timer");
        r = s.update({.now_ms = 2, .buttons = {.left = true}, .rear_left_func = 1});
        check(!r.timer_reset_clicked, "board idle: held left button clicks once");
        r = s.update({.now_ms = 3, .buttons = {.left = false, .right = true}, .rear_left_func = 1,
                      .rear_right_func = 1});
        check(r.instant_trim_clicked, "board idle: right click with INSTANT TRIM function");
        s.update({.now_ms = 4});
        r = s.update({.now_ms = 5, .buttons = {.left = true, .right = true}, .rear_left_func = 0,
                      .rear_right_func = 2});
        check(!r.timer_reset_clicked && !r.instant_trim_clicked, "board idle: other rear functions do not click");
        s.update({.now_ms = 6});
        r = s.update({.now_ms = 7, .menu_active = true, .buttons = {.left = true}, .rear_left_func = 1});
        check(!r.timer_reset_clicked, "board idle: no timer reset click inside menus");
    }
    {
        bi::State s{};
        auto r = s.update({.dt_ms = 249, .menu_active = true});
        check(!r.leds, "board idle: LEDs unchanged before 250 ms in a menu");
        r = s.update({.dt_ms = 1, .menu_active = true});
        check(r.leds == false, "board idle: LEDs blink off after 250 ms in a menu");
        r = s.update({.dt_ms = 250, .menu_active = true});
        check(r.leds == true, "board idle: LEDs blink on again");
        s.update({.dt_ms = 250, .menu_active = true});
        r = s.update({.dt_ms = 1});
        check(r.leds == true, "board idle: LEDs forced on when leaving the menu");
        check(!s.update({.dt_ms = 1}).leds, "board idle: LEDs not rewritten on the dashboard");
    }
    {
        bi::State s{};
        s.update({.dt_ms = 10});
        auto r = s.update({.dt_ms = 1000, .power_pressed = true});
        check(!r.shutdown && s.power.progress_pct() == 66, "board idle: power hold progress");
        r = s.update({.dt_ms = 500, .power_pressed = true});
        check(r.shutdown, "board idle: power hold triggers shutdown");
    }
    {
        constexpr uint16_t up_ok = tn::kKeyNavUp | tn::kKeyNavOk;
        check(bi::menu_keys(up_ok, true) == tn::kKeyNavUp, "board idle: swipe clears OK in menus");
        check(bi::menu_keys(up_ok, false) == up_ok, "board idle: keys unchanged outside menus");
        check(bi::menu_keys(tn::kKeyNavOk, true) == tn::kKeyNavOk, "board idle: OK alone kept in menus");
    }
    {
        wm::logic::display::Framebuffer fb{};
        bi::render_overlay(fb, 0);
        bool empty = true;
        for (const auto b : fb.data()) {
            empty = empty && b == 0;
        }
        check(empty, "board idle: no overlay at 0 %");
        bi::render_overlay(fb, 50);
        empty = true;
        for (const auto b : fb.data()) {
            empty = empty && b == 0;
        }
        check(!empty, "board idle: overlay drawn while the power button is held");
    }
    {
        namespace ft = wm::logic::flight_timer;
        ft::State s{};
        s.elapsed = ft::Seconds{42};
        const auto cue = ft::update(s, ft::Input{.reset_clicked = true, .dt = {10}});
        check(cue == wm::logic::buzzer::Cue::TimerReset && s.elapsed.value == 0,
              "flight timer: rear button click resets the timer");
        check(ft::update(s, ft::Input{.reset_clicked = true, .dt = {10}}) != wm::logic::buzzer::Cue::TimerReset,
              "flight timer: reset click ignored during the cooldown");
    }
}
