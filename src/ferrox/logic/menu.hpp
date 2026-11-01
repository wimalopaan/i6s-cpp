#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/curve.hpp"
#include "ferrox/logic/format.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/glyphs.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/labels.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/widgets.hpp"

namespace ferrox::logic::menu {

using display::Framebuffer;
using ferrox::logic::kKeyOk;
using safety::kKeyCancel;
using ModelName = std::array<uint8_t, 10>;
using ModelNames = std::array<ModelName, storage::kNumModels>;

inline constexpr uint16_t kKeyDown = 1u << 8;
inline constexpr uint16_t kKeyUp = 1u << 9;
inline constexpr uint16_t kNavMask = kKeyDown | kKeyUp | kKeyOk | kKeyCancel;
inline constexpr uint32_t kRepeatDelayMs = 300;
inline constexpr uint32_t kRepeatIntervalMs = 70;

inline constexpr std::size_t kMainVisible = 3;
inline constexpr std::size_t kListVisible = 4;

// Items after "Model Setup" are listed for layout parity but have no handler yet.
inline constexpr std::array<std::string_view, 12> kMainItems{
    "Model Select",   "Model Setup",   "Dual Rate/Expo", "Throttle Curve", "Wing / Mixer",  "Aux Channels",
    "Channel Reverse", "Radio Setup",  "Channel Monitor", "Calibration",   "Analog Diag",   "System Info",
};
inline constexpr std::size_t kItemModelSelect = 0;
inline constexpr std::size_t kItemModelSetup = 1;

// Rows of the model setup screen. Each row maps to one slot of the 4-row view.
enum class SetupRow : uint8_t { Name = 0, Type, ArmSwitch, Timer };
inline constexpr std::size_t kSetupRows = 4;
inline constexpr uint8_t kNameLen = 10;
inline constexpr uint8_t kModelTypeCount = 5;
inline constexpr uint8_t kArmSwitchCount = 23;
inline constexpr uint16_t kTimerStepSecs = 30;
inline constexpr uint16_t kTimerMaxSecs = 3600;

enum class Screen : uint8_t {
    Closed,
    Main,
    ModelSelect,
    ModelSetup,
    DualRate,
    ThrottleCurve,
    ChannelReverse,
    ChannelMonitor,
    DiagAnalog,
    SystemInfo,
};

inline constexpr std::size_t kItemDualRate = 2;
inline constexpr std::size_t kItemThrottleCurve = 3;
inline constexpr std::size_t kItemChannelReverse = 6;
inline constexpr std::size_t kItemChannelMonitor = 8;
inline constexpr std::size_t kItemCalibration = 9;
inline constexpr std::size_t kItemDiagAnalog = 10;
inline constexpr std::size_t kItemSystemInfo = 11;
inline constexpr std::size_t kMonitorPages = 3;
inline constexpr std::size_t kMonitorRows = 6;
inline constexpr std::size_t kDiagPages = 2;
inline constexpr std::size_t kAdcFullScale = 4095;

// Live values shown by the diagnostic screens. Filled by the caller each frame.
struct Live {
    std::array<uint16_t, kNumChannels> rf{};  // output channels in us
    input::Raw adc{};
    std::string_view mcu_name{};
};

enum class Action : uint8_t {
    None,
    CloseMenu,
    EnterModelSelect,  // caller refreshes ModelNames before the next render
    SwitchModel,       // index = target model slot
    SaveActiveModel,
    StartCalibration,  // menu closes; caller starts the calibration wizard
};

struct Event {
    Action action{Action::None};
    uint8_t index{0};
};

struct Input {
    uint16_t pressed;  // edge-detected keys
    uint16_t held;
    uint32_t now_ms;
};

struct ListPos {
    uint8_t selected{0};
    uint8_t scroll{0};
};

struct Repeat {
    bool active{false};
    uint32_t press_ms{0};
    uint32_t last_ms{0};
};

struct State {
    Screen screen{Screen::Closed};
    bool waiting_release{false};
    ListPos main{};
    ListPos models{};
    ListPos setup{};
    ListPos dual_rate{};
    ListPos throttle_curve{};  // 0 Pts, 1 Smooth, 2.. curve points
    ListPos reverse{};
    uint8_t axis{0};  // dual rate axis: 0 Roll, 1 Pitch, 2 Yaw
    uint8_t page{0};  // page of the channel monitor or analog diagnostics
    bool editing{false};
    uint8_t name_cursor{0};
    Repeat up{};
    Repeat down{};
};

struct Nav {
    bool up{false};
    bool down{false};
    bool ok{false};
    bool cancel{false};
};

// Next/previous character in the name editor cycle: ' ', 'A'..'Z', '0'..'9', '-', '_'.
constexpr char next_ascii(char c) {
    if (c == ' ') return 'A';
    if (c >= 'A' && c <= 'Y') return static_cast<char>(c + 1);
    if (c == 'Z') return '0';
    if (c >= '0' && c <= '8') return static_cast<char>(c + 1);
    if (c == '9') return '-';
    if (c == '-') return '_';
    return ' ';
}

constexpr char prev_ascii(char c) {
    if (c == ' ') return '_';
    if (c == '_') return '-';
    if (c == '-') return '9';
    if (c >= '1' && c <= '9') return static_cast<char>(c - 1);
    if (c == '0') return 'Z';
    if (c >= 'B' && c <= 'Z') return static_cast<char>(c - 1);
    if (c == 'A') return ' ';
    return ' ';
}

constexpr void open(State& s) {
    s = State{};
    s.screen = Screen::Main;
    // Keys that opened the menu must be released before they count as input.
    s.waiting_release = true;
}

// Moves the selection one step with wrap-around and keeps it inside the visible window.
constexpr void step_list(ListPos& p, std::size_t count, std::size_t visible, bool down) {
    if (count == 0) {
        return;
    }
    if (down) {
        if (p.selected + 1 < count) {
            ++p.selected;
            if (p.selected >= p.scroll + visible) {
                p.scroll = static_cast<uint8_t>(p.selected - visible + 1);
            }
        } else {
            p.selected = 0;
            p.scroll = 0;
        }
    } else {
        if (p.selected > 0) {
            --p.selected;
            if (p.selected < p.scroll) {
                p.scroll = p.selected;
            }
        } else {
            p.selected = static_cast<uint8_t>(count - 1);
            p.scroll = static_cast<uint8_t>(count > visible ? count - visible : 0);
        }
    }
}

// Reports a step on press, then again after the repeat delay at the repeat interval.
constexpr bool repeat_step(Repeat& r, bool pressed, bool held, uint32_t now) {
    if (!held) {
        r.active = false;
        return false;
    }
    if (pressed) {
        r = Repeat{true, now, now};
        return true;
    }
    if (r.active && now - r.press_ms >= kRepeatDelayMs && now - r.last_ms >= kRepeatIntervalMs) {
        r.last_ms = now;
        return true;
    }
    return false;
}

constexpr uint8_t window_scroll(std::size_t selected, std::size_t visible) {
    return static_cast<uint8_t>(selected >= visible ? selected - visible + 1 : 0);
}

constexpr Event update_main(State& s, const Nav& n, uint8_t active_model) {
    if (n.up != n.down) {
        step_list(s.main, kMainItems.size(), kMainVisible, n.down);
    }
    if (n.cancel) {
        s.screen = Screen::Closed;
        return {Action::CloseMenu, 0};
    }
    if (n.ok) {
        if (s.main.selected == kItemModelSelect) {
            s.screen = Screen::ModelSelect;
            s.models = ListPos{};
            s.models.selected = active_model;
            s.models.scroll = window_scroll(active_model, kListVisible);
            return {Action::EnterModelSelect, 0};
        }
        if (s.main.selected == kItemModelSetup) {
            s.screen = Screen::ModelSetup;
            s.setup = ListPos{};
            s.editing = false;
            s.name_cursor = 0;
        }
        if (s.main.selected == kItemDualRate) {
            s.screen = Screen::DualRate;
            s.dual_rate = ListPos{};
            s.axis = 0;
            s.editing = false;
        }
        if (s.main.selected == kItemThrottleCurve) {
            s.screen = Screen::ThrottleCurve;
            s.throttle_curve = ListPos{};
            s.editing = false;
        }
        if (s.main.selected == kItemChannelReverse) {
            s.screen = Screen::ChannelReverse;
            s.reverse = ListPos{};
        }
        if (s.main.selected == kItemChannelMonitor) {
            s.screen = Screen::ChannelMonitor;
            s.page = 0;
        }
        if (s.main.selected == kItemCalibration) {
            s.screen = Screen::Closed;
            return {Action::StartCalibration, 0};
        }
        if (s.main.selected == kItemDiagAnalog) {
            s.screen = Screen::DiagAnalog;
            s.page = 0;
        }
        if (s.main.selected == kItemSystemInfo) {
            s.screen = Screen::SystemInfo;
        }
    }
    return {};
}

constexpr Event update_models(State& s, const Nav& n) {
    if (n.up != n.down) {
        step_list(s.models, storage::kNumModels, kListVisible, n.down);
    }
    if (n.cancel) {
        s.screen = Screen::Main;
        return {};
    }
    if (n.ok) {
        s.screen = Screen::Main;
        return {Action::SwitchModel, s.models.selected};
    }
    return {};
}

constexpr Event update_setup(State& s, const Nav& n, storage::ModelConfig& m) {
    const auto row = static_cast<SetupRow>(s.setup.selected);
    const Event save{Action::SaveActiveModel, 0};

    if (!s.editing) {
        if (n.up != n.down) {
            step_list(s.setup, kSetupRows, kSetupRows, n.down);
        }
        if (n.cancel) {
            s.screen = Screen::Main;
            return save;
        }
        if (n.ok) {
            switch (static_cast<SetupRow>(s.setup.selected)) {
                case SetupRow::Name:
                    s.editing = true;
                    s.name_cursor = 0;
                    break;
                case SetupRow::Type:
                    m.model_type = static_cast<uint8_t>((m.model_type + 1) % kModelTypeCount);
                    return save;
                case SetupRow::ArmSwitch:
                case SetupRow::Timer:
                    s.editing = true;
                    break;
            }
        }
        return {};
    }

    switch (row) {
        case SetupRow::Name:
            if (n.up) {
                auto& c = m.name[s.name_cursor];
                c = static_cast<uint8_t>(next_ascii(static_cast<char>(c)));
            }
            if (n.down) {
                auto& c = m.name[s.name_cursor];
                c = static_cast<uint8_t>(prev_ascii(static_cast<char>(c)));
            }
            if (n.ok) {
                if (++s.name_cursor >= kNameLen) {
                    s.name_cursor = 0;
                    s.editing = false;
                    return save;
                }
            }
            if (n.cancel) {
                s.name_cursor = 0;
                s.editing = false;
                return save;
            }
            break;
        case SetupRow::ArmSwitch:
            if (n.up) {
                m.arm_switch = static_cast<uint8_t>((m.arm_switch + 1) % kArmSwitchCount);
            }
            if (n.down) {
                m.arm_switch = static_cast<uint8_t>((m.arm_switch + kArmSwitchCount - 1) % kArmSwitchCount);
            }
            if (n.ok || n.cancel) {
                s.editing = false;
                return save;
            }
            break;
        case SetupRow::Timer:
            if (n.up && m.timer_secs <= kTimerMaxSecs - kTimerStepSecs) {
                m.timer_secs = static_cast<uint16_t>(m.timer_secs + kTimerStepSecs);
            }
            if (n.down && m.timer_secs >= kTimerStepSecs) {
                m.timer_secs = static_cast<uint16_t>(m.timer_secs - kTimerStepSecs);
            }
            if (n.ok || n.cancel) {
                s.editing = false;
                return save;
            }
            break;
        case SetupRow::Type:
            break;
    }
    return {};
}

// Advances the menu state for one tick. `st` is edited in place; the caller persists it when told to.
inline constexpr std::size_t kDrFields = 6;
inline constexpr std::array<std::string_view, kDrFields> kDrLabels{
    "Switch:", "Channel:", "Hi Rate:", "Lo Rate:", "Hi Expo:", "Lo Expo:",
};

// Mirrors the firmware rules: SA..SD need a 2/3-position or instant-trim input, SE/SF need the external switch board.
constexpr bool is_dr_switch_available(uint8_t sw, const storage::RadioConfig& r) {
    if (sw == 0) {
        return true;
    }
    if (sw <= 4) {
        const input::AdcMode mode = input::resolve_mode(sw - 1u, r.adc_modes[sw - 1u]);
        return mode == input::AdcMode::TwoPos || mode == input::AdcMode::ThreePos ||
               mode == input::AdcMode::InstantTrim;
    }
    if (sw <= 6) {
        return r.ext_switches != 0;
    }
    return false;
}

// Next (down = false) or previous (down = true) available switch code, wrapping over 0..6.
constexpr uint8_t step_dr_switch(const storage::RadioConfig& r, uint8_t cur, bool down) {
    constexpr uint8_t kCount = 7;
    for (uint8_t i = 1; i < kCount; ++i) {
        const auto cand = static_cast<uint8_t>(down ? (cur + kCount - i) % kCount : (cur + i) % kCount);
        if (is_dr_switch_available(cand, r)) {
            return cand;
        }
    }
    return cur;
}

// One repeat step up or down that stays inside [min, max]. Values off the step grid are not snapped.
template <typename T>
constexpr void step_clamped(T& v, int min, int max, int step, bool up, bool down) {
    const int cur = v;
    if (up && cur + step <= max) {
        v = static_cast<T>(cur + step);
    } else if (down && cur - step >= min) {
        v = static_cast<T>(cur - step);
    }
}

constexpr Event update_dual_rate(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r) {
    if (n.cancel) {
        if (s.editing) {
            s.editing = false;
            return {};
        }
        s.screen = Screen::Main;
        return {Action::SaveActiveModel, 0};
    }
    if (!s.editing) {
        if (n.up != n.down) {
            step_list(s.dual_rate, kDrFields, kListVisible, n.down);
        }
        if (n.ok) {
            s.editing = true;
        }
        return {};
    }

    const std::size_t axis = s.axis;
    switch (s.dual_rate.selected) {
        case 0:
            if (n.up) {
                m.dr_switch = step_dr_switch(r, m.dr_switch, false);
            } else if (n.down) {
                m.dr_switch = step_dr_switch(r, m.dr_switch, true);
            }
            break;
        case 1:
            if (n.up || n.down) {
                s.axis = static_cast<uint8_t>((s.axis + 1) % 3);
            }
            break;
        case 2: step_clamped(m.dr_high[axis], 50, 100, 5, n.up, n.down); break;
        case 3: step_clamped(m.dr_low[axis], 30, 100, 5, n.up, n.down); break;
        case 4: step_clamped(m.expo_high[axis], -100, 100, 5, n.up, n.down); break;
        case 5: step_clamped(m.expo_low[axis], -100, 100, 5, n.up, n.down); break;
        default: break;
    }
    if (n.ok) {
        s.editing = false;
    }
    return {};
}

constexpr Event update_throttle_curve(State& s, const Nav& n, storage::ModelConfig& m) {
    const std::size_t items = 2 + curve::point_count(m.thr_curve_pts);
    const uint8_t sel = s.throttle_curve.selected;
    if (s.editing) {
        if (n.cancel) {
            s.editing = false;
            return {Action::SaveActiveModel, 0};
        }
        const std::size_t pt = sel >= 2 ? sel - 2u : 0u;
        step_clamped(m.thr_curve[pt], 0, 100, 1, n.up, n.down);
        if (n.ok) {
            if (sel + 1u < items) {
                s.throttle_curve.selected = static_cast<uint8_t>(sel + 1);
            } else {
                s.editing = false;
            }
            return {Action::SaveActiveModel, 0};
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        return {Action::SaveActiveModel, 0};
    }
    if (n.up != n.down) {
        const std::size_t next = n.down ? (sel + 1u) % items : (sel == 0 ? items - 1 : sel - 1u);
        s.throttle_curve.selected = static_cast<uint8_t>(next);
    }
    if (n.ok) {
        if (sel == 0) {
            curve::advance_mode(m.thr_curve, m.thr_curve_pts);
            return {Action::SaveActiveModel, 0};
        }
        if (sel == 1) {
            m.thr_curve_smooth = m.thr_curve_smooth != 0 ? 0 : 1;
            return {Action::SaveActiveModel, 0};
        }
        s.editing = true;
    }
    return {};
}

constexpr Event update_channel_reverse(State& s, const Nav& n, storage::ModelConfig& m) {
    if (n.up != n.down) {
        step_list(s.reverse, kNumChannels, kListVisible, n.down);
    }
    if (n.ok) {
        m.channel_reverse ^= 1u << s.reverse.selected;
    }
    if (n.cancel) {
        s.screen = Screen::Main;
        return {Action::SaveActiveModel, 0};
    }
    return {};
}

constexpr Event update_monitor(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
        return {};
    }
    if (n.down) {
        s.page = static_cast<uint8_t>((s.page + 1) % kMonitorPages);
    }
    if (n.up) {
        s.page = static_cast<uint8_t>((s.page + kMonitorPages - 1) % kMonitorPages);
    }
    return {};
}

constexpr Event update_diag(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
        return {};
    }
    if (n.up) {
        s.page = static_cast<uint8_t>((s.page + kDiagPages - 1) % kDiagPages);
    } else if (n.down || n.ok) {
        s.page = static_cast<uint8_t>((s.page + 1) % kDiagPages);
    }
    return {};
}

constexpr Event update_system(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
    }
    return {};
}

// Advances the menu state for one tick. `st` is edited in place; the caller persists it when told to.
constexpr Event update(State& s, const Input& in, storage::RadioStorage& st) {
    if (s.screen == Screen::Closed) {
        return {};
    }
    if (s.waiting_release) {
        if ((in.held & kNavMask) == 0) {
            s.waiting_release = false;
        }
        return {};
    }

    const bool up_held = (in.held & kKeyUp) != 0;
    const bool down_held = (in.held & kKeyDown) != 0;
    const bool up_pressed = (in.pressed & kKeyUp) != 0;
    const bool down_pressed = (in.pressed & kKeyDown) != 0;
    Nav n{};
    n.up = repeat_step(s.up, up_pressed, up_held, in.now_ms);
    n.down = repeat_step(s.down, down_pressed, down_held, in.now_ms);
    n.ok = (in.pressed & kKeyOk) != 0;
    n.cancel = (in.pressed & kKeyCancel) != 0;

    switch (s.screen) {
        case Screen::Main: return update_main(s, n, st.radio.active_model);
        case Screen::ModelSelect: return update_models(s, n);
        case Screen::ModelSetup: return update_setup(s, n, st.active_model);
        case Screen::DualRate: return update_dual_rate(s, n, st.active_model, st.radio);
        case Screen::ThrottleCurve: return update_throttle_curve(s, n, st.active_model);
        case Screen::ChannelReverse: return update_channel_reverse(s, n, st.active_model);
        case Screen::ChannelMonitor: return update_monitor(s, n);
        case Screen::DiagAnalog: return update_diag(s, n);
        case Screen::SystemInfo: return update_system(s, n);
        case Screen::Closed: break;
    }
    return {};
}

// Name without the NUL terminator, trailing spaces kept.
constexpr format::Text<10> name_text(const ModelName& name) {
    format::Text<10> t;
    for (const uint8_t c : name) {
        if (c == 0) {
            break;
        }
        t.push(static_cast<char>(c));
    }
    return t;
}

constexpr format::Text<8> counter_text(std::size_t cur, std::size_t total) {
    format::Text<8> t;
    format::detail::push_uint(t, static_cast<unsigned>(cur));
    t.push('/');
    format::detail::push_uint(t, static_cast<unsigned>(total));
    return t;
}

// "Mnn:name" with a trailing '*' for the active model.
constexpr format::Text<20> model_row_text(std::size_t idx, const ModelName& name, bool active) {
    format::Text<20> t;
    t.push('M');
    const auto num = format::format_dec2(static_cast<unsigned>(idx) + 1u);
    for (const char c : num.view()) {
        t.push(c);
    }
    t.push(':');
    const auto text = name_text(name);
    for (const char c : format::trim_spaces(text.view())) {
        t.push(c);
    }
    if (active) {
        t.push('*');
    }
    return t;
}

// "mm:ss", minutes capped at 99.
constexpr format::Text<5> timer_text(uint16_t secs) {
    format::Text<5> t;
    const auto mm = format::format_dec2(std::min<unsigned>(secs / 60u, 99u));
    const auto ss = format::format_dec2(secs % 60u);
    for (const char c : mm.view()) {
        t.push(c);
    }
    t.push(':');
    for (const char c : ss.view()) {
        t.push(c);
    }
    return t;
}

// 3-slot icon row: 13 px pitch, icon at x = 4, label at x = 19. Icon is inverted on the highlight bar.
constexpr void draw_icon_row(Framebuffer& fb, int slot, bool selected, const glyphs::Glyph12& icon,
                             std::string_view label) {
    const int y = 13 + slot * 14;
    if (selected) {
        fb.fill_rect(2, y, 121, 13, true);
    }
    glyphs::draw_glyph_12x12(fb, 4, y + 1, icon, !selected);
    fb.draw_str_6x10(19, y + 3, label, selected);
}

constexpr void render_main(Framebuffer& fb, const State& s) {
    widgets::draw_header(fb, "SETTINGS MENU");
    for (std::size_t slot = 0; slot < kMainVisible; ++slot) {
        const std::size_t idx = s.main.scroll + slot;
        if (idx >= kMainItems.size()) {
            break;
        }
        draw_icon_row(fb, static_cast<int>(slot), idx == s.main.selected, glyphs::kMenuGlyphs[idx], kMainItems[idx]);
    }
    widgets::draw_scrollbar(fb, s.main.selected, static_cast<int>(kMainItems.size()), 13, 40);
    const auto pos = counter_text(s.main.selected + 1u, kMainItems.size());
    widgets::draw_footer_split(fb, "[OK] Select   [ESC] Exit", pos.view());
}

constexpr void render_models(Framebuffer& fb, const State& s, const ModelNames& names,
                             const storage::RadioStorage& st) {
    widgets::draw_header(fb, "SELECT MODEL (1-20)");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.models.scroll + slot;
        if (idx >= storage::kNumModels) {
            break;
        }
        const bool active = idx == st.radio.active_model;
        const auto& name = active ? st.active_model.name : names[idx];
        const auto label = model_row_text(idx, name, active);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.models.selected, label.view(), {}, 0);
    }
    widgets::draw_scrollbar(fb, s.models.selected, static_cast<int>(storage::kNumModels), 14, 36);
    widgets::draw_footer(fb, "[OK] Load     [ESC] Back");
}

constexpr void render_setup(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "MODEL SETUP");
    constexpr int kValueX = 40;

    for (int row = 0; row < static_cast<int>(kSetupRows); ++row) {
        const bool selected = !s.editing && row == s.setup.selected;
        const int y = 14 + row * 9;
        switch (static_cast<SetupRow>(row)) {
            case SetupRow::Name: {
                const auto name = name_text(m.name);
                widgets::draw_list_row(fb, row, selected, "Name:", name.view(), kValueX);
                if (s.editing && row == s.setup.selected) {
                    const int char_x = kValueX + s.name_cursor * 6;
                    const uint8_t c = m.name[s.name_cursor];
                    const std::array<char, 1> one{c == 0 ? ' ' : static_cast<char>(c)};
                    fb.fill_rect(char_x - 1, y, 8, 9, true);
                    fb.draw_str_6x10(char_x, y, std::string_view(one.data(), 1), true);
                }
                break;
            }
            case SetupRow::Type:
                widgets::draw_list_row(fb, row, selected, "Type:", labels::model_type_label(m.model_type), kValueX);
                break;
            case SetupRow::ArmSwitch: {
                const std::string_view arm =
                    m.arm_switch == 0 ? std::string_view{"NONE"} : labels::switch_cond_label(m.arm_switch);
                widgets::draw_list_row(fb, row, selected, "Arm Sw:", arm, 52);
                break;
            }
            case SetupRow::Timer: {
                if (m.timer_secs == 0) {
                    widgets::draw_list_row(fb, row, selected, "Timer:", "OFF", kValueX);
                } else {
                    const auto t = timer_text(m.timer_secs);
                    widgets::draw_list_row(fb, row, selected, "Timer:", t.view(), kValueX);
                }
                break;
            }
        }
    }

    if (!s.editing) {
        widgets::draw_footer(fb, "[OK] Select    [ESC] Back");
    } else if (static_cast<SetupRow>(s.setup.selected) == SetupRow::Name) {
        widgets::draw_footer(fb, "[OK] Next Char [ESC] Done");
    } else {
        widgets::draw_footer(fb, "[OK] Done   [UP/DN] Value");
    }
}

constexpr std::array<std::string_view, kMonitorPages> kMonitorTitles{"CHANNELS (1-6)", "CHANNELS (7-12)",
                                                                   "CHANNELS (13-18)"};
constexpr std::array<std::string_view, kDiagPages> kDiagTitles{"ANALOG (1-6)", "ANALOG (7-11)"};
// Raw ADC index of each row. Page 1 starts at raw index 6 (PA6).
constexpr std::array<std::array<std::string_view, 6>, kDiagPages> kDiagNames{{
    {"RH:AIL", "RV:ELE", "LV:THR", "LH:RUD", "SW:SA ", "SW:SB "},
    {"POT:V1", "POT:V2", "SW:SC ", "SW:SD ", "VBAT  ", ""},
}};
constexpr std::array<std::size_t, kDiagPages> kDiagRows{6, 5};

// Copies a short label or number into a fixed buffer so the caller can hold the value for one row.
constexpr format::Text<6> to_text(std::string_view sv) {
    format::Text<6> t;
    for (const char c : sv) {
        t.push(c);
    }
    return t;
}

constexpr format::Text<6> dr_value(const storage::ModelConfig& m, std::size_t axis, std::size_t idx) {
    switch (idx) {
        case 0: return to_text(labels::kDrSwitchNames[std::min<std::size_t>(m.dr_switch, 6)]);
        case 1: return to_text(labels::kAxisNames[axis]);
        case 2: return to_text(format::format_int(m.dr_high[axis]).view());
        case 3: return to_text(format::format_int(m.dr_low[axis]).view());
        case 4: return to_text(format::format_int(m.expo_high[axis]).view());
        default: return to_text(format::format_int(m.expo_low[axis]).view());
    }
}

constexpr void render_dual_rate(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "DUAL RATE / EXPO");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.dual_rate.scroll + slot;
        if (idx >= kDrFields) {
            break;
        }
        const format::Text<6> value = dr_value(m, s.axis, idx);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.dual_rate.selected, kDrLabels[idx], value.view(),
                               60);
    }
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Value" : "[OK] Edit   [ESC] Exit");
}

// "P01:  50%" as on the device: point number, then the value right-aligned in three columns.
constexpr format::Text<9> curve_point_label(std::size_t idx, uint8_t val) {
    const auto digit = [](std::size_t d) { return static_cast<char>('0' + d); };
    const std::size_t num = idx + 1;
    const std::size_t hundreds = val / 100;
    const std::size_t tens = (val / 10) % 10;
    format::Text<9> t{};
    t.push('P');
    t.push(digit(num / 10));
    t.push(digit(num % 10));
    t.push(':');
    t.push(' ');
    t.push(hundreds != 0 ? '1' : ' ');
    t.push(hundreds != 0 || tens != 0 ? digit(tens) : ' ');
    t.push(digit(val % 10));
    t.push('%');
    return t;
}

constexpr void render_throttle_curve(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "THROTTLE CURVE");
    const std::size_t pts = curve::point_count(m.thr_curve_pts);
    const std::size_t sel = s.throttle_curve.selected;

    const bool sel_pts = !s.editing && sel == 0;
    if (sel_pts) {
        fb.fill_rect(2, 13, 70, 9, true);
    }
    fb.draw_str_6x10(4, 13, "Pts:", sel_pts);
    fb.draw_str_6x10(32, 13, pts == 9 ? "9-PT" : (pts == 7 ? "7-PT" : "5-PT"), sel_pts);

    const bool sel_crv = !s.editing && sel == 1;
    if (sel_crv) {
        fb.fill_rect(2, 23, 70, 9, true);
    }
    fb.draw_str_6x10(4, 23, "Crv:", sel_crv);
    fb.draw_str_6x10(32, 23, m.thr_curve_smooth != 0 ? "SMOOTH" : "LINEAR", sel_crv);

    if (sel >= 2) {
        const format::Text<9> label = curve_point_label(sel - 2, m.thr_curve[sel - 2]);
        if (s.editing) {
            fb.fill_rect(2, 35, 70, 11, true);
            fb.draw_str_6x10(4, 37, label.view(), true);
        } else {
            fb.draw_rect(2, 35, 70, 11, true);
            fb.draw_str_6x10(4, 37, label.view(), false);
        }
    } else {
        fb.draw_str_6x10(4, 37, pts == 9 ? "Pts: 1..9" : (pts == 7 ? "Pts: 1..7" : "Pts: 1..5"), false);
    }

    // Graph box 49x37 at (76,13). Plot area 47 px wide, output 0..34 px high.
    fb.draw_rect(76, 13, 49, 37, true);
    const bool smooth = m.thr_curve_smooth != 0;
    for (int px = 0; px < 47; ++px) {
        const auto input = static_cast<uint16_t>(px * 1000 / 46);
        const uint16_t out = curve::evaluate(input, m.thr_curve_pts, smooth, m.thr_curve);
        fb.set_pixel(77 + px, 48 - out * 34 / 1000, true);
    }
    if (sel >= 2) {
        const std::size_t idx = sel - 2;
        const int input = static_cast<int>(idx * 1000 / (pts - 1));
        const int dot_x = 77 + input * 46 / 1000;
        const int dot_y = 48 - (m.thr_curve[idx] * 10 * 34) / 1000;
        fb.fill_rect(dot_x - 1, dot_y - 1, 3, 3, true);
    }

    if (s.editing) {
        widgets::draw_footer(fb, "[OK] Next Pt   [ESC] Done");
    } else if (sel >= 2) {
        widgets::draw_footer(fb, "[OK] Edit Pt   [ESC] Back");
    } else {
        widgets::draw_footer(fb, "[OK] Toggle    [ESC] Back");
    }
}

constexpr void render_channel_reverse(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "CHANNEL REVERSE");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.reverse.scroll + slot;
        if (idx >= kNumChannels) {
            break;
        }
        const bool reversed = (m.channel_reverse & (1u << idx)) != 0;
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.reverse.selected, labels::kChannelNames[idx],
                               reversed ? "REVERSE" : "NORMAL ", 76);
    }
    widgets::draw_scrollbar(fb, s.reverse.selected, static_cast<int>(kNumChannels), 14, 36);
    widgets::draw_footer(fb, "[OK] Toggle   [ESC] Back");
}

constexpr void render_monitor(Framebuffer& fb, const State& s, const Live& live, const storage::ModelConfig& m) {
    widgets::draw_header(fb, kMonitorTitles[s.page]);
    constexpr int kMinUs = kChannelMinUs;
    constexpr int kSpanUs = kChannelSpanUs;
    for (std::size_t row = 0; row < kMonitorRows; ++row) {
        const std::size_t ch = s.page * kMonitorRows + row;
        const int us = std::clamp<int>(live.rf[ch], kMinUs, kMinUs + kSpanUs);
        const int fill = std::min((us - kMinUs) * 38 / kSpanUs, 38);
        const auto label = labels::channel_label(ch, m);
        widgets::draw_gauge_row(fb, 12 + static_cast<int>(row) * 7, label.view(), fill, static_cast<uint16_t>(us));
    }
    widgets::draw_footer(fb, "[UP/DN] Page  [ESC] Back");
}

constexpr void render_diag(Framebuffer& fb, const State& s, const Live& live) {
    widgets::draw_header(fb, kDiagTitles[s.page]);
    const std::size_t start = s.page == 0 ? 0 : 6;
    for (std::size_t row = 0; row < kDiagRows[s.page]; ++row) {
        const uint16_t raw = std::min<uint16_t>(live.adc[start + row], kAdcFullScale);
        const int fill = std::min(static_cast<int>(raw) * 38 / static_cast<int>(kAdcFullScale), 38);
        widgets::draw_gauge_row(fb, 13 + static_cast<int>(row) * 7, kDiagNames[s.page][row], fill, raw);
    }
    widgets::draw_footer(fb, "[UP/DN/OK] Page  [ESC] Back");
}

constexpr void render_system(Framebuffer& fb, std::string_view mcu_name) {
    widgets::draw_header(fb, "SYSTEM INFORMATION");
    fb.draw_str_4x6(4, 14, "MCU:", false);
    fb.draw_str_4x6(48, 14, mcu_name, false);
    fb.draw_str_4x6(4, 22, "Flash: 128KB | SRAM: 16KB", false);
    fb.draw_str_4x6(4, 30, "Profiles: 20 Models (16KB)", false);
    widgets::draw_footer(fb, "[ESC] Back");
}

// Draws the menu screen that is open. Call only while `s.screen != Screen::Closed`.
constexpr void render(Framebuffer& fb, const State& s, const ModelNames& names, const storage::RadioStorage& st,
                      const Live& live) {
    switch (s.screen) {
        case Screen::Main: render_main(fb, s); break;
        case Screen::ModelSelect: render_models(fb, s, names, st); break;
        case Screen::ModelSetup: render_setup(fb, s, st.active_model); break;
        case Screen::DualRate: render_dual_rate(fb, s, st.active_model); break;
        case Screen::ThrottleCurve: render_throttle_curve(fb, s, st.active_model); break;
        case Screen::ChannelReverse: render_channel_reverse(fb, s, st.active_model); break;
        case Screen::ChannelMonitor: render_monitor(fb, s, live, st.active_model); break;
        case Screen::DiagAnalog: render_diag(fb, s, live); break;
        case Screen::SystemInfo: render_system(fb, live.mcu_name); break;
        case Screen::Closed: break;
    }
}

}  // namespace ferrox::logic::menu
