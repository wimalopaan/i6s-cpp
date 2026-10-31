#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/format.hpp"
#include "ferrox/logic/framebuffer.hpp"
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

enum class Screen : uint8_t { Closed, Main, ModelSelect, ModelSetup };

enum class Action : uint8_t {
    None,
    CloseMenu,
    EnterModelSelect,  // caller refreshes ModelNames before the next render
    SwitchModel,       // index = target model slot
    SaveActiveModel,
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

constexpr void render_main(Framebuffer& fb, const State& s) {
    widgets::draw_header(fb, "SETTINGS MENU");
    for (std::size_t slot = 0; slot < kMainVisible; ++slot) {
        const std::size_t idx = s.main.scroll + slot;
        if (idx >= kMainItems.size()) {
            break;
        }
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.main.selected, kMainItems[idx], {}, 0);
    }
    widgets::draw_scrollbar(fb, s.main.selected, static_cast<int>(kMainItems.size()), 14, 27);
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

// Draws the menu screen that is open. Call only while `s.screen != Screen::Closed`.
constexpr void render(Framebuffer& fb, const State& s, const ModelNames& names, const storage::RadioStorage& st) {
    switch (s.screen) {
        case Screen::Main: render_main(fb, s); break;
        case Screen::ModelSelect: render_models(fb, s, names, st); break;
        case Screen::ModelSetup: render_setup(fb, s, st.active_model); break;
        case Screen::Closed: break;
    }
}

}  // namespace ferrox::logic::menu
