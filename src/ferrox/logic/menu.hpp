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
    WingMixer,
    MixerLine,
    GlobalVars,
    AuxChannels,
    RadioSetup,
    InputSetup,
};

inline constexpr std::size_t kItemDualRate = 2;
inline constexpr std::size_t kItemThrottleCurve = 3;
inline constexpr std::size_t kItemWingMixer = 4;
inline constexpr std::size_t kItemAuxChannels = 5;
inline constexpr std::size_t kItemChannelReverse = 6;
inline constexpr std::size_t kAuxChannelCount = 14;  // CH5..CH18
// Aux source IDs offered by the Rust AUX_SOURCES list (None, sticks, VRA/VRB, SA..SD, SE, SF, SWI, SWJ, VRC, VRD).
inline constexpr std::array<uint8_t, 17> kAuxSources{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 31, 32, 35, 36, 37, 38};
inline constexpr std::size_t kItemChannelMonitor = 8;
inline constexpr std::size_t kItemCalibration = 9;
inline constexpr std::size_t kItemDiagAnalog = 10;
inline constexpr std::size_t kItemSystemInfo = 11;
inline constexpr std::size_t kItemRadioSetup = 7;
inline constexpr std::size_t kRadioRows = 16;
inline constexpr std::size_t kRadioInputRow = 13;  // opens the Input Modes sub-screen
inline constexpr std::size_t kRadioInputMaxRows = 10;
inline constexpr std::size_t kAdcModeCount = 7;
inline constexpr uint8_t kTrainerPpmOut = 0x01;
inline constexpr uint8_t kTrainerPpmIn = 0x02;
inline constexpr std::array<std::string_view, kRadioRows> kRadioLabels{
    "Thr Trim:", "Beeper:",   "Tones:",     "BL Timer:",  "BL Level:", "Contrast:", "Bat Warn:", "USB Mode:",
    "PC13 Pwr:", "Ext Sw:",   "P7 Analog:", "Stick Mode:", "Gimbals:",  "Input Setup", "PPM Out:", "Trainer In:"};
inline constexpr std::array<std::string_view, kRadioInputMaxRows> kAuxInputNames{
    "SA (PA4):", "SB (PA5):", "SC (PB0):", "SD (PB1):", "VRA (PA6):",
    "VRB (PA7):", "VRC (PC2):", "VRD (PC3):", "VRE (PC4):", "VRF (PC5):"};
inline constexpr std::array<std::string_view, kAdcModeCount> kAdcModeNames{
    "DEFAULT", "2-POS", "3-POS", "6-POS", "POT", "POT-D", "INST-TRIM"};
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
    SaveRadioConfig,   // caller persists the radio config and applies LCD backlight and contrast
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
    ListPos wing{};
    ListPos mix_line{};
    ListPos gvars{};
    uint8_t mix_idx{0};  // mix line shown by the line editor, 0..kNumMixLines-1
    ListPos reverse{};
    ListPos aux{};
    ListPos radio{};
    ListPos input_setup{};
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

// Direction of a list step. Only meaningful when exactly one of Nav::up and Nav::down is set.
enum class Dir : uint8_t { Up, Down };

constexpr Dir nav_dir(const Nav& n) { return n.down ? Dir::Down : Dir::Up; }

// Number of list rows that fit on screen (kept apart from the item count).
struct Rows {
    std::size_t value;
};

struct KeyState {
    bool pressed{false};
    bool held{false};
};

struct Range {
    int min;
    int max;
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
constexpr void step_list(ListPos& p, std::size_t count, Rows rows, Dir dir) {
    const std::size_t visible = rows.value;
    if (count == 0) {
        return;
    }
    if (dir == Dir::Down) {
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
constexpr bool repeat_step(Repeat& r, KeyState key, uint32_t now) {
    const bool pressed = key.pressed;
    const bool held = key.held;
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
        step_list(s.main, kMainItems.size(), Rows{kMainVisible}, nav_dir(n));
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
        if (s.main.selected == kItemWingMixer) {
            s.screen = Screen::WingMixer;
            s.wing = ListPos{};
            s.editing = false;
        }
        if (s.main.selected == kItemAuxChannels) {
            s.screen = Screen::AuxChannels;
            s.aux = ListPos{};
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
        if (s.main.selected == kItemRadioSetup) {
            s.screen = Screen::RadioSetup;
            s.radio = ListPos{};
        }
    }
    return {};
}

constexpr Event update_models(State& s, const Nav& n) {
    if (n.up != n.down) {
        step_list(s.models, storage::kNumModels, Rows{kListVisible}, nav_dir(n));
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
            step_list(s.setup, kSetupRows, Rows{kSetupRows}, nav_dir(n));
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
constexpr void step_clamped(T& v, Range range, int step, const Nav& n) {
    const int cur = v;
    if (n.up && cur + step <= range.max) {
        v = static_cast<T>(cur + step);
    } else if (n.down && cur - step >= range.min) {
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
            step_list(s.dual_rate, kDrFields, Rows{kListVisible}, nav_dir(n));
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
        case 2: step_clamped(m.dr_high[axis], Range{50, 100}, 5, n); break;
        case 3: step_clamped(m.dr_low[axis], Range{30, 100}, 5, n); break;
        case 4: step_clamped(m.expo_high[axis], Range{-100, 100}, 5, n); break;
        case 5: step_clamped(m.expo_low[axis], Range{-100, 100}, 5, n); break;
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
        step_clamped(m.thr_curve[pt], Range{0, 100}, 1, n);
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

inline constexpr std::size_t kMixFields = 7;
inline constexpr std::array<std::string_view, kMixFields> kMixLabels{
    "Target:", "Source:", "Wt Src:", "Weight:", "Offset:", "Switch:", "Mode:",
};
inline constexpr std::array<std::string_view, 5> kGvarLabels{"GV1:", "GV2:", "GV3:", "GV4:", "GV5:"};

constexpr input::AdcMode adc_mode_at(const storage::RadioConfig& r, std::size_t idx) {
    return input::resolve_mode(idx, r.adc_modes[idx]);
}

constexpr bool is_two_or_three_pos(input::AdcMode mode) {
    return mode == input::AdcMode::TwoPos || mode == input::AdcMode::ThreePos;
}

constexpr bool is_switch_input(input::AdcMode mode) {
    return is_two_or_three_pos(mode) || mode == input::AdcMode::InstantTrim;
}

constexpr bool is_pot_input(input::AdcMode mode) {
    return mode == input::AdcMode::Pot || mode == input::AdcMode::PotDetent;
}

// Channels 1..18 that drive an output: primary controls, flaperon CH6, assigned aux channels or mix targets.
constexpr bool is_channel_configured(uint8_t ch, const storage::ModelConfig& m) {
    if (ch == 0 || ch > 18) {
        return false;
    }
    if (ch <= 4) {
        return true;
    }
    if (m.wing_tail_mix == 3 && ch == 6) {
        return true;
    }
    if (m.aux_channels[ch - 5u] != 0) {
        return true;
    }
    for (const storage::MixLine& line : m.mixes) {
        if (line.target_ch == ch) {
            return true;
        }
    }
    return false;
}

constexpr bool is_source_available(uint8_t src, const storage::RadioConfig& r, const storage::ModelConfig& m) {
    const bool ext_adc = r.ext_adc != 0;
    if (src <= 4) {
        return true;  // None, Roll, Pitch, Thr, Yaw
    }
    if (src == 5) {
        return is_pot_input(adc_mode_at(r, 4));  // VRA
    }
    if (src == 6) {
        return is_pot_input(adc_mode_at(r, 5));  // VRB
    }
    if (src <= 10) {
        return is_switch_input(adc_mode_at(r, src - 7u));  // SA..SD
    }
    if (src <= 15) {
        return true;  // MAX, CH1..CH4
    }
    if (src <= 29) {
        return is_channel_configured(static_cast<uint8_t>(src - 11), m);  // CH5..CH18
    }
    if (src == 30) {
        return true;  // Thr+
    }
    if (src <= 32) {
        return r.ext_switches != 0;  // SE, SF
    }
    if (src <= 34) {
        return r.j4_sg_sh_en != 0;  // SG, SH
    }
    if (src == 35) {
        return ext_adc && is_two_or_three_pos(adc_mode_at(r, 6));  // SWI
    }
    if (src == 36) {
        return ext_adc && is_two_or_three_pos(adc_mode_at(r, 7));  // SWJ
    }
    if (src <= 40) {
        return ext_adc && is_pot_input(adc_mode_at(r, src - 31u));  // VRC..VRF
    }
    return src <= 45;  // GV1..GV5
}

constexpr bool is_weight_src_available(uint8_t ws, const storage::ModelConfig& m) {
    if (ws == 0) {
        return true;  // Fixed
    }
    if (ws <= 18) {
        return is_channel_configured(ws, m);
    }
    if (ws <= 36) {
        return is_channel_configured(static_cast<uint8_t>(ws - 18), m);
    }
    return ws <= 41;  // GV1..GV5
}

constexpr bool is_switch_cond_available(uint8_t cond, const storage::RadioConfig& r) {
    switch (cond) {
        case 0: return true;
        case 1:
        case 2: return is_switch_input(adc_mode_at(r, 0));
        case 3:
        case 5: return is_switch_input(adc_mode_at(r, 1));
        case 4: return adc_mode_at(r, 1) == input::AdcMode::ThreePos;
        case 6:
        case 8: return is_switch_input(adc_mode_at(r, 2));
        case 7: return adc_mode_at(r, 2) == input::AdcMode::ThreePos;
        case 9:
        case 10: return is_switch_input(adc_mode_at(r, 3));
        case 11:
        case 12:
        case 13:
        case 14: return r.ext_switches != 0;
        case 15:
        case 16:
        case 17:
        case 18: return r.j4_sg_sh_en != 0;
        case 19:
        case 20: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 6));
        case 21:
        case 22: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 7));
        default: return false;
    }
}

// Next (down = false) or previous (down = true) code that passes `ok`, wrapping over 0..count-1. Returns cur if none does.
template <typename Ok>
constexpr uint8_t step_available(uint8_t cur, std::size_t count, bool down, Ok ok) {
    for (std::size_t i = 1; i < count; ++i) {
        const auto cand = static_cast<uint8_t>(down ? (cur + count - i) % count : (cur + i) % count);
        if (ok(cand)) {
            return cand;
        }
    }
    return cur;
}

constexpr bool wing_has_diff(const storage::ModelConfig& m) {
    return m.wing_tail_mix == 1 || m.wing_tail_mix == 3;
}

// Row of the "GVars" entry. The 32 mix lines follow it.
constexpr std::size_t wing_gvars_row(const storage::ModelConfig& m) {
    return wing_has_diff(m) ? 2 : 1;
}

constexpr std::size_t wing_row_count(const storage::ModelConfig& m) {
    return wing_gvars_row(m) + 1 + storage::kNumMixLines;
}

constexpr Event update_wing_mixer(State& s, const Nav& n, storage::ModelConfig& m) {
    const Event save{Action::SaveActiveModel, 0};
    if (s.editing) {
        if (n.cancel) {
            s.editing = false;
            return save;
        }
        step_clamped(m.template_diff, Range{-100, 100}, 5, n);
        if (n.ok) {
            s.editing = false;
            return save;
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        return save;
    }
    if (n.up != n.down) {
        step_list(s.wing, wing_row_count(m), Rows{kListVisible}, nav_dir(n));
    }
    if (!n.ok) {
        return {};
    }

    const std::size_t sel = s.wing.selected;
    const std::size_t gvars_row = wing_gvars_row(m);
    if (sel == 0) {
        m.wing_tail_mix = static_cast<uint8_t>((m.wing_tail_mix + 1) % 4);
        return save;
    }
    if (wing_has_diff(m) && sel == 1) {
        s.editing = true;
    } else if (sel == gvars_row) {
        s.screen = Screen::GlobalVars;
        s.gvars = ListPos{};
        s.editing = false;
    } else {
        s.mix_idx = static_cast<uint8_t>(sel - (gvars_row + 1));
        s.mix_line = ListPos{};
        s.editing = false;
        s.screen = Screen::MixerLine;
    }
    return {};
}

constexpr Event update_mixer_line(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r) {
    storage::MixLine& mix = m.mixes[s.mix_idx];
    if (!s.editing) {
        if (n.cancel) {
            // Return to the mix row this line came from. The line editor does not save; the Wing screen does.
            const std::size_t row = wing_gvars_row(m) + 1u + s.mix_idx;
            s.screen = Screen::WingMixer;
            s.wing.selected = static_cast<uint8_t>(row);
            s.wing.scroll = window_scroll(row, kListVisible);
            return {};
        }
        if (n.up != n.down) {
            step_list(s.mix_line, kMixFields, Rows{kListVisible}, nav_dir(n));
        }
        if (n.ok) {
            s.editing = true;
        }
        return {};
    }

    if (n.cancel) {
        s.editing = false;
        return {};
    }
    const bool up = n.up && !n.down;
    const bool down = n.down && !n.up;
    switch (s.mix_line.selected) {
        case 0:
            if (up) {
                mix.target_ch = static_cast<uint8_t>((mix.target_ch + 1) % 19);
            } else if (down) {
                mix.target_ch = static_cast<uint8_t>(mix.target_ch == 0 ? 18 : mix.target_ch - 1);
            }
            break;
        case 1:
            if (up || down) {
                mix.source = step_available(mix.source, labels::kSourceNames.size(), down,
                                            [&](uint8_t c) { return is_source_available(c, r, m); });
            }
            break;
        case 2:
            if (up || down) {
                mix.weight_src = step_available(mix.weight_src, labels::kWeightSrcNames.size(), down,
                                                [&](uint8_t c) { return is_weight_src_available(c, m); });
            }
            break;
        case 3: step_clamped(mix.weight, Range{-100, 100}, 5, n); break;
        case 4: step_clamped(mix.offset, Range{-100, 100}, 5, n); break;
        case 5:
            if (up || down) {
                mix.switch_sel = step_available(mix.switch_sel, labels::kSwitchCondNames.size(), down,
                                                [&](uint8_t c) { return is_switch_cond_available(c, r); });
            }
            break;
        case 6:
            if (up) {
                mix.mode = static_cast<uint8_t>((mix.mode + 1) % labels::kMixModeNames.size());
            } else if (down) {
                mix.mode = static_cast<uint8_t>(mix.mode == 0 ? labels::kMixModeNames.size() - 1 : mix.mode - 1);
            }
            break;
        default: break;
    }
    if (n.ok) {
        s.editing = false;
        return {Action::SaveActiveModel, 0};
    }
    return {};
}

constexpr Event update_global_vars(State& s, const Nav& n, storage::ModelConfig& m) {
    const Event save{Action::SaveActiveModel, 0};
    if (s.editing) {
        if (n.cancel) {
            s.editing = false;
            return save;
        }
        step_clamped(m.gvars[s.gvars.selected], Range{-1000, 1000}, 50, n);
        if (n.ok) {
            s.editing = false;
            return save;
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        return save;
    }
    if (n.up != n.down) {
        step_list(s.gvars, m.gvars.size(), Rows{kListVisible}, nav_dir(n));
    }
    if (n.ok) {
        s.editing = true;
    }
    return {};
}

// Next (down = false) or previous (down = true) usable aux source, wrapping like Rust next/prev_available_aux_source.
// An unknown current source is treated as index 0.
template <typename Ok>
constexpr uint8_t step_aux_source(uint8_t cur, bool down, Ok ok) {
    std::size_t pos = 0;
    for (std::size_t i = 0; i < kAuxSources.size(); ++i) {
        if (kAuxSources[i] == cur) {
            pos = i;
            break;
        }
    }
    const std::size_t len = kAuxSources.size();
    for (std::size_t i = 1; i < len; ++i) {
        const std::size_t idx = down ? (pos + len - i) % len : (pos + i) % len;
        if (ok(kAuxSources[idx])) {
            return kAuxSources[idx];
        }
    }
    return cur;
}

constexpr Event update_aux(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r) {
    if (n.cancel && s.editing) {
        s.editing = false;
        return {};
    }
    if (n.cancel) {
        s.screen = Screen::Main;
        return {Action::SaveActiveModel, 0};
    }
    if (!s.editing) {
        if (n.up != n.down) {
            step_list(s.aux, kAuxChannelCount, Rows{kListVisible}, nav_dir(n));
        }
        if (n.ok) {
            s.editing = true;
        }
        return {};
    }
    if (n.up != n.down) {
        auto& src = m.aux_channels[s.aux.selected];
        src = step_aux_source(src, n.down, [&](uint8_t c) { return is_source_available(c, r, m); });
    }
    if (n.ok) {
        s.editing = false;
    }
    return {};
}

constexpr void toggle_flag(uint8_t& v) {
    v = v == 0 ? 1 : 0;
}

// Advances one Radio Setup row with the same cycle rules as the Rust menu.
constexpr void cycle_radio_row(storage::RadioConfig& r, std::size_t row) {
    switch (row) {
        case 0: r.throttle_trim = static_cast<uint8_t>((r.throttle_trim + 1) % 3); break;
        case 1: toggle_flag(r.audio_enabled); break;
        case 2: toggle_flag(r.tone_style); break;
        case 3: r.backlight_timeout = static_cast<uint8_t>((r.backlight_timeout + 1) % 4); break;
        case 4: r.backlight_brightness = r.backlight_brightness >= 10 ? 1 : static_cast<uint8_t>(r.backlight_brightness + 1); break;
        case 5: r.lcd_contrast = r.lcd_contrast >= 50 ? 20 : static_cast<uint8_t>(std::min<unsigned>(r.lcd_contrast + 3u, 50u)); break;
        case 6: r.vbat_warn_deci = r.vbat_warn_deci >= 50 ? 40 : static_cast<uint8_t>(r.vbat_warn_deci + 1); break;
        case 7: r.usb_mode = static_cast<uint8_t>((r.usb_mode + 1) % 4); break;
        case 8: toggle_flag(r.ext_module_pwr); break;
        case 9: toggle_flag(r.ext_switches); break;
        case 10: toggle_flag(r.ext_adc); break;
        case 11: r.stick_mode = static_cast<uint8_t>((r.stick_mode + 1) % 4); break;
        case 12: toggle_flag(r.gimbal_type); break;
        case 14: r.trainer_config ^= kTrainerPpmOut; break;
        case 15: r.trainer_config ^= kTrainerPpmIn; break;
        default: break;
    }
}

constexpr std::size_t input_rows(const storage::RadioConfig& r) {
    return r.ext_adc != 0 ? kRadioInputMaxRows : 6;
}

constexpr Event update_radio_setup(State& s, const Nav& n, storage::RadioConfig& r) {
    if (n.cancel) {
        s.screen = Screen::Main;
        return {};
    }
    if (n.up != n.down) {
        step_list(s.radio, kRadioRows, Rows{kListVisible}, nav_dir(n));
    }
    if (!n.ok) {
        return {};
    }
    if (s.radio.selected == kRadioInputRow) {
        s.screen = Screen::InputSetup;
        s.input_setup = ListPos{};
        return {};
    }
    cycle_radio_row(r, s.radio.selected);
    return {Action::SaveRadioConfig, 0};
}

constexpr Event update_input_setup(State& s, const Nav& n, storage::RadioConfig& r) {
    if (n.cancel) {
        s.screen = Screen::RadioSetup;
        s.radio = ListPos{kRadioInputRow, 10};
        return {};
    }
    const std::size_t count = input_rows(r);
    if (n.up != n.down) {
        step_list(s.input_setup, count, Rows{kListVisible}, nav_dir(n));
    }
    if (!n.ok || s.input_setup.selected >= count) {
        return {};
    }
    auto& mode = r.adc_modes[s.input_setup.selected];
    const uint8_t cur = std::min<uint8_t>(mode, kAdcModeCount - 1);
    mode = static_cast<uint8_t>((cur + 1) % kAdcModeCount);
    return {Action::SaveRadioConfig, 0};
}

constexpr Event update_channel_reverse(State& s, const Nav& n, storage::ModelConfig& m) {
    if (n.up != n.down) {
        step_list(s.reverse, kNumChannels, Rows{kListVisible}, nav_dir(n));
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
    n.up = repeat_step(s.up, KeyState{up_pressed, up_held}, in.now_ms);
    n.down = repeat_step(s.down, KeyState{down_pressed, down_held}, in.now_ms);
    n.ok = (in.pressed & kKeyOk) != 0;
    n.cancel = (in.pressed & kKeyCancel) != 0;

    switch (s.screen) {
        case Screen::Main: return update_main(s, n, st.radio.active_model);
        case Screen::ModelSelect: return update_models(s, n);
        case Screen::ModelSetup: return update_setup(s, n, st.active_model);
        case Screen::DualRate: return update_dual_rate(s, n, st.active_model, st.radio);
        case Screen::ThrottleCurve: return update_throttle_curve(s, n, st.active_model);
        case Screen::AuxChannels: return update_aux(s, n, st.active_model, st.radio);
        case Screen::RadioSetup: return update_radio_setup(s, n, st.radio);
        case Screen::InputSetup: return update_input_setup(s, n, st.radio);
        case Screen::ChannelReverse: return update_channel_reverse(s, n, st.active_model);
        case Screen::ChannelMonitor: return update_monitor(s, n);
        case Screen::DiagAnalog: return update_diag(s, n);
        case Screen::SystemInfo: return update_system(s, n);
        case Screen::WingMixer: return update_wing_mixer(s, n, st.active_model);
        case Screen::MixerLine: return update_mixer_line(s, n, st.active_model, st.radio);
        case Screen::GlobalVars: return update_global_vars(s, n, st.active_model);
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

// Value text of a Radio Setup row, right-aligned by the caller.
constexpr format::Text<12> radio_value(const storage::RadioConfig& r, std::size_t row) {
    format::Text<12> t;
    auto put = [&t](std::string_view sv) {
        for (const char c : sv) {
            t.push(c);
        }
    };
    switch (row) {
        case 0: put(r.throttle_trim == 1 ? "IDLE" : r.throttle_trim == 2 ? "LINEAR" : "OFF (Lock)"); break;
        case 1: put(r.audio_enabled != 0 ? "ENABLED" : "MUTED"); break;
        case 2: put(r.tone_style != 0 ? "RICH" : "SIMPLE"); break;
        case 3:
            put(r.backlight_timeout == 1   ? "15 SEC"
                : r.backlight_timeout == 2 ? "30 SEC"
                : r.backlight_timeout == 3 ? "60 SEC"
                                           : "ALWAYS ON");
            break;
        case 4: {
            format::detail::push_uint(t, std::min<unsigned>(r.backlight_brightness * 10u, 100u));
            t.push('%');
            break;
        }
        case 5: {
            const auto num = format::format_dec2(std::min<unsigned>(r.lcd_contrast, 99u));
            put(num.view());
            break;
        }
        case 6: {
            format::detail::push_uint(t, r.vbat_warn_deci / 10u);
            t.push('.');
            t.push(static_cast<char>('0' + r.vbat_warn_deci % 10u));
            t.push('V');
            break;
        }
        case 7:
            put(r.usb_mode == 1 ? "JOYSTICK" : r.usb_mode == 2 ? "SERIAL" : r.usb_mode == 3 ? "COMPOSITE" : "OFF");
            break;
        case 8: put(r.ext_module_pwr == 0 ? "HIGH (N)" : "LOW (P)"); break;
        case 9: put(r.ext_switches != 0 ? "PC12+PC15" : "OFF"); break;
        case 10: put(r.ext_adc != 0 ? "AD12-15" : "OFF"); break;
        case 11: {
            constexpr std::array<std::string_view, 4> kStickModes{"MODE 1", "MODE 2", "MODE 3", "MODE 4"};
            put(kStickModes[r.stick_mode % 4]);
            break;
        }
        case 12: put(r.gimbal_type != 0 ? "POT" : "HALL"); break;
        case 13: put(">"); break;
        case 14: put((r.trainer_config & kTrainerPpmOut) != 0 ? "ON (PF10)" : "OFF"); break;
        case 15: put((r.trainer_config & kTrainerPpmIn) != 0 ? "ON (SD DN)" : "OFF"); break;
        default: break;
    }
    return t;
}

constexpr void render_radio_setup(Framebuffer& fb, const State& s, const storage::RadioConfig& r) {
    widgets::draw_header(fb, "RADIO SETUP");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.radio.scroll + slot;
        if (idx >= kRadioRows) {
            break;
        }
        const auto value = radio_value(r, idx);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.radio.selected, kRadioLabels[idx], value.view(), 0);
    }
    widgets::draw_scrollbar(fb, s.radio.selected, static_cast<int>(kRadioRows), 14, 36);
    widgets::draw_footer(fb, "[OK] Toggle/Cycle   [ESC] Back");
}

constexpr void render_input_setup(Framebuffer& fb, const State& s, const storage::RadioConfig& r) {
    widgets::draw_header(fb, "INPUT MODES");
    const std::size_t count = input_rows(r);
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.input_setup.scroll + slot;
        if (idx >= count) {
            break;
        }
        const std::size_t mode = std::min<std::size_t>(r.adc_modes[idx], kAdcModeCount - 1);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.input_setup.selected, kAuxInputNames[idx],
                               kAdcModeNames[mode], 0);
    }
    widgets::draw_scrollbar(fb, s.input_setup.selected, static_cast<int>(count), 14, 36);
    widgets::draw_footer(fb, "[OK] Cycle Mode    [ESC] Back");
}

constexpr void render_aux(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "AUX CHANNELS");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.aux.scroll + slot;
        if (idx >= kAuxChannelCount) {
            break;
        }
        // Rust pads single-digit channels so the colon lines up with two-digit ones.
        const std::size_t ch = idx + 5;
        format::Text<8> label;
        label.push('C');
        label.push('H');
        format::detail::push_uint(label, static_cast<unsigned>(ch));
        if (ch < 10) {
            label.push(' ');
        }
        label.push(':');
        const uint8_t src = m.aux_channels[idx];
        const std::size_t src_idx = std::min<std::size_t>(src, labels::kSourceNames.size() - 1);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.aux.selected, label.view(),
                               labels::kSourceNames[src_idx], 48);
    }
    if (s.editing) {
        widgets::draw_footer(fb, "[OK] Done   [UP/DN] Source");
    } else {
        widgets::draw_footer(fb, "[OK] Edit   [ESC] Exit");
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

// "+50%" with a forced sign, as the device shows Diff, Weight and Offset.
constexpr format::Text<6> signed_percent(int v) {
    format::Text<6> t;
    t.push(v < 0 ? '-' : '+');
    for (const char c : format::format_int(v < 0 ? -v : v).view()) {
        t.push(c);
    }
    t.push('%');
    return t;
}

template <std::size_t N>
constexpr std::string_view name_at(const std::array<std::string_view, N>& names, std::size_t idx) {
    return names[std::min(idx, N - 1)];
}

template <std::size_t N>
constexpr format::Text<N> text_of(std::string_view sv) {
    format::Text<N> t;
    for (const char c : sv) {
        t.push(c);
    }
    return t;
}

// "M01:" for mix line 0..31.
constexpr format::Text<4> mix_label(std::size_t idx) {
    const std::size_t num = idx + 1;
    format::Text<4> t;
    t.push('M');
    t.push(static_cast<char>('0' + num / 10));
    t.push(static_cast<char>('0' + num % 10));
    t.push(':');
    return t;
}

// "CH12" for a non-zero target channel.
constexpr format::Text<4> target_text(uint8_t ch) {
    format::Text<4> t;
    t.push('C');
    t.push('H');
    format::detail::push_uint(t, static_cast<unsigned>(ch));
    return t;
}

constexpr format::Text<11> mix_value(const storage::MixLine& mix, std::size_t idx) {
    switch (idx) {
        case 0:
            return mix.target_ch == 0 ? text_of<11>("Disabled") : text_of<11>(target_text(mix.target_ch).view());
        case 1: return text_of<11>(name_at(labels::kSourceNames, mix.source));
        case 2: return text_of<11>(name_at(labels::kWeightSrcNames, mix.weight_src));
        case 3: return text_of<11>(signed_percent(mix.weight).view());
        case 4: return text_of<11>(signed_percent(mix.offset).view());
        case 5: return text_of<11>(name_at(labels::kSwitchCondNames, mix.switch_sel));
        default: return text_of<11>(name_at(labels::kMixModeNames, mix.mode));
    }
}

constexpr void render_wing_mixer(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "WING & MIXER");
    const std::size_t gvars_row = wing_gvars_row(m);
    const std::size_t rows = wing_row_count(m);
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.wing.scroll + slot;
        if (idx >= rows) {
            break;
        }
        const bool sel = idx == s.wing.selected;
        const int row = static_cast<int>(slot);
        const int y = 14 + row * 9;
        if (idx == 0) {
            widgets::draw_list_row(fb, row, sel, "Wing:", labels::kTemplateNames[std::min<std::size_t>(m.wing_tail_mix, 3)], 36);
        } else if (wing_has_diff(m) && idx == 1) {
            const format::Text<6> diff = signed_percent(m.template_diff);
            widgets::draw_list_row(fb, row, sel, "Diff:", diff.view(), 36);
        } else if (idx == gvars_row) {
            widgets::draw_list_row(fb, row, sel, "GVars: [Edit GV1..5]", {}, 0);
        } else {
            const std::size_t mi = idx - (gvars_row + 1);
            const storage::MixLine& mix = m.mixes[mi];
            widgets::draw_list_row(fb, row, sel, mix_label(mi).view(), {}, 0);
            if (mix.target_ch == 0) {
                fb.draw_str_6x10(32, y, "[DISABLED]", sel);
            } else {
                fb.draw_str_6x10(30, y, target_text(mix.target_ch).view(), sel);
                fb.draw_str_6x10(58, y, "<-", sel);
                fb.draw_str_6x10(74, y, name_at(labels::kSourceNames, mix.source), sel);
            }
        }
    }
    widgets::draw_scrollbar(fb, static_cast<int>(s.wing.selected), static_cast<int>(rows), 14, 36);
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Diff" : "[OK] Select   [ESC] Back");
}

constexpr void render_mixer_line(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    const storage::MixLine& mix = m.mixes[s.mix_idx];
    const std::size_t num = static_cast<std::size_t>(s.mix_idx) + 1;
    format::Text<11> title = text_of<11>("EDIT MIX ");
    title.push(static_cast<char>('0' + num / 10));
    title.push(static_cast<char>('0' + num % 10));
    widgets::draw_header(fb, title.view());
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.mix_line.scroll + slot;
        if (idx >= kMixFields) {
            break;
        }
        const format::Text<11> value = mix_value(mix, idx);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.mix_line.selected, kMixLabels[idx], value.view(), 56);
    }
    widgets::draw_scrollbar(fb, static_cast<int>(s.mix_line.selected), static_cast<int>(kMixFields), 14, 36);
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Value" : "[OK] Edit   [ESC] Back");
}

constexpr void render_global_vars(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "GLOBAL VARIABLES");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.gvars.scroll + slot;
        if (idx >= m.gvars.size()) {
            break;
        }
        const format::Text<5> value = format::format_percent(m.gvars[idx]);
        widgets::draw_list_row(fb, static_cast<int>(slot), idx == s.gvars.selected, kGvarLabels[idx], value.view(), 56);
    }
    widgets::draw_scrollbar(fb, static_cast<int>(s.gvars.selected), static_cast<int>(m.gvars.size()), 14, 36);
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Value" : "[OK] Edit   [ESC] Back");
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
        case Screen::AuxChannels: render_aux(fb, s, st.active_model); break;
        case Screen::RadioSetup: render_radio_setup(fb, s, st.radio); break;
        case Screen::InputSetup: render_input_setup(fb, s, st.radio); break;
        case Screen::ChannelReverse: render_channel_reverse(fb, s, st.active_model); break;
        case Screen::ChannelMonitor: render_monitor(fb, s, live, st.active_model); break;
        case Screen::DiagAnalog: render_diag(fb, s, live); break;
        case Screen::SystemInfo: render_system(fb, live.mcu_name); break;
        case Screen::WingMixer: render_wing_mixer(fb, s, st.active_model); break;
        case Screen::MixerLine: render_mixer_line(fb, s, st.active_model); break;
        case Screen::GlobalVars: render_global_vars(fb, s, st.active_model); break;
        case Screen::Closed: break;
    }
}

}  // namespace ferrox::logic::menu
