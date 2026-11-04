#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include "wm/logic/auto_select.hpp"
#include "wm/logic/jog.hpp"
#include "wm/logic/buzzer.hpp"
#include "wm/logic/channels.hpp"
#include "wm/logic/crsf_config.hpp"
#include "wm/logic/crsf_menu.hpp"
#include "wm/logic/curve.hpp"
#include "wm/logic/format.hpp"
#include "wm/logic/framebuffer.hpp"
#include "wm/logic/glyphs.hpp"
#include "wm/logic/input.hpp"
#include "wm/logic/labels.hpp"
#include "wm/logic/model_copy.hpp"
#include "wm/logic/power.hpp"
#include "wm/logic/protocol.hpp"
#include "wm/logic/safety.hpp"
#include "wm/logic/storage.hpp"
#include "wm/logic/sysinfo.hpp"
#include "wm/logic/telemetry.hpp"
#include "wm/logic/widgets.hpp"

namespace wm::logic::menu {

using display::Framebuffer;
using display::Point;
using wm::logic::kKeyOk;
using safety::kKeyCancel;
using ModelName = std::array<uint8_t, 10>;
using ModelNames = std::array<ModelName, storage::kNumModels>;

inline constexpr uint16_t kKeyDown = 1u << 8;
inline constexpr uint16_t kKeyUp = 1u << 9;
inline constexpr uint16_t kNavMask = kKeyDown | kKeyUp | kKeyOk | kKeyCancel | kKeyBind;
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
enum class SetupRow : uint8_t {
    Name = 0, Type, ArmSwitch, Timer, Trigger, Copy, Vario, Telemetry, Protocol, Reset,
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    MultiSwitch,
#endif
};
inline constexpr std::size_t kSetupRows = board::kHasMultiSwitch ? 11 : 10;
inline constexpr std::size_t kItemVarioRow = static_cast<std::size_t>(SetupRow::Vario);
inline constexpr std::size_t kItemTelemRow = static_cast<std::size_t>(SetupRow::Telemetry);
inline constexpr uint8_t kNameLen = 10;
inline constexpr uint8_t kModelTypeCount = 5;
inline constexpr uint8_t kArmSwitchCount = board::kMaxSwitchCond + 1;
inline constexpr uint16_t kTimerStepSecs = 30;
inline constexpr uint16_t kTimerMaxSecs = 3600;
inline constexpr uint8_t kTimerSourceCount = 18;
// Vario switch codes: 0 OFF, 1 ON, 2.. arm conditions 1..kMaxSwitchCond.
inline constexpr uint8_t kVarioSwitchCount = board::kMaxSwitchCond + 2;
inline constexpr uint8_t kVarioClimbMaxTenths = 10;
inline constexpr uint8_t kVarioSinkMaxTenths = 20;
inline constexpr std::size_t kVarioRows = 5;
enum class VarioRow : uint8_t { Switch = 0, ClimbDeadband, SinkDeadband, SinkMode, MotorMute };

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
    ProtocolSetup,
    VarioSetup,
    TelemSetup,
    CrsfSetup,
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    MultiSwitchSetup,
#endif
};

inline constexpr std::size_t kItemDualRate = 2;
inline constexpr std::size_t kItemThrottleCurve = 3;
inline constexpr std::size_t kItemWingMixer = 4;
inline constexpr std::size_t kItemAuxChannels = 5;
inline constexpr std::size_t kItemChannelReverse = 6;
inline constexpr std::size_t kAuxChannelCount = 14;  // CH5..CH18
// Aux source IDs offered by the Rust AUX_SOURCES list (None, sticks, VRA/VRB, SA..SD, SE, SF, SWI, SWJ, VRC, VRD;
// the I6S adds SG, SH, VRE..VRG and SWK..SWM).
#if WM_PCB_I6S
inline constexpr std::array<uint8_t, 25> kAuxSources{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 31, 32,
                                                     33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44};
#else
inline constexpr std::array<uint8_t, 17> kAuxSources{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 31, 32, 35, 36, 37, 38};
#endif
inline constexpr std::size_t kItemChannelMonitor = 8;
inline constexpr std::size_t kItemCalibration = 9;
inline constexpr std::size_t kItemDiagAnalog = 10;
inline constexpr std::size_t kItemSystemInfo = 11;
inline constexpr std::size_t kItemRadioSetup = 7;
// Radio Setup row contents; the order per board is given by kBoardRadioItems.
enum class RadioItem : uint8_t {
    ThrTrim, Beeper, Tones, BlTimer, BlLevel, Contrast, BatWarn, UsbMode, ModulePwr, ExtSwitches, ExtAdc,
    StickMode, Gimbals, InputSetup, PpmOut, TrainerIn, J4Mode, RearLeft, RearRight, Touch, SwitchSound,
};
#if WM_PCB_I6S
inline constexpr std::array<RadioItem, 20> kBoardRadioItems{
    RadioItem::ThrTrim,   RadioItem::Beeper,   RadioItem::Tones,     RadioItem::BlTimer,     RadioItem::BlLevel,
    RadioItem::Contrast,  RadioItem::BatWarn,  RadioItem::UsbMode,   RadioItem::ModulePwr,   RadioItem::J4Mode,
    RadioItem::RearLeft,  RadioItem::RearRight, RadioItem::Touch,    RadioItem::StickMode,   RadioItem::Gimbals,
    RadioItem::SwitchSound, RadioItem::InputSetup, RadioItem::ExtAdc, RadioItem::PpmOut,     RadioItem::TrainerIn};
#else
inline constexpr std::array<RadioItem, 16> kBoardRadioItems{
    RadioItem::ThrTrim,   RadioItem::Beeper,  RadioItem::Tones,     RadioItem::BlTimer,    RadioItem::BlLevel,
    RadioItem::Contrast,  RadioItem::BatWarn, RadioItem::UsbMode,   RadioItem::ModulePwr,  RadioItem::ExtSwitches,
    RadioItem::ExtAdc,    RadioItem::StickMode, RadioItem::Gimbals, RadioItem::InputSetup, RadioItem::PpmOut,
    RadioItem::TrainerIn};
#endif
#if WM_USE_USB
inline constexpr bool kHasUsb = true;
#else
inline constexpr bool kHasUsb = false;
#endif

// Rows whose feature is compiled out (USB Mode without USE_USB) are not shown.
constexpr bool radio_item_present(const RadioItem item) {
    return kHasUsb || item != RadioItem::UsbMode;
}

inline constexpr auto kRadioItems = [] {
    constexpr auto n = static_cast<std::size_t>(std::ranges::count_if(kBoardRadioItems, radio_item_present));
    std::array<RadioItem, n> items{};
    std::ranges::copy_if(kBoardRadioItems, items.begin(), radio_item_present);
    return items;
}();
inline constexpr std::size_t kRadioRows = kRadioItems.size();

constexpr std::size_t radio_row_of(const RadioItem item) {
    for (std::size_t i = 0; i < kRadioItems.size(); ++i) {
        if (kRadioItems[i] == item) {
            return i;
        }
    }
    return 0;
}

inline constexpr std::size_t kRadioInputRow = radio_row_of(RadioItem::InputSetup);  // opens the Input Modes sub-screen
inline constexpr std::size_t kRadioInputMaxRows = board::kAuxCount;
inline constexpr std::size_t kAdcModeCount = board::kMaxAdcMode + 1;
inline constexpr uint8_t kTrainerPpmOut = 0x01;
inline constexpr uint8_t kTrainerPpmIn = 0x02;

constexpr std::string_view radio_label(const RadioItem item) {
    switch (item) {
        case RadioItem::ThrTrim: return "Thr Trim:";
        case RadioItem::Beeper: return "Beeper:";
        case RadioItem::Tones: return "Tones:";
        case RadioItem::BlTimer: return "BL Timer:";
        case RadioItem::BlLevel: return "BL Level:";
        case RadioItem::Contrast: return "Contrast:";
        case RadioItem::BatWarn: return "Bat Warn:";
        case RadioItem::UsbMode: return "USB Mode:";
        case RadioItem::ModulePwr: return board::kIsI6S ? "H1 PF6 Pwr:" : "PC13 Pwr:";
        case RadioItem::ExtSwitches: return "Ext Sw:";
        case RadioItem::ExtAdc: return board::kIsI6S ? "Ext ADC (PC):" : "P7 Analog:";
        case RadioItem::StickMode: return "Stick Mode:";
        case RadioItem::Gimbals: return "Gimbals:";
        case RadioItem::InputSetup: return board::kIsI6S ? "Inputs:" : "Input Setup";
        case RadioItem::PpmOut: return "PPM Out:";
        case RadioItem::TrainerIn: return "Trainer In:";
        case RadioItem::J4Mode: return "J4 Mode:";
        case RadioItem::RearLeft: return "Rear L:";
        case RadioItem::RearRight: return "Rear R:";
        case RadioItem::Touch: return "Touch:";
        case RadioItem::SwitchSound: return "Sw Sound:";
    }
    return "";
}

#if WM_PCB_I6S
inline constexpr std::array<std::string_view, kRadioInputMaxRows> kAuxInputNames{
    "SA (PA4):", "SB (PA5):", "SC (PB0):", "SD (PB1):", "VRA (PA6):", "VRB (PA7):",
    "VRC (PC1):", "VRD (PC2):", "VRE (PC3):", "VRF (PC4):", "VRG (PC5):"};
inline constexpr std::array<std::string_view, kAdcModeCount> kAdcModeNames{
    "DEFAULT", "OFF", "2-POS", "3-POS", "6-POS", "POT", "POT-D", "INST-TRIM"};
#else
inline constexpr std::array<std::string_view, kRadioInputMaxRows> kAuxInputNames{
    "SA (PA4):", "SB (PA5):", "SC (PB0):", "SD (PB1):", "VRA (PA6):",
    "VRB (PA7):", "VRC (PC2):", "VRD (PC3):", "VRE (PC4):", "VRF (PC5):"};
inline constexpr std::array<std::string_view, kAdcModeCount> kAdcModeNames{
    "DEFAULT", "2-POS", "3-POS", "6-POS", "POT", "POT-D", "INST-TRIM"};
#endif
inline constexpr std::size_t kMonitorPages = 3;
inline constexpr std::size_t kMonitorRows = 6;
inline constexpr std::size_t kDiagPages = 5;
inline constexpr std::size_t kAdcFullScale = 4095;

// Live values shown by the diagnostic screens. Filled by the caller each frame.
struct Live {
    std::array<uint16_t, kNumChannels> rf{};  // output channels in us
    input::Raw adc{};
    Switches switches{};  // digital switches, SE and SF on page 2
    storage::EepromStatus eeprom{};
    std::string_view mcu_name{};
    sysinfo::Build build{};
    const crsf::menu::Loader* crsf_loader{nullptr};
    const crsf::menu::Command* crsf_command{nullptr};
    const crsf::config::DeviceList* crsf_devices{nullptr};
};

enum class Action : uint8_t {
    None,
    CloseMenu,
    EnterModelSelect,  // caller refreshes ModelNames before the next render
    SwitchModel,       // index = target model slot
    SaveActiveModel,
    SaveRadioConfig,   // caller persists the radio config and applies LCD backlight and contrast
    StartCalibration,  // menu closes; caller starts the calibration wizard
    CopyActiveModel,   // index = destination slot; caller copies the active model there
    ResetActiveModel,  // active model already reset to defaults; caller reloads trims and applies the RF link
    CrsfRequest,       // caller performs State::crsf_req on the CRSF link
    StartBind,         // menu closes; caller starts AFHDS 2A binding
};

struct Event {
    Action action{Action::None};
    uint8_t index{0};
};

struct Input {
    uint16_t pressed;  // edge-detected keys
    uint16_t held;
    uint32_t now_ms;
    auto_select::Changes changes{};  // switch and pot movement since the previous tick
    const crsf::menu::Loader* crsf_loader{nullptr};
    const crsf::menu::Command* crsf_command{nullptr};
    const crsf::config::DeviceList* crsf_devices{nullptr};
    int16_t vr1{0};  // rear pots, used as jog wheels on boards with board::kHasJogWheels
    int16_t vr2{0};
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
    bool nav_tick{false};  // set on each list step this tick, for the navigation click
    buzzer::Cue cue{buzzer::Cue::None};  // value-edit tone for this tick
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
    ListPos vario{};
    ListPos telem{};
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    ListPos multiswitch{};
#endif
    ListPos protocol{};
    crsf::config::State crsf{};
    crsf::config::Request crsf_req{};  // pending request for Action::CrsfRequest
    uint8_t axis{0};  // dual rate axis: 0 Roll, 1 Pitch, 2 Yaw
    uint8_t page{0};  // page of the channel monitor or analog diagnostics
    bool editing{false};
    uint8_t name_cursor{0};
    model_copy::Picker copy{};  // destination chosen on the Copy row
    Repeat up{};
    Repeat down{};
    jog::Wheel jog{};
};

struct Nav {
    bool up{false};
    bool down{false};
    bool ok{false};
    bool cancel{false};
    bool bind{false};  // BIND key: "next" in Model Setup, Vario Setup, name edit and Throttle Curve (Rust)
    auto_select::Changes changes{};
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
constexpr char next_ascii(const char c) {
    if (c == ' ') return 'A';
    if (c >= 'A' && c <= 'Y') return static_cast<char>(c + 1);
    if (c == 'Z') return '0';
    if (c >= '0' && c <= '8') return static_cast<char>(c + 1);
    if (c == '9') return '-';
    if (c == '-') return '_';
    return ' ';
}

constexpr char prev_ascii(const char c) {
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
constexpr bool step_list(ListPos& p, const std::size_t count, const Rows rows, const Dir dir) {
    const std::size_t visible = rows.value;
    if (count == 0) {
        return false;
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
    return true;
}

// Reports a step on press, then again after the repeat delay at the repeat interval.
constexpr bool repeat_step(Repeat& r, const KeyState key, const uint32_t now) {
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

struct ItemIndex {
    std::size_t value;
    friend constexpr bool operator==(const ItemIndex&, const ItemIndex&) = default;
};

constexpr uint8_t window_scroll(const ItemIndex selected, const Rows visible) {
    return static_cast<uint8_t>(selected.value >= visible.value ? selected.value - visible.value + 1 : 0);
}

constexpr Event update_main(State& s, const Nav& n, const uint8_t active_model) {
    if (n.up != n.down) {
        s.nav_tick = step_list(s.main, kMainItems.size(), Rows{kMainVisible}, nav_dir(n));
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
            // Active model in the third row where possible (Rust: sel - 2, at most count - 4).
            s.models.scroll = static_cast<uint8_t>(
                std::min<std::size_t>(active_model > 2 ? active_model - 2u : 0u, storage::kNumModels - kListVisible));
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
        s.nav_tick = step_list(s.models, storage::kNumModels, Rows{kListVisible}, nav_dir(n));
    }
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {};
    }
    if (n.ok) {
        // Rust stays in the list (the new model gets the [*] mark) and waits for the keys to be released.
        s.waiting_release = true;
        return {Action::SwitchModel, s.models.selected};
    }
    return {};
}

// Mixer source IDs of the two pots (VRA, VRB).
inline constexpr uint8_t kSourceVra = 5;
inline constexpr uint8_t kSourceVrb = 6;

// Pot that moved this tick, as a mixer source ID. VR1 wins when both moved.
constexpr std::optional<uint8_t> pot_change_source(const auto_select::Changes& c) {
    if (c.vr1) {
        return kSourceVra;
    }
    if (c.vr2) {
        return kSourceVrb;
    }
    return std::nullopt;
}

constexpr bool is_switch_cond_available(const uint8_t cond, const storage::RadioConfig& r);

// Next (down = false) or previous (down = true) code that passes `ok`, wrapping over 0..count-1. Returns cur if none does.
template <typename Ok>
constexpr uint8_t step_available(const uint8_t cur, const std::size_t count, const bool down, Ok ok) {
    for (std::size_t i = 1; i < count; ++i) {
        const auto cand = static_cast<uint8_t>(down ? (cur + count - i) % count : (cur + i) % count);
        if (ok(cand)) {
            return cand;
        }
    }
    return cur;
}

// Arm switch codes are condition codes 1..22 (0 is unused by auto-select).
constexpr bool is_arm_switch_available(const uint8_t sw, const storage::RadioConfig& r) {
    return sw < kArmSwitchCount && is_switch_cond_available(sw, r);
}

// Timer trigger codes 0..3 always exist; 4..17 need switch condition 1..14.
constexpr bool is_timer_source_available(const uint8_t ts, const storage::RadioConfig& r) {
    if (ts <= 3) {
        return true;
    }
    return ts < kTimerSourceCount && is_switch_cond_available(static_cast<uint8_t>(ts - 3), r);
}

constexpr Event update_setup(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r,
                             const model_copy::SrcSlot active) {
    const auto row = static_cast<SetupRow>(s.setup.selected);
    const Event save{Action::SaveActiveModel, 0};

    if (!s.editing) {
        if (const bool down = n.down || n.bind; n.up != down) {
            s.nav_tick = step_list(s.setup, kSetupRows, Rows{kListVisible}, down ? Dir::Down : Dir::Up);
        }
        if (n.cancel) {
            s.screen = Screen::Main;
            s.cue = buzzer::Cue::Click;
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
                case SetupRow::Trigger:
                    s.editing = true;
                    break;
                case SetupRow::Copy:
                    s.editing = true;
                    s.copy = model_copy::open(active);
                    break;
                case SetupRow::Vario:
                    s.screen = Screen::VarioSetup;
                    s.vario = ListPos{};
                    s.editing = false;
                    break;
                case SetupRow::Telemetry:
                    s.screen = Screen::TelemSetup;
                    s.telem = ListPos{};
                    s.editing = false;
                    break;
                case SetupRow::Protocol:
                    s.screen = Screen::ProtocolSetup;
                    s.protocol = ListPos{};
                    s.editing = false;
                    break;
#if WM_PCB_I6S && WM_USE_MULTISWITCH
                case SetupRow::MultiSwitch:
                    s.screen = Screen::MultiSwitchSetup;
                    s.multiswitch = ListPos{};
                    s.editing = false;
                    break;
#endif
                case SetupRow::Reset:
                    m = protocol::reset_model(protocol::ModelSlot{active.value});
                    return {Action::ResetActiveModel, 0};
            }
        }
        return {};
    }

    switch (row) {
        case SetupRow::Vario:
        case SetupRow::Telemetry:
        case SetupRow::Protocol:
        case SetupRow::Reset:
#if WM_PCB_I6S && WM_USE_MULTISWITCH
        case SetupRow::MultiSwitch:
#endif
            break;
        case SetupRow::Name:
            if (n.cancel) {
                s.name_cursor = 0;
                s.editing = false;
                return save;
            }
            if (n.bind) {
                s.name_cursor = static_cast<uint8_t>((s.name_cursor + 1) % kNameLen);
                s.cue = buzzer::Cue::Click;
            } else if (n.ok) {
                if (s.name_cursor + 1u < kNameLen) {
                    ++s.name_cursor;
                } else {
                    // Done after the last character: continue on the next row (Rust).
                    s.name_cursor = 0;
                    s.editing = false;
                    s.setup.selected = static_cast<uint8_t>(SetupRow::Type);
                    return save;
                }
            } else if (n.up) {
                auto& c = m.name[s.name_cursor];
                c = static_cast<uint8_t>(next_ascii(static_cast<char>(c)));
            } else if (n.down) {
                auto& c = m.name[s.name_cursor];
                c = static_cast<uint8_t>(prev_ascii(static_cast<char>(c)));
            }
            break;
        case SetupRow::ArmSwitch:
            if (n.changes.sw_cond) {
                if (is_arm_switch_available(*n.changes.sw_cond, r)) {
                    m.arm_switch = *n.changes.sw_cond;
                }
            } else if (n.up != n.down) {
                // Skip switches that are not configured (Rust next/prev_available_arm_switch).
                m.arm_switch = step_available(m.arm_switch, kArmSwitchCount, n.down,
                                              [&](const uint8_t sw) { return is_arm_switch_available(sw, r); });
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
        case SetupRow::Trigger:
            if (n.changes.sw_cond) {
                const auto ts = static_cast<uint8_t>(*n.changes.sw_cond + 3);
                if (is_timer_source_available(ts, r)) {
                    m.timer_source = ts;
                }
            } else if (n.up != n.down) {
                m.timer_source = step_available(m.timer_source, kTimerSourceCount, n.down,
                                                [&](const uint8_t ts) { return is_timer_source_available(ts, r); });
            }
            if (n.ok || n.cancel) {
                s.editing = false;
                return save;
            }
            break;
        case SetupRow::Type:
            break;
        case SetupRow::Copy:
            switch (model_copy::update(s.copy, model_copy::Keys{n.up, n.down, n.ok, n.cancel})) {
                case model_copy::Outcome::Choosing: break;
                case model_copy::Outcome::Cancelled:
                    s.editing = false;
                    break;
                case model_copy::Outcome::Copy:
                    s.editing = false;
                    return {Action::CopyActiveModel, static_cast<uint8_t>(s.copy.dst.value)};
            }
            break;
    }
    return {};
}

inline constexpr std::size_t kProtocolRowsAfhds = 6;
inline constexpr std::size_t kProtocolRowsCrsf = 5;
inline constexpr std::size_t kCrsfBaudCount = 5;
enum class CrsfRow : uint8_t { Proto = 0, Baud, Duplex, ArmMode, Configure };
enum class AfhdsRow : uint8_t { Proto = 0, Bind, Power, ServoHz, RxOut, Serial };
inline constexpr std::size_t kRfPowerCount = 4;
inline constexpr std::array<std::string_view, kRfPowerCount> kRfPowerNames{"100mW (HI)", "10mW (MID)", "1mW (LOW)",
                                                                           "100uW (TST)"};

// Servo rate steps of the RX output; unknown values restart at 50 Hz.
constexpr uint16_t next_servo_hz(const uint16_t hz) {
    switch (hz) {
        case 50: return 60;
        case 60: return 100;
        case 100: return 150;
        case 150: return 200;
        case 200: return 250;
        case 250: return 300;
        case 300: return 350;
        case 350: return 400;
        default: return 50;
    }
}

// Protocol sub-screen. Proto and Baud are edited (UP/DN, OK saves); Duplex and Arm Mode toggle on OK;
// Configure Module opens the device parameter browser. As in Rust, ESC while editing keeps the changed value
// in RAM without saving, and ESC otherwise returns to Model Setup.
constexpr Event update_protocol(State& s, const Nav& n, storage::ModelConfig& m) {
    const bool crsf_active = protocol::from_raw(m.rf_protocol) == protocol::Protocol::Crsf;
    const std::size_t rows = crsf_active ? kProtocolRowsCrsf : kProtocolRowsAfhds;
    if (s.protocol.selected >= rows) {
        s.protocol = ListPos{};
    }
    const auto row = static_cast<CrsfRow>(s.protocol.selected);

    if (s.editing) {
        if (n.up != n.down) {
            if (row == CrsfRow::Proto) {
                m.rf_protocol = protocol::to_raw(protocol::toggle(protocol::from_raw(m.rf_protocol)));
            } else {
                const uint8_t cur = m.crsf_baud < kCrsfBaudCount ? m.crsf_baud : 0;
                m.crsf_baud = n.up ? static_cast<uint8_t>((cur + 1) % kCrsfBaudCount)
                                   : static_cast<uint8_t>(cur == 0 ? kCrsfBaudCount - 1 : cur - 1);
                s.cue = buzzer::Cue::ValueStep;
            }
        }
        if (n.ok) {
            s.editing = false;
            return {Action::SaveActiveModel, 0};
        }
        if (n.cancel) {
            s.editing = false;
        }
        return {};
    }

    if (n.up != n.down) {
        s.nav_tick = step_list(s.protocol, rows, Rows{kListVisible}, nav_dir(n));
    }
    if (n.cancel) {
        s.screen = Screen::ModelSetup;
        s.cue = buzzer::Cue::Click;
        return {};
    }
    if (!n.ok) {
        return {};
    }
    if (row == CrsfRow::Proto) {
        s.editing = true;
        return {};
    }
    if (!crsf_active) {
        switch (static_cast<AfhdsRow>(s.protocol.selected)) {
            case AfhdsRow::Bind:
                s.screen = Screen::Closed;
                s.cue = buzzer::Cue::Click;
                return {Action::StartBind, 0};
            case AfhdsRow::Power: m.rf_power = static_cast<uint8_t>((m.rf_power + 1) % kRfPowerCount); break;
            case AfhdsRow::ServoHz: m.servo_rate_hz = next_servo_hz(m.servo_rate_hz); break;
            case AfhdsRow::RxOut: m.rx_out_mode = m.rx_out_mode != 0 ? 0 : 1; break;
            case AfhdsRow::Serial: m.rx_serial_proto = m.rx_serial_proto != 0 ? 0 : 1; break;
            case AfhdsRow::Proto: return {};
        }
        s.cue = buzzer::Cue::Click;
        return {Action::SaveActiveModel, 0};
    }
    switch (row) {
        case CrsfRow::Baud:
            s.editing = true;
            break;
        case CrsfRow::Duplex:
            m.crsf_half_duplex = m.crsf_half_duplex != 0 ? 0 : 1;
            return {Action::SaveActiveModel, 0};
        case CrsfRow::ArmMode:
            m.crsf_arm_mode = m.crsf_arm_mode != 0 ? 0 : 1;
            return {Action::SaveActiveModel, 0};
        case CrsfRow::Configure:
            s.screen = Screen::CrsfSetup;
            s.crsf = crsf::config::State{};
            s.crsf_req = {crsf::config::Request::Kind::Restart, 0, 0};
            s.waiting_release = true;
            s.cue = buzzer::Cue::Click;
            return {Action::CrsfRequest, 0};
        case CrsfRow::Proto: break;
    }
    return {};
}

// Configure Module screen. The caller performs `s.crsf_req` when the event asks for it.
constexpr Event update_crsf_setup(State& s, const Nav& n, const Input& in) {
    const auto r = crsf::config::update(s.crsf, {n.up, n.down, n.ok, n.cancel}, in.crsf_loader, in.crsf_command,
                                     in.crsf_devices);
    s.nav_tick = r.nav_tick;
    if (r.click) {
        s.cue = buzzer::Cue::Click;
    }
    if (r.leave) {
        s.screen = Screen::ProtocolSetup;
        return {};
    }
    if (r.req.kind == crsf::config::Request::Kind::None) {
        return {};
    }
    s.crsf_req = r.req;
    return {Action::CrsfRequest, 0};
}

// Advances the menu state for one tick. `st` is edited in place; the caller persists it when told to.
inline constexpr std::size_t kDrFields = 6;
inline constexpr std::array<std::string_view, kDrFields> kDrLabels{
    "Switch:", "Channel:", "Hi Rate:", "Lo Rate:", "Hi Expo:", "Lo Expo:",
};

// SE/SF exist with the I6X external switch board, or on the I6S when the rear button acts as RC switch
// (flysky-i6s-rs still checks ext_switches, which has no setup row there, so SE/SF could never be picked).
constexpr bool has_switch_se(const storage::RadioConfig& r) {
    return board::kIsI6S ? r.rear_left_func == 0 : r.ext_switches != 0;
}

constexpr bool has_switch_sf(const storage::RadioConfig& r) {
    return board::kIsI6S ? r.rear_right_func == 0 : r.ext_switches != 0;
}

// Mirrors the firmware rules: SA..SD need a 2/3-position or instant-trim input, SE/SF need the external switch board.
constexpr bool is_dr_switch_available(const uint8_t sw, const storage::RadioConfig& r) {
    if (sw == 0) {
        return true;
    }
    if (sw <= 4) {
        const input::AdcMode mode = input::resolve_mode(sw - 1u, r.adc_modes[sw - 1u]);
        return mode == input::AdcMode::TwoPos || mode == input::AdcMode::ThreePos ||
               mode == input::AdcMode::InstantTrim;
    }
    if (sw == 5) {
        return has_switch_se(r);
    }
    if (sw == 6) {
        return has_switch_sf(r);
    }
    return false;
}

// Next (down = false) or previous (down = true) available switch code, wrapping over 0..6.
constexpr uint8_t step_dr_switch(const storage::RadioConfig& r, const uint8_t cur, const bool down) {
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
// Returns true only when the value changed.
template <typename T>
constexpr bool step_clamped(T& v, const Range range, const int step, const Nav& n) {
    const int cur = v;
    if (n.up && cur + step <= range.max) {
        v = static_cast<T>(cur + step);
        return true;
    }
    if (n.down && cur - step >= range.min) {
        v = static_cast<T>(cur - step);
        return true;
    }
    return false;
}

// Value-edit tone for a clamped step, played only when the value moved.
template <typename T>
constexpr void step_clamped_tone(State& s, T& v, const Range range, const int step, const Nav& n) {
    if (step_clamped(v, range, step, n)) {
        s.cue = buzzer::Cue::ValueStep;
    }
}

#if WM_PCB_I6S && WM_USE_MULTISWITCH
constexpr Event update_multiswitch(State& s, const Nav& n, storage::ModelConfig& m) {
    if (s.editing) {
        if (n.up != n.down) {
            step_clamped_tone(s, m.multiswitch_addresses[s.multiswitch.selected], Range{-1, 255}, 1, n);
        }
        if (n.ok || n.cancel) {
            s.editing = false;
            return {Action::SaveActiveModel, 0};
        }
        return {};
    }
    if (n.cancel) {
        s.screen = Screen::ModelSetup;
        s.cue = buzzer::Cue::Click;
        s.setup.selected = static_cast<uint8_t>(SetupRow::MultiSwitch);
        s.setup.scroll = window_scroll({s.setup.selected}, {kListVisible});
        return {};
    }
    if (const bool down = n.down || n.bind; n.up != down) {
        s.nav_tick = step_list(s.multiswitch, m.multiswitch_addresses.size(), Rows{kListVisible},
                               down ? Dir::Down : Dir::Up);
    }
    if (n.ok) {
        s.editing = true;
    }
    return {};
}
#endif

constexpr Event update_dual_rate(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r) {
    if (n.cancel) {
        if (s.editing) {
            s.editing = false;
            return {};
        }
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {Action::SaveActiveModel, 0};
    }
    if (!s.editing) {
        if (n.up != n.down) {
            s.nav_tick = step_list(s.dual_rate, kDrFields, Rows{kListVisible}, nav_dir(n));
        }
        if (n.ok) {
            s.editing = true;
        }
        return {};
    }

    const std::size_t axis = s.axis;
    switch (s.dual_rate.selected) {
        case 0:
            if (n.changes.dr_switch) {
                if (is_dr_switch_available(*n.changes.dr_switch, r)) {
                    m.dr_switch = *n.changes.dr_switch;
                    s.cue = buzzer::Cue::ValueSelect;
                }
            } else if (n.up) {
                m.dr_switch = step_dr_switch(r, m.dr_switch, false);
                s.cue = buzzer::Cue::ValueStep;
            } else if (n.down) {
                m.dr_switch = step_dr_switch(r, m.dr_switch, true);
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 1:
            if (n.up || n.down) {
                s.axis = static_cast<uint8_t>((s.axis + 1) % 3);
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 2: step_clamped_tone(s, m.dr_high[axis], Range{50, 100}, 5, n); break;
        case 3: step_clamped_tone(s, m.dr_low[axis], Range{30, 100}, 5, n); break;
        case 4: step_clamped_tone(s, m.expo_high[axis], Range{-100, 100}, 5, n); break;
        case 5: step_clamped_tone(s, m.expo_low[axis], Range{-100, 100}, 5, n); break;
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
        if (n.up || n.down) {
            s.cue = buzzer::Cue::ValueStep;
        }
        if (n.ok) {
            if (sel + 1u < items) {
                s.throttle_curve.selected = static_cast<uint8_t>(sel + 1);
            } else {
                s.editing = false;
            }
            return {Action::SaveActiveModel, 0};
        }
        if (n.bind) {
            // Next point, wrapping back to P1, without leaving edit mode (Rust).
            s.throttle_curve.selected = static_cast<uint8_t>(sel + 1u < items ? sel + 1 : 2);
            s.cue = buzzer::Cue::Click;
            return {Action::SaveActiveModel, 0};
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {Action::SaveActiveModel, 0};
    }
    if (n.up != n.down) {
        const std::size_t next = n.down ? (sel + 1u) % items : (sel == 0 ? items - 1 : sel - 1u);
        s.throttle_curve.selected = static_cast<uint8_t>(next);
        s.cue = buzzer::Cue::ValueStep;
    }
    if (n.bind) {
        // Jump to the curve points: P1 from the mode rows, otherwise the next point, wrapping to P1 (Rust).
        const uint8_t cur = s.throttle_curve.selected;
        s.throttle_curve.selected = static_cast<uint8_t>(cur < 2 || cur + 1u >= items ? 2 : cur + 1);
        s.cue = buzzer::Cue::Click;
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

constexpr input::AdcMode adc_mode_at(const storage::RadioConfig& r, const std::size_t idx) {
    return input::resolve_mode(idx, r.adc_modes[idx]);
}

constexpr bool is_two_or_three_pos(const input::AdcMode mode) {
    return mode == input::AdcMode::TwoPos || mode == input::AdcMode::ThreePos;
}

constexpr bool is_switch_input(const input::AdcMode mode) {
    return is_two_or_three_pos(mode) || mode == input::AdcMode::InstantTrim;
}

constexpr bool is_pot_input(const input::AdcMode mode) {
    return mode == input::AdcMode::Pot || mode == input::AdcMode::PotDetent;
}

// Channels 1..18 that drive an output: primary controls, flaperon CH6, assigned aux channels or mix targets.
constexpr bool is_channel_configured(const uint8_t ch, const storage::ModelConfig& m) {
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

constexpr bool is_source_available(const uint8_t src, const storage::RadioConfig& r, const storage::ModelConfig& m) {
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
    if (src == 31) {
        return has_switch_se(r);
    }
    if (src == 32) {
        return has_switch_sf(r);
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
    if constexpr (board::kIsI6S) {
        if (src == 41) {
            return ext_adc && is_pot_input(adc_mode_at(r, 10));  // VRG
        }
        if (src <= 44) {
            return ext_adc && is_two_or_three_pos(adc_mode_at(r, src - 34u));  // SWK..SWM
        }
    }
    return src < board::kSourceCount;  // GV1..GV5
}

constexpr bool is_weight_src_available(const uint8_t ws, const storage::ModelConfig& m) {
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

constexpr bool is_switch_cond_available(const uint8_t cond, const storage::RadioConfig& r) {
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
        case 12: return has_switch_se(r);
        case 13:
        case 14: return has_switch_sf(r);
        case 15:
        case 16:
        case 17:
        case 18: return r.j4_sg_sh_en != 0;
        case 19:
        case 20: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 6));
        case 21:
        case 22: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 7));
#if WM_PCB_I6S
        case 23:
        case 24: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 8));
        case 25:
        case 26: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 9));
        case 27:
        case 28: return r.ext_adc != 0 && is_two_or_three_pos(adc_mode_at(r, 10));
#endif
        default: return false;
    }
}

// Vario switch codes that can be picked: OFF and ON always, others when their arm condition exists.
constexpr bool is_vario_switch_available(const uint8_t sw, const storage::RadioConfig& r) {
    if (sw <= 1) {
        return true;
    }
    return sw < kVarioSwitchCount && is_switch_cond_available(static_cast<uint8_t>(sw - 1), r);
}

// Deadband cycles as in Rust: climb 0.1..1.0 m/s, sink steps by 2 tenths and can reach 2.1 from an odd start.
constexpr uint8_t next_climb_deadband(const uint8_t tenths) {
    return tenths >= kVarioClimbMaxTenths ? 1 : static_cast<uint8_t>(tenths + 1);
}

constexpr uint8_t next_sink_deadband(const uint8_t tenths) {
    return tenths >= kVarioSinkMaxTenths ? 2 : static_cast<uint8_t>(tenths + 2);
}

// Telemetry Setup sub-screen (Rust `TelemSetup`). Each OK cycles one slot's sensor and saves at once.
constexpr Event update_telem(State& s, const Nav& n, storage::ModelConfig& m) {
    if (n.cancel) {
        s.screen = Screen::ModelSetup;
        s.cue = buzzer::Cue::Click;
        s.setup.selected = kItemTelemRow;
        s.setup.scroll = window_scroll(ItemIndex{kItemTelemRow}, Rows{kListVisible});
        return {};
    }
    if (n.up != n.down) {
        s.nav_tick = step_list(s.telem, telemetry::kTelemSlotCount, Rows{kListVisible}, nav_dir(n));
    }
    if (n.ok) {
        auto& sensor = m.telem_slots[s.telem.selected];
        sensor = telemetry::next_telem_sensor(sensor);
        return {Action::SaveActiveModel, 0};
    }
    return {};
}

// Vario Setup sub-screen. Each change is saved at once, as Rust does. The switch row edits like the arm switch.
constexpr Event update_vario(State& s, const Nav& n, storage::ModelConfig& m, const storage::RadioConfig& r) {
    const Event save{Action::SaveActiveModel, 0};

    if (!s.editing) {
        if (n.cancel) {
            s.screen = Screen::ModelSetup;
            s.cue = buzzer::Cue::Click;
            s.setup.selected = kItemVarioRow;
            s.setup.scroll = window_scroll(ItemIndex{kItemVarioRow}, Rows{kListVisible});
            return {};
        }
        if (const bool down = n.down || n.bind; n.up != down) {
            s.nav_tick = step_list(s.vario, kVarioRows, Rows{kListVisible}, down ? Dir::Down : Dir::Up);
        }
        if (n.ok) {
            switch (static_cast<VarioRow>(s.vario.selected)) {
                case VarioRow::Switch:
                    s.editing = true;
                    break;
                case VarioRow::ClimbDeadband:
                    m.vario_climb_deadband = next_climb_deadband(m.vario_climb_deadband);
                    return save;
                case VarioRow::SinkDeadband:
                    m.vario_sink_deadband = next_sink_deadband(m.vario_sink_deadband);
                    return save;
                case VarioRow::SinkMode:
                    m.vario_sink_mode = static_cast<uint8_t>((m.vario_sink_mode + 1) % 3);
                    return save;
                case VarioRow::MotorMute:
                    m.vario_motor_mute = m.vario_motor_mute == 0 ? 1 : 0;
                    return save;
            }
        }
        return {};
    }

    if (n.changes.sw_cond) {
        // Vario switch codes are condition codes shifted by one; 0 means no switch.
        const auto sw = static_cast<uint8_t>(*n.changes.sw_cond + 1);
        if (is_vario_switch_available(sw, r)) {
            m.vario_switch = sw;
        }
    } else if (n.up != n.down) {
        m.vario_switch = step_available(m.vario_switch, kVarioSwitchCount, n.down,
                                        [&r](const uint8_t c) { return is_vario_switch_available(c, r); });
    }
    if (n.ok || n.cancel) {
        s.editing = false;
        return save;
    }
    return {};
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
        step_clamped_tone(s, m.template_diff, Range{-100, 100}, 5, n);
        if (n.ok) {
            s.editing = false;
            return save;
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return save;
    }
    if (n.up != n.down) {
        s.nav_tick = step_list(s.wing, wing_row_count(m), Rows{kListVisible}, nav_dir(n));
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
            s.wing.scroll = window_scroll({row}, {kListVisible});
            s.cue = buzzer::Cue::Click;
            return {};
        }
        if (n.up != n.down) {
            s.nav_tick = step_list(s.mix_line, kMixFields, Rows{kListVisible}, nav_dir(n));
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
                s.cue = buzzer::Cue::ValueStep;
            } else if (down) {
                mix.target_ch = static_cast<uint8_t>(mix.target_ch == 0 ? 18 : mix.target_ch - 1);
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 1:
            if (const auto pot = pot_change_source(n.changes)) {
                if (is_source_available(*pot, r, m)) {
                    mix.source = *pot;
                    s.cue = buzzer::Cue::ValueSelect;
                }
            } else if (up || down) {
                mix.source = step_available(mix.source, labels::kSourceNames.size(), down,
                                            [&](const uint8_t c) { return is_source_available(c, r, m); });
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 2:
            if (up || down) {
                mix.weight_src = step_available(mix.weight_src, labels::kWeightSrcNames.size(), down,
                                                [&](const uint8_t c) { return is_weight_src_available(c, m); });
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 3: step_clamped_tone(s, mix.weight, Range{-100, 100}, 5, n); break;
        case 4: step_clamped_tone(s, mix.offset, Range{-100, 100}, 5, n); break;
        case 5:
            if (n.changes.sw_cond) {
                if (is_switch_cond_available(*n.changes.sw_cond, r)) {
                    mix.switch_sel = *n.changes.sw_cond;
                    s.cue = buzzer::Cue::ValueSelect;
                }
            } else if (up || down) {
                mix.switch_sel = step_available(mix.switch_sel, labels::kSwitchCondNames.size(), down,
                                                [&](const uint8_t c) { return is_switch_cond_available(c, r); });
                s.cue = buzzer::Cue::ValueStep;
            }
            break;
        case 6:
            if (up) {
                mix.mode = static_cast<uint8_t>((mix.mode + 1) % labels::kMixModeNames.size());
                s.cue = buzzer::Cue::ValueStep;
            } else if (down) {
                mix.mode = static_cast<uint8_t>(mix.mode == 0 ? labels::kMixModeNames.size() - 1 : mix.mode - 1);
                s.cue = buzzer::Cue::ValueStep;
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
        step_clamped_tone(s, m.gvars[s.gvars.selected], Range{-1000, 1000}, 50, n);
        if (n.ok) {
            s.editing = false;
            return save;
        }
        return {};
    }

    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return save;
    }
    if (n.up != n.down) {
        s.nav_tick = step_list(s.gvars, m.gvars.size(), Rows{kListVisible}, nav_dir(n));
    }
    if (n.ok) {
        s.editing = true;
    }
    return {};
}

// Next (down = false) or previous (down = true) usable aux source, wrapping like Rust next/prev_available_aux_source.
// An unknown current source is treated as index 0.
template <typename Ok>
constexpr uint8_t step_aux_source(const uint8_t cur, const bool down, Ok ok) {
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
        s.cue = buzzer::Cue::Click;
        return {Action::SaveActiveModel, 0};
    }
    if (!s.editing) {
        if (n.up != n.down) {
            s.nav_tick = step_list(s.aux, kAuxChannelCount, Rows{kListVisible}, nav_dir(n));
        }
        if (n.ok) {
            s.editing = true;
        }
        return {};
    }
    if (const auto pot = pot_change_source(n.changes)) {
        m.aux_channels[s.aux.selected] = *pot;
    } else if (n.up != n.down) {
        auto& src = m.aux_channels[s.aux.selected];
        src = step_aux_source(src, n.down,         [&](const uint8_t c) { return is_source_available(c, r, m); });
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
constexpr void cycle_radio_row(storage::RadioConfig& r, const std::size_t row) {
    if (row >= kRadioRows) {
        return;
    }
    switch (kRadioItems[row]) {
        case RadioItem::ThrTrim: r.throttle_trim = static_cast<uint8_t>((r.throttle_trim + 1) % 3); break;
        case RadioItem::Beeper: toggle_flag(r.audio_enabled); break;
        case RadioItem::Tones: toggle_flag(r.tone_style); break;
        case RadioItem::BlTimer: r.backlight_timeout = static_cast<uint8_t>((r.backlight_timeout + 1) % 4); break;
        case RadioItem::BlLevel: r.backlight_brightness = r.backlight_brightness >= 10 ? 1 : static_cast<uint8_t>(r.backlight_brightness + 1); break;
        case RadioItem::Contrast: r.lcd_contrast = r.lcd_contrast >= 50 ? 20 : static_cast<uint8_t>(std::min<unsigned>(r.lcd_contrast + 3u, 50u)); break;
        case RadioItem::BatWarn: r.vbat_warn_deci = r.vbat_warn_deci >= 50 ? 40 : static_cast<uint8_t>(r.vbat_warn_deci + 1); break;
        case RadioItem::UsbMode: r.usb_mode = static_cast<uint8_t>((r.usb_mode + 1) % 4); break;
        case RadioItem::ModulePwr: toggle_flag(r.ext_module_pwr); break;
        case RadioItem::ExtSwitches: toggle_flag(r.ext_switches); break;
        case RadioItem::ExtAdc: toggle_flag(r.ext_adc); break;
        case RadioItem::StickMode: r.stick_mode = static_cast<uint8_t>((r.stick_mode + 1) % 4); break;
        case RadioItem::Gimbals: toggle_flag(r.gimbal_type); break;
        case RadioItem::PpmOut: r.trainer_config ^= kTrainerPpmOut; break;
        case RadioItem::TrainerIn: r.trainer_config ^= kTrainerPpmIn; break;
        case RadioItem::J4Mode: toggle_flag(r.j4_sg_sh_en); break;
        case RadioItem::RearLeft: r.rear_left_func = static_cast<uint8_t>((r.rear_left_func + 1) % 3); break;
        case RadioItem::RearRight: r.rear_right_func = static_cast<uint8_t>((r.rear_right_func + 1) % 3); break;
        case RadioItem::Touch: toggle_flag(r.touch_disabled); break;
        case RadioItem::SwitchSound: toggle_flag(r.switch_beep); break;
        case RadioItem::InputSetup: break;
    }
}

constexpr std::size_t input_rows(const storage::RadioConfig& r) {
    return r.ext_adc != 0 ? kRadioInputMaxRows : 6;
}

constexpr Event update_radio_setup(State& s, const Nav& n, storage::RadioConfig& r) {
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {};
    }
    if (n.up != n.down) {
        s.nav_tick = step_list(s.radio, kRadioRows, Rows{kListVisible}, nav_dir(n));
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
        s.cue = buzzer::Cue::Click;
        s.radio = ListPos{kRadioInputRow, kRadioInputRow - 3};
        return {};
    }
    const std::size_t count = input_rows(r);
    if (n.up != n.down) {
        s.nav_tick = step_list(s.input_setup, count, Rows{kListVisible}, nav_dir(n));
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
        s.nav_tick = step_list(s.reverse, kNumChannels, Rows{kListVisible}, nav_dir(n));
    }
    if (n.ok) {
        m.channel_reverse ^= 1u << s.reverse.selected;
    }
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {Action::SaveActiveModel, 0};
    }
    return {};
}

constexpr Event update_monitor(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {};
    }
    // UP turns forward and DOWN back, as in Rust `update_channel_monitor`.
    if (n.up) {
        s.page = static_cast<uint8_t>((s.page + 1) % kMonitorPages);
        s.nav_tick = true;
    } else if (n.down) {
        s.page = static_cast<uint8_t>((s.page + kMonitorPages - 1) % kMonitorPages);
        s.nav_tick = true;
    }
    return {};
}

constexpr Event update_diag(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
        return {};
    }
    if (n.up) {
        s.page = static_cast<uint8_t>((s.page + kDiagPages - 1) % kDiagPages);
        s.nav_tick = true;
    } else if (n.down || n.ok) {
        s.page = static_cast<uint8_t>((s.page + 1) % kDiagPages);
        s.nav_tick = true;
    }
    return {};
}

constexpr Event update_system(State& s, const Nav& n) {
    if (n.cancel) {
        s.screen = Screen::Main;
        s.cue = buzzer::Cue::Click;
    }
    return {};
}

// Advances the menu state for one tick. `st` is edited in place; the caller persists it when told to.
// Value picking where a pot movement selects VRA/VRB (Rust `is_source_selection`).
constexpr bool is_source_selection(const State& s) {
    return s.editing && (s.screen == Screen::AuxChannels || (s.screen == Screen::MixerLine && s.mix_line.selected == 1));
}

// Per-screen handler for one tick of navigation keys.
constexpr Event update_screen(State& s, const Nav& n, const Input& in, storage::RadioStorage& st) {
    switch (s.screen) {
        case Screen::Main: return update_main(s, n, st.radio.active_model);
        case Screen::ModelSelect: return update_models(s, n);
        case Screen::ModelSetup: return update_setup(s, n, st.active_model, st.radio, model_copy::active_slot(st));
        case Screen::DualRate: return update_dual_rate(s, n, st.active_model, st.radio);
        case Screen::ThrottleCurve: return update_throttle_curve(s, n, st.active_model);
        case Screen::AuxChannels: return update_aux(s, n, st.active_model, st.radio);
        case Screen::RadioSetup: return update_radio_setup(s, n, st.radio);
        case Screen::InputSetup: return update_input_setup(s, n, st.radio);
        case Screen::ProtocolSetup: return update_protocol(s, n, st.active_model);
        case Screen::CrsfSetup: return update_crsf_setup(s, n, in);
        case Screen::VarioSetup: return update_vario(s, n, st.active_model, st.radio);
        case Screen::TelemSetup: return update_telem(s, n, st.active_model);
#if WM_PCB_I6S && WM_USE_MULTISWITCH
        case Screen::MultiSwitchSetup: return update_multiswitch(s, n, st.active_model);
#endif
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

// OK confirms with a click as in Rust (`buzzer.click()` in the screen handlers). Configure Module plays its own
// tones; Channel Monitor and System Info ignore OK.
constexpr bool ok_clicks(const Screen screen) {
    switch (screen) {
        case Screen::CrsfSetup:
        case Screen::ChannelMonitor:
        case Screen::SystemInfo:
        case Screen::Closed: return false;
        default: return true;
    }
}

constexpr Event update(State& s, const Input& in, storage::RadioStorage& st) {
    s.nav_tick = false;
    s.cue = buzzer::Cue::None;
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
    n.bind = (in.pressed & kKeyBind) != 0;
    n.changes = in.changes;
    if constexpr (board::kHasJogWheels) {
        const auto raw_vra = input::mode_from_u8(st.radio.adc_modes[4]);
        const auto raw_vrb = input::mode_from_u8(st.radio.adc_modes[5]);
        const bool picking = is_source_selection(s);
        if (picking) {
            jog::prefer_deflected_pot(n.changes, raw_vra, in.vr1, raw_vrb, in.vr2);
        }
        const bool jog_active = !picking && s.screen != Screen::InputSetup;
        const auto step = s.jog.update(jog_active, raw_vra, in.vr1, raw_vrb, in.vr2);
        n.up = n.up || step == jog::Step::Up;
        n.down = n.down || step == jog::Step::Down;
    }

    const Screen screen = s.screen;
    const Event ev = update_screen(s, n, in, st);
    if (n.ok && s.cue == buzzer::Cue::None && !s.nav_tick && ok_clicks(screen)) {
        s.cue = buzzer::Cue::Click;
    }
    return ev;
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

struct Ordinal {
    std::size_t value;
    friend constexpr bool operator==(const Ordinal&, const Ordinal&) = default;
};

struct Total {
    std::size_t value;
    friend constexpr bool operator==(const Total&, const Total&) = default;
};

constexpr format::Text<8> counter_text(const Ordinal cur, const Total total) {
    format::Text<8> t;
    format::detail::push_uint(t, static_cast<unsigned>(cur.value));
    t.push('/');
    format::detail::push_uint(t, static_cast<unsigned>(total.value));
    return t;
}

// "Mnn:name" with a trailing '*' for the active model.
constexpr format::Text<20> model_row_text(const std::size_t idx, const ModelName& name, const bool active) {
    format::Text<20> t;
    t.push('M');
    const auto num = format::format_dec2(static_cast<unsigned>(idx) + 1u);
    for (const char c : num.view()) {
        t.push(c);
    }
    t.push(':');
    // All 10 name characters at fixed width, then " [*]" on the active model (Rust `update_select`).
    for (const uint8_t c : name) {
        t.push(c >= 0x20 && c < 0x7F ? static_cast<char>(c) : ' ');
    }
    if (active) {
        for (const char c : std::string_view{" [*]"}) {
            t.push(c);
        }
    }
    return t;
}

// "mm:ss", minutes capped at 99.
constexpr format::Text<5> timer_text(const uint16_t secs) {
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
constexpr void draw_icon_row(Framebuffer& fb, const int slot, const bool selected, const glyphs::Glyph12& icon,
                             std::string_view label) {
    const int y = 13 + slot * 14;
    if (selected) {
        fb.fill_rect({{2, y}, {121, 13}}, true);
    }
    glyphs::draw_glyph_12x12(fb, Point{4, y + 1}, icon, !selected);
    fb.draw_str_6x10(Point{19, y + 3}, label, selected);
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
    widgets::draw_scrollbar(fb, {s.main.selected}, {static_cast<int>(kMainItems.size())}, {13, 40});
    const auto pos = counter_text({s.main.selected + 1u}, {kMainItems.size()});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.models.selected, label.view(), {}, {0});
    }
    widgets::draw_footer(fb, "[OK] Load     [ESC] Back");
}

constexpr void render_setup(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "MODEL SETUP");
    constexpr int kValueX = 40;

    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.setup.scroll + slot;
        if (idx >= kSetupRows) {
            break;
        }
        const int y = 14 + static_cast<int>(slot) * 9;
        // The row stays highlighted while it is edited (Rust `is_sel || is_edit`).
        const bool selected = idx == s.setup.selected;
        switch (static_cast<SetupRow>(idx)) {
            case SetupRow::Name: {
                // Deliberate deviation: no row highlight while editing the name, otherwise the inverted
                // cursor cell is invisible on the inverted row (as it is in Rust).
                const auto name = name_text(m.name);
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected && !s.editing, "Name:", name.view(),
                                       {kValueX});
                if (s.editing && idx == s.setup.selected) {
                    const int char_x = kValueX + s.name_cursor * 6;
                    const uint8_t c = m.name[s.name_cursor];
                    const std::array<char, 1> one{c == 0 ? ' ' : static_cast<char>(c)};
                    fb.fill_rect({{char_x - 1, y}, {8, 9}}, true);
                    fb.draw_str_6x10(Point{char_x, y}, std::string_view(one.data(), 1), true);
                }
                break;
            }
            case SetupRow::Type:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Type:",
                                       labels::model_type_label(m.model_type), {kValueX});
                break;
            case SetupRow::ArmSwitch: {
                const std::string_view arm =
                    m.arm_switch == 0 ? std::string_view{"NONE"} : labels::switch_cond_label(m.arm_switch);
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Arm Sw:", arm, {52});
                break;
            }
            case SetupRow::Timer: {
                if (m.timer_secs == 0) {
                    widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Timer:", "OFF", {52});
                } else {
                    const auto t = timer_text(m.timer_secs);
                    widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Timer:", t.view(), {52});
                }
                break;
            }
            case SetupRow::Trigger:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "T-Trig:",
                                       labels::timer_source_label(m.timer_source), {52});
                break;
            case SetupRow::Copy: {
                if (s.editing && idx == s.setup.selected) {
                    const auto target = model_copy::target_label(s.copy.dst);
                    widgets::draw_list_row(fb, {static_cast<int>(slot)}, true, target.view(), {}, {0});
                } else {
                    widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Copy: [OK Duplicate]", {}, {0});
                }
                break;
            }
            case SetupRow::Vario:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Vario: [SETUP]", {}, {0});
                break;
            case SetupRow::Telemetry:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Telemetry: [SETUP]", {}, {0});
                break;
            case SetupRow::Protocol:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Protocol:",
                                       protocol::label(protocol::from_raw(m.rf_protocol)), {64});
                break;
            case SetupRow::Reset:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "Reset: [OK Defaults]", {}, {0});
                break;
#if WM_PCB_I6S && WM_USE_MULTISWITCH
            case SetupRow::MultiSwitch:
                widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, "MultiSwitch: [SETUP]", {}, {0});
                break;
#endif
        }
    }

    if (!s.editing) {
        widgets::draw_footer(fb, "[OK] Select    [ESC] Back");
    } else if (static_cast<SetupRow>(s.setup.selected) == SetupRow::Name) {
        widgets::draw_footer(fb, "[OK] Next Char [ESC] Done");
    } else if (static_cast<SetupRow>(s.setup.selected) == SetupRow::Copy) {
        widgets::draw_footer(fb, "[OK] Duplicate [ESC] Cancel");
    } else {
        widgets::draw_footer(fb, "[OK] Done   [UP/DN] Value");
    }
}

#if WM_PCB_I6S && WM_USE_MULTISWITCH
constexpr void render_multiswitch(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "MULTISWITCH SETUP");
    constexpr std::array<std::string_view, 4> labels{"MS1 Addr:", "MS2 Addr:", "MS3 Addr:", "MS4 Addr:"};
    for (std::size_t slot = 0; slot < m.multiswitch_addresses.size(); ++slot) {
        const auto address = m.multiswitch_addresses[slot];
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, slot == s.multiswitch.selected, labels[slot],
                               address < 0 ? std::string_view{"Off"} : format::format_int(address).view(), {64});
    }
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Value" : "[OK] Edit   [ESC] Back");
}
#endif

constexpr void render_telem(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "TELEM DASH SETUP");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.telem.scroll + slot;
        if (idx >= telemetry::kTelemSlotCount) {
            break;
        }
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.telem.selected,
                               telemetry::kTelemSlotNames[idx], telemetry::telem_sensor_name(m.telem_slots[idx]),
                               {0});
    }
    widgets::draw_footer(fb, "[OK] Cycle Sensor  [ESC] Back");
}

constexpr void render_vario(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "VARIO SETUP");
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.vario.scroll + slot;
        if (idx >= kVarioRows) {
            break;
        }
        const bool selected = idx == s.vario.selected;
        const int slot_i = static_cast<int>(slot);
        switch (static_cast<VarioRow>(idx)) {
            case VarioRow::Switch: {
                const uint8_t sw = m.vario_switch;
                const std::string_view value = sw == 0   ? std::string_view{"OFF"}
                                               : sw == 1 ? std::string_view{"ON"}
                                               : sw < kVarioSwitchCount
                                                   ? labels::switch_cond_label(static_cast<uint8_t>(sw - 1))
                                                   : std::string_view{"OFF"};
                widgets::draw_list_row(fb, {slot_i}, selected, "Switch:", value, {0});
                break;
            }
            case VarioRow::ClimbDeadband: {
                const auto t = format::format_deadband_ms(m.vario_climb_deadband, false);
                widgets::draw_list_row(fb, {slot_i}, selected, "Climb Dead:", t.view(), {0});
                break;
            }
            case VarioRow::SinkDeadband: {
                const auto t = format::format_deadband_ms(m.vario_sink_deadband, true);
                widgets::draw_list_row(fb, {slot_i}, selected, "Sink Dead:", t.view(), {0});
                break;
            }
            case VarioRow::SinkMode:
                widgets::draw_list_row(fb, {slot_i}, selected, "Sink Mode:",
                                       labels::vario_sink_mode_label(m.vario_sink_mode), {0});
                break;
            case VarioRow::MotorMute:
                widgets::draw_list_row(fb, {slot_i}, selected, "Motor Mute:",
                                       labels::vario_motor_mute_label(m.vario_motor_mute), {0});
                break;
        }
    }
    widgets::draw_scrollbar(fb, {s.vario.selected}, {static_cast<int>(kVarioRows)}, {14, 36});
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Switch" : "[OK] Edit/Cycle    [ESC] Back");
}

inline constexpr std::array<std::string_view, kCrsfBaudCount> kCrsfBaudNames{
    "115.2k (Low)", "416.6k (TBS)", "420k (ELRS)", "921.6k (Fast)", "1.875M (Max)"};

// `sv`, wrapped in [] while it is being edited.
template <std::size_t N>
constexpr format::Text<N> bracketed(const std::string_view sv, const bool edit) {
    format::Text<N> t;
    if (edit) {
        t.push('[');
    }
    for (const char c : sv) {
        t.push(c);
    }
    if (edit) {
        t.push(']');
    }
    return t;
}

constexpr void render_afhds_row(Framebuffer& fb, const int slot, const AfhdsRow row, const bool selected, const bool edit,
                                const storage::ModelConfig& m) {
    switch (row) {
        case AfhdsRow::Proto: {
            const auto t = bracketed<12>(protocol::label(protocol::from_raw(m.rf_protocol)), edit);
            widgets::draw_list_row(fb, {slot}, selected, "Proto:", t.view(), {0});
            break;
        }
        case AfhdsRow::Bind: {
            format::Text<16> t;
            for (const char c : std::string_view{"[Bind: "}) {
                t.push(c);
            }
            for (const char c : format::format_hex32(m.rx_id).view()) {
                t.push(c);
            }
            t.push(']');
            widgets::draw_list_row(fb, {slot}, selected, t.view(), {}, {0});
            break;
        }
        case AfhdsRow::Power:
            widgets::draw_list_row(fb, {slot}, selected, "Power:", kRfPowerNames[m.rf_power % kRfPowerCount], {0});
            break;
        case AfhdsRow::ServoHz: {
            const uint16_t hz = std::clamp<uint16_t>(m.servo_rate_hz, 50, 400);
            std::array<char, 3> digits{};
            std::size_t n = 0;
            for (uint16_t v = hz; v > 0; v /= 10) {
                digits[n++] = static_cast<char>('0' + v % 10);
            }
            format::Text<8> t;
            while (n > 0) {
                t.push(digits[--n]);
            }
            for (const char c : std::string_view{" Hz"}) {
                t.push(c);
            }
            widgets::draw_list_row(fb, {slot}, selected, "Servo Hz:", t.view(), {0});
            break;
        }
        case AfhdsRow::RxOut:
            widgets::draw_list_row(fb, {slot}, selected, "RX Out:", m.rx_out_mode == 0 ? "PWM" : "PPM", {0});
            break;
        case AfhdsRow::Serial:
            widgets::draw_list_row(fb, {slot}, selected, "Serial:", m.rx_serial_proto == 0 ? "i-BUS" : "S.BUS", {0});
            break;
    }
}

constexpr void render_protocol(Framebuffer& fb, const State& s, const storage::ModelConfig& m) {
    widgets::draw_header(fb, "PROTOCOL SETUP");
    const bool crsf_active = protocol::from_raw(m.rf_protocol) == protocol::Protocol::Crsf;
    const std::size_t rows = crsf_active ? kProtocolRowsCrsf : kProtocolRowsAfhds;
    for (std::size_t slot = 0; slot < kListVisible; ++slot) {
        const std::size_t idx = s.protocol.scroll + slot;
        if (idx >= rows) {
            break;
        }
        const bool selected = idx == s.protocol.selected;
        const bool edit = selected && s.editing;
        const int slot_i = static_cast<int>(slot);
        if (!crsf_active) {
            render_afhds_row(fb, slot_i, static_cast<AfhdsRow>(idx), selected, edit, m);
            continue;
        }
        switch (static_cast<CrsfRow>(idx)) {
            case CrsfRow::Proto: {
                const auto t = bracketed<12>(protocol::label(protocol::from_raw(m.rf_protocol)), edit);
                widgets::draw_list_row(fb, {slot_i}, selected, "Proto:", t.view(), {0});
                break;
            }
            case CrsfRow::Baud: {
                const auto t = bracketed<16>(kCrsfBaudNames[std::min<std::size_t>(m.crsf_baud, kCrsfBaudCount - 1)], edit);
                widgets::draw_list_row(fb, {slot_i}, selected, "Baud:", t.view(), {0});
                break;
            }
            case CrsfRow::Duplex:
                widgets::draw_list_row(fb, {slot_i}, selected, "Duplex:",
                                       m.crsf_half_duplex == 0 ? "Full (2W)" : "Half (1W)", {0});
                break;
            case CrsfRow::ArmMode:
                widgets::draw_list_row(fb, {slot_i}, selected, "Arm Mode:",
                                       m.crsf_arm_mode == 0 ? "CH5 (Std)" : "Switch", {0});
                break;
            case CrsfRow::Configure:
                widgets::draw_list_row(fb, {slot_i}, selected, "[Configure Module]", {}, {0});
                break;
        }
    }
    widgets::draw_scrollbar(fb, {s.protocol.selected}, {static_cast<int>(rows)}, {14, 36});
    if (s.editing) {
        widgets::draw_footer(fb, "[OK] Save   [UP/DN] Change");
    } else if (!crsf_active) {
        if (s.protocol.selected == static_cast<uint8_t>(AfhdsRow::Bind)) {
            widgets::draw_footer(fb, "[OK] Start Bind   [ESC] Exit");
        } else if (s.protocol.selected >= static_cast<uint8_t>(AfhdsRow::Power)) {
            widgets::draw_footer(fb, "[OK] Toggle/Cycle [ESC] Exit");
        } else {
            widgets::draw_footer(fb, "[OK] Edit   [ESC] Exit");
        }
    } else if (s.protocol.selected == static_cast<uint8_t>(CrsfRow::Duplex) ||
               s.protocol.selected == static_cast<uint8_t>(CrsfRow::ArmMode)) {
        widgets::draw_footer(fb, "[OK] Toggle Mode  [ESC] Exit");
    } else if (s.protocol.selected == static_cast<uint8_t>(CrsfRow::Configure)) {
        widgets::draw_footer(fb, "[OK] Open Config  [ESC] Exit");
    } else {
        widgets::draw_footer(fb, "[OK] Edit   [ESC] Exit");
    }
}

constexpr std::array<std::string_view, kMonitorPages> kMonitorTitles{"CHANNELS (1-6)", "CHANNELS (7-12)",
                                                                   "CHANNELS (13-18)"};
#if WM_PCB_I6S
constexpr std::array<std::string_view, kDiagPages> kDiagTitles{
    "ANALOG (1-6)", "ANALOG (7-11)", "SWITCHES (E/F/G/H)", "EXT ANALOG (PC1-5)", "STORAGE (EEPROM)"};
#else
constexpr std::array<std::string_view, kDiagPages> kDiagTitles{
    "ANALOG (1-6)", "ANALOG (7-11)", "EXT SWITCHES (SE/SF)", "EXT ANALOG (P7)", "STORAGE (EEPROM)"};
#endif
// Raw ADC index of each row. Analog page 1 starts at raw index 6 (PA6).
constexpr std::array<std::array<std::string_view, 6>, 2> kAnalogNames{{
    {"RH:AIL", "RV:ELE", "LV:THR", "LH:RUD", "SW:SA ", "SW:SB "},
    {"POT:V1", "POT:V2", "SW:SC ", "SW:SD ", "VBAT  ", ""},
}};
constexpr std::array<std::size_t, 2> kAnalogRows{6, 5};
// P7 external ADC rows: raw index 11..14 (AD12..AD15).
constexpr std::array<std::string_view, 4> kP7Names{"VRC:AD12", "VRD:AD13", "VRE:AD14", "VRF:AD15"};
constexpr std::size_t kP7RawStart = 11;
constexpr int kGaugeMaxFill = 38;
constexpr int kSwitchBarFill = 30;

// Copies a short label or number into a fixed buffer so the caller can hold the value for one row.
constexpr format::Text<6> to_text(std::string_view sv) {
    format::Text<6> t;
    for (const char c : sv) {
        t.push(c);
    }
    return t;
}

struct Axis {
    std::size_t value;
    friend constexpr bool operator==(const Axis&, const Axis&) = default;
};

struct DrField {
    std::size_t value;
    friend constexpr bool operator==(const DrField&, const DrField&) = default;
};

constexpr format::Text<6> dr_value(const storage::ModelConfig& m, const Axis axis_sel, const DrField field) {
    const std::size_t axis = axis_sel.value;
    switch (field.value) {
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
        const format::Text<6> value = dr_value(m, {s.axis}, {idx});
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.dual_rate.selected, kDrLabels[idx], value.view(),
                               {60});
    }
    widgets::draw_footer(fb, s.editing ? "[OK] Done   [UP/DN] Value" : "[OK] Edit   [ESC] Exit");
}

// "P01:  50%" as on the device: point number, then the value right-aligned in three columns.
constexpr format::Text<9> curve_point_label(const std::size_t idx, const uint8_t val) {
    const auto digit = [](const std::size_t d) { return static_cast<char>('0' + d); };
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
        fb.fill_rect({{2, 13}, {70, 9}}, true);
    }
    fb.draw_str_6x10(Point{4, 13}, "Pts:", sel_pts);
    fb.draw_str_6x10(Point{32, 13}, pts == 9 ? "9-PT" : (pts == 7 ? "7-PT" : "5-PT"), sel_pts);

    const bool sel_crv = !s.editing && sel == 1;
    if (sel_crv) {
        fb.fill_rect({{2, 23}, {70, 9}}, true);
    }
    fb.draw_str_6x10(Point{4, 23}, "Crv:", sel_crv);
    fb.draw_str_6x10(Point{32, 23}, m.thr_curve_smooth != 0 ? "SMOOTH" : "LINEAR", sel_crv);

    if (sel >= 2) {
        const format::Text<9> label = curve_point_label(sel - 2, m.thr_curve[sel - 2]);
        if (s.editing) {
            fb.fill_rect({{2, 35}, {70, 11}}, true);
            fb.draw_str_6x10(Point{4, 37}, label.view(), true);
        } else {
            fb.draw_rect({{2, 35}, {70, 11}}, true);
            fb.draw_str_6x10(Point{4, 37}, label.view(), false);
        }
    } else {
        fb.draw_str_6x10(Point{4, 37}, pts == 9 ? "Pts: 1..9" : (pts == 7 ? "Pts: 1..7" : "Pts: 1..5"), false);
    }

    // Graph box 49x37 at (76,13). Plot area 47 px wide, output 0..34 px high.
    fb.draw_rect({{76, 13}, {49, 37}}, true);
    const bool smooth = m.thr_curve_smooth != 0;
    for (int px = 0; px < 47; ++px) {
        const auto input = static_cast<uint16_t>(px * 1000 / 46);
        const uint16_t out = curve::evaluate(input, m.thr_curve_pts, smooth, m.thr_curve);
        fb.set_pixel(Point{77 + px, 48 - out * 34 / 1000}, true);
    }
    if (sel >= 2) {
        const std::size_t idx = sel - 2;
        const int input = static_cast<int>(idx * 1000 / (pts - 1));
        const int dot_x = 77 + input * 46 / 1000;
        const int dot_y = 48 - (m.thr_curve[idx] * 10 * 34) / 1000;
        fb.fill_rect({{dot_x - 1, dot_y - 1}, {3, 3}}, true);
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
constexpr format::Text<12> radio_value(const storage::RadioConfig& r, const std::size_t row) {
    format::Text<12> t;
    auto put = [&t](std::string_view sv) {
        for (const char c : sv) {
            t.push(c);
        }
    };
    if (row >= kRadioRows) {
        return t;
    }
    switch (kRadioItems[row]) {
        case RadioItem::ThrTrim: put(r.throttle_trim == 1 ? "IDLE" : r.throttle_trim == 2 ? "LINEAR" : "OFF (Lock)"); break;
        case RadioItem::Beeper: put(r.audio_enabled != 0 ? "ENABLED" : "MUTED"); break;
        case RadioItem::Tones: put(r.tone_style != 0 ? "RICH" : "SIMPLE"); break;
        case RadioItem::BlTimer:
            put(r.backlight_timeout == 1   ? "15 SEC"
                : r.backlight_timeout == 2 ? "30 SEC"
                : r.backlight_timeout == 3 ? "60 SEC"
                                           : "ALWAYS ON");
            break;
        case RadioItem::BlLevel: {
            format::detail::push_uint(t, std::min<unsigned>(r.backlight_brightness * 10u, 100u));
            t.push('%');
            break;
        }
        case RadioItem::Contrast: {
            const auto num = format::format_dec2(std::min<unsigned>(r.lcd_contrast, 99u));
            put(num.view());
            break;
        }
        case RadioItem::BatWarn: {
            format::detail::push_uint(t, r.vbat_warn_deci / 10u);
            t.push('.');
            t.push(static_cast<char>('0' + r.vbat_warn_deci % 10u));
            t.push('V');
            break;
        }
        case RadioItem::UsbMode:
            put(r.usb_mode == 1 ? "JOYSTICK" : r.usb_mode == 2 ? "SERIAL" : r.usb_mode == 3 ? "COMPOSITE" : "OFF");
            break;
        case RadioItem::ModulePwr: put(r.ext_module_pwr == 0 ? "HIGH (N)" : "LOW (P)"); break;
        case RadioItem::ExtSwitches: put(r.ext_switches != 0 ? "PC12+PC15" : "OFF"); break;
        case RadioItem::ExtAdc:
            put(board::kIsI6S ? (r.ext_adc != 0 ? "ENABLED" : "DISABLED") : (r.ext_adc != 0 ? "AD12-15" : "OFF"));
            break;
        case RadioItem::StickMode: {
            constexpr std::array<std::string_view, 4> kStickModes{"MODE 1", "MODE 2", "MODE 3", "MODE 4"};
            put(kStickModes[r.stick_mode % 4]);
            break;
        }
        case RadioItem::Gimbals: put(r.gimbal_type == 1 ? "HALL" : "POT"); break;
        case RadioItem::InputSetup: put(board::kIsI6S ? "[SETUP]" : ">"); break;
        case RadioItem::PpmOut:
            put((r.trainer_config & kTrainerPpmOut) != 0 ? (board::kIsI6S ? "ENABLED" : "ON (PF10)") : "OFF");
            break;
        case RadioItem::TrainerIn:
            put((r.trainer_config & kTrainerPpmIn) != 0 ? (board::kIsI6S ? "ENABLED" : "ON (SD DN)") : "OFF");
            break;
        case RadioItem::J4Mode: put(r.j4_sg_sh_en != 0 ? "SG/SH BTN" : "SWD (DBG)"); break;
        case RadioItem::RearLeft:
            put(r.rear_left_func == 1 ? "TIMER RESET" : r.rear_left_func == 2 ? "TRIMS MOD" : "RC SW (SWE)");
            break;
        case RadioItem::RearRight:
            put(r.rear_right_func == 1 ? "INSTANT TRIM" : r.rear_right_func == 2 ? "TRIMS MOD" : "RC SW (SWF)");
            break;
        case RadioItem::Touch: put(r.touch_disabled == 0 ? "ENABLED" : "DISABLED"); break;
        case RadioItem::SwitchSound: put(r.switch_beep != 0 ? "ENABLED" : "MUTED"); break;
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.radio.selected, radio_label(kRadioItems[idx]), value.view(), {0});
    }
    widgets::draw_scrollbar(fb, {s.radio.selected}, {static_cast<int>(kRadioRows)}, {14, 36});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.input_setup.selected, kAuxInputNames[idx],
                               kAdcModeNames[mode], {0});
    }
    widgets::draw_scrollbar(fb, {s.input_setup.selected}, {static_cast<int>(count)}, {14, 36});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.aux.selected, label.view(),
                               labels::kSourceNames[src_idx], {48});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.reverse.selected, labels::kChannelNames[idx],
                               reversed ? "REVERSE" : "NORMAL ", {76});
    }
    widgets::draw_scrollbar(fb, {s.reverse.selected}, {static_cast<int>(kNumChannels)}, {14, 36});
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
        widgets::draw_gauge_row(fb, {12 + static_cast<int>(row) * 7}, label.view(), {fill}, static_cast<uint16_t>(us));
    }
    widgets::draw_footer(fb, "[UP/DN] Page  [ESC] Back");
}

// Gauge fill for a raw ADC value, 0..kGaugeMaxFill.
constexpr int adc_fill(const uint16_t raw) {
    const uint16_t clamped = std::min<uint16_t>(raw, kAdcFullScale);
    return std::min(static_cast<int>(clamped) * kGaugeMaxFill / static_cast<int>(kAdcFullScale), kGaugeMaxFill);
}

constexpr std::string_view ext_switch_config_label(const bool enabled) {
    return enabled ? "EXT SW: ENABLED (PC12/15)" : "EXT SW: DISABLED (OFF)";
}

constexpr std::string_view switch_pos_label(const SwitchPos pos) {
    return pos == SwitchPos::Down ? "DN (LOW)" : "UP (HIGH)";
}

constexpr int switch_fill(const SwitchPos pos) { return pos == SwitchPos::Down ? kSwitchBarFill : 0; }

constexpr std::string_view p7_config_label(const bool enabled) {
    return enabled ? "P7 ADC: ENABLED (15-CH)" : "P7 ADC: DISABLED (OFF)";
}

// Four-character value of a P7 row: the raw count when the ADC is enabled, " OFF" otherwise.
constexpr format::Text<4> p7_value(const uint16_t raw, const bool enabled) {
    format::Text<4> t;
    if (enabled) {
        const auto digits = widgets::format_u16_4(std::min<uint16_t>(raw, kAdcFullScale));
        for (const char c : digits) {
            t.push(c);
        }
    } else {
        for (const char c : std::string_view{" OFF"}) {
            t.push(c);
        }
    }
    return t;
}

// "ADDR: 0x50 (FT24C128A 16KB)" with the device address in hex.
constexpr format::Text<27> eeprom_addr_line(const uint8_t addr) {
    format::Text<27> t;
    for (const char c : std::string_view{"ADDR: 0x"}) {
        t.push(c);
    }
    t.push(format::detail::hex_digit(addr >> 4));
    t.push(format::detail::hex_digit(addr));
    for (const char c : std::string_view{" (FT24C128A 16KB)"}) {
        t.push(c);
    }
    return t;
}

constexpr std::string_view eeprom_state_label(const storage::EepromStatus& e) {
    if (e.write_err) {
        return "STATUS: WRITE ERR (FAIL)";
    }
    return e.read_ok ? "STATUS: READY (HEALTHY)" : "STATUS: READ ERR (FAIL)";
}

constexpr void draw_analog_page(Framebuffer& fb, const std::size_t page, const input::Raw& adc) {
    const std::size_t start = page == 0 ? 0 : 6;
    for (std::size_t row = 0; row < kAnalogRows[page]; ++row) {
        const uint16_t raw = std::min<uint16_t>(adc[start + row], kAdcFullScale);
        widgets::draw_gauge_row(fb, {13 + static_cast<int>(row) * 7}, kAnalogNames[page][row], {adc_fill(raw)}, raw);
    }
}

constexpr void draw_ext_switches_page(Framebuffer& fb, const Switches& sw, const bool enabled) {
    fb.draw_str_4x6(Point{2, 13}, ext_switch_config_label(enabled), false);
    fb.draw_str_4x6(Point{2, 23}, "SW:SE", false);
    widgets::draw_bar_gauge(fb, {{32, 23}, {30, 6}}, switch_fill(sw.se));
    fb.draw_str_4x6(Point{66, 23}, switch_pos_label(sw.se), false);
    fb.draw_str_4x6(Point{2, 33}, "SW:SF", false);
    widgets::draw_bar_gauge(fb, {{32, 33}, {30, 6}}, switch_fill(sw.sf));
    fb.draw_str_4x6(Point{66, 33}, switch_pos_label(sw.sf), false);
    fb.draw_str_4x6(Point{2, 44}, "Active-LOW mod to GND", false);
}

// I6S page 2: rear buttons (SE/SF) and J4 buttons (SG/SH).
constexpr void draw_board_switches_page(Framebuffer& fb, const Switches& sw) {
    const std::array<std::pair<std::string_view, SwitchPos>, 4> items{{
        {"SW:SE", sw.se}, {"SW:SF", sw.sf}, {"SW:SG", sw.sg}, {"SW:SH", sw.sh}}};
    for (std::size_t i = 0; i < items.size(); ++i) {
        const int y = 13 + static_cast<int>(i) * 9;
        fb.draw_str_4x6(Point{2, y}, items[i].first, false);
        widgets::draw_bar_gauge(fb, {{32, y}, {30, 6}}, switch_fill(items[i].second));
        fb.draw_str_4x6(Point{66, y}, switch_pos_label(items[i].second), false);
    }
}

// I6S page 3: PC1..PC5 (raw 11..15); inputs in mode Off show " OFF".
constexpr void draw_pc_adc_page(Framebuffer& fb, const input::Raw& adc, const storage::RadioConfig& radio) {
    const bool enabled = radio.ext_adc != 0;
    fb.draw_str_4x6(Point{2, 13}, enabled ? "PC ADC: ENABLED (16-CH)" : "PC ADC: DISABLED (OFF)", false);
    constexpr std::array<std::string_view, 5> kNames{"VRC:PC1 ", "VRD:PC2 ", "VRE:PC3 ", "VRF:PC4 ", "VRG:PC5 "};
    for (std::size_t i = 0; i < kNames.size(); ++i) {
        const int y = 23 + static_cast<int>(i) * 8;
        const uint16_t raw = std::min<uint16_t>(adc[kP7RawStart + i], kAdcFullScale);
        const bool active = enabled && radio.adc_modes[6 + i] != static_cast<uint8_t>(input::AdcMode::Off);
        fb.draw_str_4x6(Point{2, y}, kNames[i], false);
        widgets::draw_bar_gauge(fb, {{44, y + 1}, {40, 5}}, active ? adc_fill(raw) : 0);
        const format::Text<4> value = p7_value(raw, active);
        fb.draw_str_4x6(Point{90, y}, value.view(), false);
    }
}

constexpr void draw_ext_adc_page(Framebuffer& fb, const input::Raw& adc, const bool enabled) {
    fb.draw_str_4x6(Point{2, 13}, p7_config_label(enabled), false);
    for (std::size_t i = 0; i < kP7Names.size(); ++i) {
        const int y = 25 + static_cast<int>(i) * 7;
        const uint16_t raw = std::min<uint16_t>(adc[kP7RawStart + i], kAdcFullScale);
        fb.draw_str_4x6(Point{2, y}, kP7Names[i], false);
        widgets::draw_bar_gauge(fb, {{44, y + 1}, {40, 5}}, enabled ? adc_fill(raw) : 0);
        const format::Text<4> value = p7_value(raw, enabled);
        fb.draw_str_4x6(Point{90, y}, value.view(), false);
    }
}

constexpr void draw_eeprom_page(Framebuffer& fb, const storage::EepromStatus& e) {
    if (!e.detected) {
        fb.draw_str_4x6(Point{2, 13}, "BUS: I2C2 (PB10/PB11)", false);
        fb.draw_str_4x6(Point{2, 23}, "STATUS: NOT DETECTED", false);
        fb.draw_str_4x6(Point{2, 33}, "CHECK: Pull-ups / Wiring", false);
        fb.draw_str_4x6(Point{2, 43}, "FALLBACK: Factory Defaults", false);
        return;
    }
    const format::Text<27> addr = eeprom_addr_line(e.addr);
    fb.draw_str_4x6(Point{2, 13}, e.bus == 1 ? "BUS: I2C1 (PB8/PB9)" : "BUS: I2C2 (PB10/PB11 Fm+)", false);
    fb.draw_str_4x6(Point{2, 23}, addr.view(), false);
    fb.draw_str_4x6(Point{2, 33}, eeprom_state_label(e), false);
    fb.draw_str_4x6(Point{2, 43}, "SLOTS: 2xRadio  20xModels", false);
}

constexpr void render_diag(Framebuffer& fb, const State& s, const Live& live, const storage::RadioConfig& radio) {
    widgets::draw_header(fb, kDiagTitles[s.page]);
    switch (s.page) {
        case 0:
        case 1: draw_analog_page(fb, s.page, live.adc); break;
#if WM_PCB_I6S
        case 2: draw_board_switches_page(fb, live.switches); break;
        case 3: draw_pc_adc_page(fb, live.adc, radio); break;
#else
        case 2: draw_ext_switches_page(fb, live.switches, radio.ext_switches != 0); break;
        case 3: draw_ext_adc_page(fb, live.adc, radio.ext_adc != 0); break;
#endif
        default: draw_eeprom_page(fb, live.eeprom); break;
    }
    widgets::draw_footer(fb, "[UP/DN/OK] Page  [ESC] Back");
}

constexpr void render_system(Framebuffer& fb, const Live& live) {
    widgets::draw_header(fb, "SYSTEM INFORMATION");
    fb.draw_str_4x6(Point{4, 14}, "MCU:", false);
    fb.draw_str_4x6(Point{48, 14}, live.mcu_name, false);
    fb.draw_str_4x6(Point{4, 22}, "Firmware:", false);
    const format::Text<sysinfo::kFirmwareLineMax> firmware = sysinfo::firmware_line(live.build);
    fb.draw_str_4x6(Point{48, 22}, firmware.view(), false);
    fb.draw_str_4x6(Point{4, 30}, "Flash: 128KB | SRAM: 16KB", false);
    const format::Text<sysinfo::kStorageLineMax> storage = sysinfo::storage_line(live.eeprom);
    fb.draw_str_4x6(Point{4, 38}, storage.view(), false);
    fb.draw_str_4x6(Point{4, 46}, "Profiles: 20 Models (16KB)", false);
    widgets::draw_footer(fb, "[ESC] Back");
}

// "+50%" with a forced sign, as the device shows Diff, Weight and Offset.
constexpr format::Text<6> signed_percent(const int v) {
    format::Text<6> t;
    t.push(v < 0 ? '-' : '+');
    for (const char c : format::format_int(v < 0 ? -v : v).view()) {
        t.push(c);
    }
    t.push('%');
    return t;
}

template <std::size_t N>
constexpr std::string_view name_at(const std::array<std::string_view, N>& names, const std::size_t idx) {
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
constexpr format::Text<4> mix_label(const std::size_t idx) {
    const std::size_t num = idx + 1;
    format::Text<4> t;
    t.push('M');
    t.push(static_cast<char>('0' + num / 10));
    t.push(static_cast<char>('0' + num % 10));
    t.push(':');
    return t;
}

// "CH12" for a non-zero target channel.
constexpr format::Text<4> target_text(const uint8_t ch) {
    format::Text<4> t;
    t.push('C');
    t.push('H');
    format::detail::push_uint(t, static_cast<unsigned>(ch));
    return t;
}

constexpr format::Text<11> mix_value(const storage::MixLine& mix, const std::size_t idx) {
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
            widgets::draw_list_row(fb, {row}, sel, "Wing:", labels::kTemplateNames[std::min<std::size_t>(m.wing_tail_mix, 3)], {36});
        } else if (wing_has_diff(m) && idx == 1) {
            const format::Text<6> diff = signed_percent(m.template_diff);
            widgets::draw_list_row(fb, {row}, sel, "Diff:", diff.view(), {36});
        } else if (idx == gvars_row) {
            widgets::draw_list_row(fb, {row}, sel, "GVars: [Edit GV1..5]", {}, {0});
        } else {
            const std::size_t mi = idx - (gvars_row + 1);
            const storage::MixLine& mix = m.mixes[mi];
            widgets::draw_list_row(fb, {row}, sel, mix_label(mi).view(), {}, {0});
            if (mix.target_ch == 0) {
                fb.draw_str_6x10(Point{32, y}, "[DISABLED]", sel);
            } else {
                fb.draw_str_6x10(Point{30, y}, target_text(mix.target_ch).view(), sel);
                fb.draw_str_6x10(Point{58, y}, "<-", sel);
                fb.draw_str_6x10(Point{74, y}, name_at(labels::kSourceNames, mix.source), sel);
            }
        }
    }
    widgets::draw_scrollbar(fb, {static_cast<int>(s.wing.selected)}, {static_cast<int>(rows)}, {14, 36});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.mix_line.selected, kMixLabels[idx], value.view(), {56});
    }
    widgets::draw_scrollbar(fb, {static_cast<int>(s.mix_line.selected)}, {static_cast<int>(kMixFields)}, {14, 36});
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
        widgets::draw_list_row(fb, {static_cast<int>(slot)}, idx == s.gvars.selected, kGvarLabels[idx], value.view(), {56});
    }
    widgets::draw_scrollbar(fb, {static_cast<int>(s.gvars.selected)}, {static_cast<int>(m.gvars.size())}, {14, 36});
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
        case Screen::ProtocolSetup: render_protocol(fb, s, st.active_model); break;
        case Screen::CrsfSetup: crsf::config::render(fb, s.crsf, live.crsf_loader, live.crsf_command, live.crsf_devices); break;
        case Screen::VarioSetup: render_vario(fb, s, st.active_model); break;
        case Screen::TelemSetup: render_telem(fb, s, st.active_model); break;
#if WM_PCB_I6S && WM_USE_MULTISWITCH
        case Screen::MultiSwitchSetup: render_multiswitch(fb, s, st.active_model); break;
#endif
        case Screen::ChannelReverse: render_channel_reverse(fb, s, st.active_model); break;
        case Screen::ChannelMonitor: render_monitor(fb, s, live, st.active_model); break;
        case Screen::DiagAnalog: render_diag(fb, s, live, st.radio); break;
        case Screen::SystemInfo: render_system(fb, live); break;
        case Screen::WingMixer: render_wing_mixer(fb, s, st.active_model); break;
        case Screen::MixerLine: render_mixer_line(fb, s, st.active_model); break;
        case Screen::GlobalVars: render_global_vars(fb, s, st.active_model); break;
        case Screen::Closed: break;
    }
}

}  // namespace wm::logic::menu
