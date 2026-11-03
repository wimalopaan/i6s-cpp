#pragma once

// FS-i6S specific host tests (flysky-i6s-rs behaviour). Included from test_logic.cpp after
// `check(bool, const char*)` is defined; only built with -DPCB=I6S.

#if WM_PCB_I6S

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "wm/board.hpp"
#include "wm/logic/auto_select.hpp"
#include "wm/logic/buzzer.hpp"
#include "wm/logic/flight.hpp"
#include "wm/logic/input.hpp"
#include "wm/logic/jog.hpp"
#include "wm/logic/labels.hpp"
#include "wm/logic/menu.hpp"
#include "wm/logic/mixer.hpp"
#include "wm/logic/storage.hpp"

inline void i6s_storage_tests() {
    namespace st = wm::logic::storage;
    using st::ModelConfig;
    using st::RadioConfig;

    // flysky-i6s-rs RadioConfig layout comments (148 bytes) and the 516-byte RadioStorage.
    check(sizeof(RadioConfig) == 148 && sizeof(st::RadioStorage) == 516 && sizeof(ModelConfig) == 368,
          "i6s storage: RadioConfig 148, RadioStorage 516, ModelConfig 368 bytes");
    check(offsetof(RadioConfig, aux_pots) == 48 && offsetof(RadioConfig, ext_module_pwr) == 114 &&
              offsetof(RadioConfig, servo_rate_hz) == 116 && offsetof(RadioConfig, ext_switches) == 120 &&
              offsetof(RadioConfig, ext_adc) == 121 && offsetof(RadioConfig, adc_modes) == 122 &&
              offsetof(RadioConfig, stick_mode) == 133 && offsetof(RadioConfig, trainer_config) == 138,
          "i6s storage: RadioConfig offsets up to trainer_config match Rust");
    check(offsetof(RadioConfig, rear_left_func) == 139 && offsetof(RadioConfig, rear_right_func) == 140 &&
              offsetof(RadioConfig, touch_disabled) == 141 && offsetof(RadioConfig, j4_sg_sh_en) == 142 &&
              offsetof(RadioConfig, switch_beep) == 143 && offsetof(RadioConfig, gimbal_type) == 144 &&
              offsetof(RadioConfig, stick_invert_mask) == 145 && offsetof(RadioConfig, pad_align) == 146,
          "i6s storage: RadioConfig tail offsets match Rust");
    check(offsetof(ModelConfig, vario_switch) == 346 && offsetof(ModelConfig, crsf_arm_mode) == 351 &&
              offsetof(ModelConfig, gvars) == 352 && offsetof(ModelConfig, telem_slots) == 362,
          "i6s storage: ModelConfig vario/gvars/telem tail offsets match Rust");

    const RadioConfig def = st::default_radio_config();
    check(def.adc_modes.size() == 11 && def.aux_pots.size() == 11 && def.rear_left_func == 0 &&
              def.rear_right_func == 0 && def.touch_disabled == 0,
          "i6s storage: 11 aux inputs, rear buttons default to RC switches, touch enabled");

    // Sanitize keeps everything the I6S can select and clears what lies beyond.
    st::RadioStorage s = st::default_storage();
    s.radio.adc_modes[10] = static_cast<uint8_t>(wm::logic::input::AdcMode::InstantTrim);
    s.radio.adc_modes[9] = wm::board::kMaxAdcMode + 1;
    s.active_model.arm_switch = 28;
    s.active_model.vario_switch = 29;
    s.active_model.aux_channels[0] = 41;  // VRG
    s.active_model.aux_channels[1] = 44;  // SWM
    s.active_model.aux_channels[2] = 45;  // GV1 is no aux source
    s.active_model.mixes[0] = st::MixLine{.target_ch = 5, .source = 49, .weight_src = 0, .weight = 100,
                                          .offset = 0, .switch_sel = 28, .mode = 0, .reserved = 0};
    s.active_model.mixes[1] = st::MixLine{.target_ch = 5, .source = 50, .weight_src = 0, .weight = 100,
                                          .offset = 0, .switch_sel = 0, .mode = 0, .reserved = 0};
    s.active_model.mixes[2] = st::MixLine{.target_ch = 5, .source = 1, .weight_src = 0, .weight = 100,
                                          .offset = 0, .switch_sel = 29, .mode = 0, .reserved = 0};
    st::sanitize(s);
    check(s.radio.adc_modes[10] == 7 && s.radio.adc_modes[9] == 0, "i6s sanitize: ADC modes 0..7 are kept");
    check(s.active_model.arm_switch == 28 && s.active_model.vario_switch == 29,
          "i6s sanitize: arm switch SWMv and vario switch SWMv are kept");
    check(s.active_model.aux_channels[0] == 41 && s.active_model.aux_channels[1] == 44 &&
              s.active_model.aux_channels[2] == 0,
          "i6s sanitize: aux sources VRG..SWM are kept, GV1 is cleared");
    check(s.active_model.mixes[0].source == 49 && s.active_model.mixes[0].switch_sel == 28 &&
              s.active_model.mixes[1].target_ch == 0 && s.active_model.mixes[2].target_ch == 0 &&
              s.active_model.mixes[2].switch_sel == 0,
          "i6s sanitize: mix source GV5 and switch SWMv are kept, codes beyond are disabled");
    s.active_model.arm_switch = 29;
    s.active_model.vario_switch = 30;
    st::sanitize(s);
    check(s.active_model.arm_switch == 0 && s.active_model.vario_switch == 0,
          "i6s sanitize: arm/vario switch codes beyond SWMv are cleared");
}

inline void i6s_input_tests() {
    namespace in = wm::logic::input;
    namespace st = wm::logic::storage;
    using wm::logic::SwitchPos;

    check(in::kAuxCount == 11 && in::kRawCount == 16 && in::aux_index_to_raw(10) == 15 &&
              in::aux_index_to_raw(9) == 14,
          "i6s input: 16 raw channels, VRG is raw 15");
    check(in::mode_from_u8(1) == in::AdcMode::Off && in::mode_from_u8(7) == in::AdcMode::InstantTrim &&
              in::mode_from_u8(8) == in::AdcMode::Default,
          "i6s input: stored modes 0..7 with Off at 1");
    check(in::resolve_mode(1, 0) == in::AdcMode::ThreePos && in::resolve_mode(0, 0) == in::AdcMode::TwoPos &&
              in::resolve_mode(10, 0) == in::AdcMode::Pot && in::resolve_mode(10, 1) == in::AdcMode::Off,
          "i6s input: SB defaults to 3-pos, VRG to pot");

    st::RadioConfig cfg = st::default_radio_config();
    cfg.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::TwoPos);
    in::Raw raw{};
    raw.fill(2048);
    raw[15] = 4000;
    in::Calibration cal{};
    in::apply_calibration(cfg, cal);
    check(in::process(raw, in::BoardSwitches{}, cal).switches.swm == SwitchPos::Up,
          "i6s input: SWM stays up without the external ADC");
    cfg.ext_adc = 1;
    in::apply_calibration(cfg, cal);
    check(in::process(raw, in::BoardSwitches{}, cal).switches.swm == SwitchPos::Down,
          "i6s input: SWM decodes 2-pos from raw 15");
    cfg.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::ThreePos);
    raw[15] = 2048;
    in::apply_calibration(cfg, cal);
    check(in::process(raw, in::BoardSwitches{}, cal).switches.swm == SwitchPos::Mid,
          "i6s input: SWM decodes 3-pos mid from raw 15");
    cfg.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::Pot);
    raw[15] = 4095;
    in::apply_calibration(cfg, cal);
    const auto pot_st = in::process(raw, in::BoardSwitches{}, cal);
    check(pot_st.switches.swm == SwitchPos::Up && pot_st.aux_pots[10] == 1000 && pot_st.pots.vr7 == 1000,
          "i6s input: VRG as pot reads raw 15 into pots.vr7");

    st::RadioConfig off_cfg = st::default_radio_config();
    off_cfg.adc_modes[4] = static_cast<uint8_t>(in::AdcMode::Off);
    in::Calibration off_cal{};
    in::apply_calibration(off_cfg, off_cal);
    in::Raw off_raw{};
    off_raw.fill(2048);
    off_raw[6] = 4095;
    const auto off_st = in::process(off_raw, in::BoardSwitches{}, off_cal);
    check(off_st.aux_pots[4] == 0 && off_st.pots.vr1 == 0, "i6s input: AdcMode::Off reads 0");

    const auto sw_st = in::process(off_raw, in::BoardSwitches{.se_high = true, .sf_high = false, .sg_high = false,
                                                              .sh_high = true},
                                   off_cal);
    check(sw_st.switches.se == SwitchPos::Up && sw_st.switches.sf == SwitchPos::Down &&
              sw_st.switches.sg == SwitchPos::Down && sw_st.switches.sh == SwitchPos::Up,
          "i6s input: SE..SH follow the board button levels");
}

inline void i6s_mixer_tests() {
    namespace mx = wm::logic::mixer;
    namespace lb = wm::logic::labels;
    using wm::logic::Switches;
    using wm::logic::SwitchPos;

    Switches sw{};
    sw.swk = SwitchPos::Down;
    sw.swl = SwitchPos::Up;
    sw.swm = SwitchPos::Down;
    check(!mx::is_switch_active(23, sw) && mx::is_switch_active(24, sw), "i6s mixer: SWK conditions 23/24");
    check(mx::is_switch_active(25, sw) && !mx::is_switch_active(26, sw), "i6s mixer: SWL conditions 25/26");
    check(!mx::is_switch_active(27, sw) && mx::is_switch_active(28, sw), "i6s mixer: SWM conditions 27/28");

    {
        namespace fl = wm::logic::flight;
        wm::logic::input::InputState in{};
        auto model = wm::logic::storage::default_model_config(0);
        model.aux_channels[0] = 41;  // CH5 <- VRG
        in.aux_pots[10] = 1000;
        const auto ch = fl::rf_channels(in, wm::logic::safety::StickMode::Mode2, model, {}, 0);
        check(ch[4] > 1900, "i6s flight: VRG (source 41) reaches the RF channel");
    }
    check(mx::eval_arm_switch(28, sw) && !mx::eval_arm_switch(27, sw) && !mx::eval_arm_switch(29, sw),
          "i6s mixer: arm switch 28 is valid, 29 is not");
    check(wm::logic::flight::is_armed(28, sw) && !wm::logic::flight::is_armed(27, sw),
          "i6s flight: SWM arm conditions");
    check(wm::logic::buzzer::arm_cue({wm::logic::buzzer::SwitchIndex{28}, true, false, false}) ==
              wm::logic::buzzer::Cue::Armed,
          "i6s buzzer: arm switch 28 chimes");

    const mx::Sticks sticks{};
    const std::array<int16_t, 7> pots{0, 0, 0, 0, 0, 0, 640};
    const mx::ChannelValues channels{};
    const mx::GVars gvars{111, 0, 0, 0, -222};
    sw.swl = SwitchPos::Mid;
    const auto eval = [&](const uint8_t src) { return mx::evaluate_source(src, sticks, pots, sw, channels, gvars); };
    check(eval(41) == 640, "i6s mixer: source 41 is VRG");
    check(eval(42) == wm::logic::kMixerMax && eval(43) == wm::logic::kMixerCenter && eval(44) == wm::logic::kMixerMax,
          "i6s mixer: sources 42..44 are SWK..SWM");
    check(eval(45) == 111 && eval(49) == -222 && eval(50) == wm::logic::kMixerCenter,
          "i6s mixer: GV1..GV5 are sources 45..49");

    check(lb::switch_cond_label(23) == "SWK^" && lb::switch_cond_label(26) == "SWLv" &&
              lb::switch_cond_label(28) == "SWMv",
          "i6s labels: switch conditions up to SWMv");
    check(lb::kSourceNames[41] == "VRG" && lb::kSourceNames[42] == "SWK" && lb::kSourceNames[44] == "SWM" &&
              lb::kSourceNames[45] == "GV1" && lb::kSourceNames[49] == "GV5",
          "i6s labels: source names VRG, SWK..SWM, GV1..GV5");
}

inline void i6s_availability_tests() {
    namespace mn = wm::logic::menu;
    namespace in = wm::logic::input;
    namespace st = wm::logic::storage;

    st::RadioStorage s = st::default_storage();
    st::RadioConfig& r = s.radio;
    const st::ModelConfig& m = s.active_model;

    check(mn::is_switch_cond_available(4, r) && mn::is_switch_cond_available(7, r),
          "i6s avail: SB and SC mid are usable by default (3-pos)");
    check(mn::is_switch_cond_available(11, r) && mn::is_switch_cond_available(14, r) &&
              mn::is_source_available(31, r, m) && mn::is_source_available(32, r, m),
          "i6s avail: SE/SF exist while the rear buttons are RC switches");
    r.rear_left_func = 1;
    check(!mn::is_switch_cond_available(11, r) && !mn::is_source_available(31, r, m) &&
              mn::is_switch_cond_available(13, r) && !mn::is_dr_switch_available(5, r) &&
              mn::is_dr_switch_available(6, r),
          "i6s avail: rear left as timer reset removes only SE");
    r.rear_right_func = 2;
    r.ext_switches = 1;
    check(!mn::is_switch_cond_available(14, r) && !mn::is_source_available(32, r, m),
          "i6s avail: rear right as trim modifier removes SF, ext_switches has no effect");

    for (uint8_t c = 23; c <= 28; ++c) {
        check(!mn::is_switch_cond_available(c, r), "i6s avail: SWK..SWM need the external ADC");
    }
    check(!mn::is_source_available(41, r, m), "i6s avail: VRG needs the external ADC");
    r.ext_adc = 1;
    check(mn::is_source_available(41, r, m) && !mn::is_source_available(42, r, m) &&
              !mn::is_switch_cond_available(27, r),
          "i6s avail: with ext ADC the default VRG is a pot, not SWM");
    r.adc_modes[8] = static_cast<uint8_t>(in::AdcMode::TwoPos);
    r.adc_modes[9] = static_cast<uint8_t>(in::AdcMode::ThreePos);
    r.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::TwoPos);
    check(mn::is_switch_cond_available(23, r) && mn::is_switch_cond_available(26, r) &&
              mn::is_switch_cond_available(28, r) && mn::is_arm_switch_available(28, r),
          "i6s avail: conditions 23..28 with SWK..SWM as switches");
    check(mn::is_source_available(42, r, m) && mn::is_source_available(43, r, m) &&
              mn::is_source_available(44, r, m) && !mn::is_source_available(41, r, m) &&
              !mn::is_source_available(39, r, m),
          "i6s avail: sources SWK..SWM replace VRE..VRG");
    r.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::Off);
    check(!mn::is_switch_cond_available(27, r) && !mn::is_source_available(41, r, m) &&
              !mn::is_source_available(44, r, m),
          "i6s avail: an input set to Off is neither switch nor pot");
    check(mn::is_source_available(45, r, m) && mn::is_source_available(49, r, m) &&
              !mn::is_source_available(50, r, m),
          "i6s avail: GV1..GV5 are 45..49");
}

inline void i6s_radio_setup_tests() {
    namespace mn = wm::logic::menu;
    namespace st = wm::logic::storage;
    using mn::RadioItem;

    // flysky-i6s-rs setup.rs rows 0..7 and hw/mod.rs draw_board_setup_row 8..19.
    constexpr std::array<std::string_view, 20> kRustLabels{
        "Thr Trim:", "Beeper:",     "Tones:",   "BL Timer:", "BL Level:",   "Contrast:",  "Bat Warn:",
        "USB Mode:", "H1 PF6 Pwr:", "J4 Mode:", "Rear L:",   "Rear R:",     "Touch:",     "Stick Mode:",
        "Gimbals:",  "Sw Sound:",   "Inputs:",  "Ext ADC (PC):", "PPM Out:", "Trainer In:"};
    // Without USE_USB the "USB Mode:" row (Rust row 7) is dropped and later rows move up by one.
    const std::size_t skip = mn::kHasUsb ? 0 : 1;
    bool labels_ok = mn::kRadioRows == kRustLabels.size() - skip;
    for (std::size_t i = 0, row = 0; labels_ok && i < kRustLabels.size(); ++i) {
        if (!mn::kHasUsb && kRustLabels[i] == "USB Mode:") {
            continue;
        }
        labels_ok = mn::radio_label(mn::kRadioItems[row++]) == kRustLabels[i];
    }
    check(labels_ok, "i6s radio: Radio Setup rows in Rust order and labels");
    check(mn::kRadioInputRow == 16 - skip && mn::radio_row_of(RadioItem::Gimbals) == 14 - skip &&
              mn::radio_row_of(RadioItem::PpmOut) == 18 - skip && mn::radio_row_of(RadioItem::TrainerIn) == 19 - skip,
          "i6s radio: Inputs is row 16, PPM rows 18/19 (one less without USB)");

    st::RadioConfig r = st::default_radio_config();
    const auto value = [&](const RadioItem item) { return mn::radio_value(r, mn::radio_row_of(item)); };
    check(value(RadioItem::RearLeft).view() == "RC SW (SWE)" && value(RadioItem::RearRight).view() == "RC SW (SWF)" &&
              value(RadioItem::J4Mode).view() == "SWD (DBG)" && value(RadioItem::Touch).view() == "ENABLED" &&
              value(RadioItem::InputSetup).view() == "[SETUP]" && value(RadioItem::ExtAdc).view() == "DISABLED",
          "i6s radio: default row values match Rust");

    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::RearLeft));
    check(r.rear_left_func == 1 && value(RadioItem::RearLeft).view() == "TIMER RESET", "i6s radio: Rear L cycles");
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::RearLeft));
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::RearLeft));
    check(r.rear_left_func == 0, "i6s radio: Rear L wraps after TRIMS MOD");
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::RearRight));
    check(value(RadioItem::RearRight).view() == "INSTANT TRIM", "i6s radio: Rear R cycles");
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::J4Mode));
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::Touch));
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::SwitchSound));
    mn::cycle_radio_row(r, mn::radio_row_of(RadioItem::ExtAdc));
    check(r.j4_sg_sh_en == 1 && r.touch_disabled == 1 && r.switch_beep == 0 && r.ext_adc == 1 &&
              value(RadioItem::ExtAdc).view() == "ENABLED",
          "i6s radio: J4, Touch, Sw Sound and Ext ADC toggle");
    check(mn::input_rows(r) == 11 && mn::kAdcModeNames[1] == "OFF" && mn::kAdcModeNames[7] == "INST-TRIM" &&
              mn::kAuxInputNames[10] == "VRG (PC5):",
          "i6s radio: 11 input rows with VRG and 8 modes including OFF");
}

inline void i6s_jog_tests() {
    namespace jog = wm::logic::jog;
    namespace mn = wm::logic::menu;
    namespace in = wm::logic::input;
    namespace st = wm::logic::storage;
    using jog::Step;
    constexpr auto kDef = in::AdcMode::Default;

    jog::Wheel w{};
    check(w.update(true, kDef, 300, kDef, 0) == Step::None, "jog: 300 is still inside the deadband");
    check(w.update(true, kDef, 301, kDef, 0) == Step::Down, "jog: deflection past 300 steps down once");
    check(w.update(true, kDef, 1000, kDef, 0) == Step::None, "jog: holding the wheel does not repeat");
    check(w.update(true, kDef, 1000, kDef, -1000) == Step::None, "jog: VRB is ignored while VRA is deflected");
    check(w.update(true, kDef, 0, kDef, 0) == Step::None, "jog: centring re-arms");
    check(w.update(true, kDef, -500, kDef, 0) == Step::Up, "jog: deflection below -300 steps up");
    w.update(true, kDef, 0, kDef, 0);
    check(w.update(true, kDef, 0, in::AdcMode::PotDetent, 900) == Step::Down, "jog: VRB steps while VRA is centred");
    w.update(true, kDef, 0, kDef, 0);
    check(w.update(true, in::AdcMode::TwoPos, 1000, in::AdcMode::ThreePos, 1000) == Step::None,
          "jog: switch modes never step");
    check(w.update(true, in::AdcMode::Pot, 1000, kDef, 0) == Step::Down, "jog: pot mode steps");
    check(w.update(false, in::AdcMode::Pot, 1000, kDef, 0) == Step::None, "jog: inactive wheel never steps");
    check(w.update(true, in::AdcMode::Pot, 1000, kDef, 0) == Step::None,
          "jog: a wheel held through an inactive phase must be centred first (Rust keeps jog_deflection_dir)");

    // Menu integration: one Radio Setup row per deflection.
    st::RadioStorage s = st::default_storage();
    mn::State ms{};
    mn::open(ms);
    ms.screen = mn::Screen::RadioSetup;
    ms.radio = mn::ListPos{};
    uint32_t now = 0;
    const auto tick = [&](const int16_t vr1, const int16_t vr2) {
        mn::Input input{0, 0, now += 10};
        input.vr1 = vr1;
        input.vr2 = vr2;
        return mn::update(ms, input, s);
    };
    tick(0, 0);
    tick(800, 0);
    check(ms.radio.selected == 1, "jog menu: VRA deflection moves one row down");
    tick(800, 0);
    tick(800, 0);
    check(ms.radio.selected == 1, "jog menu: a held wheel does not scroll");
    tick(0, 0);
    tick(0, -800);
    check(ms.radio.selected == 0, "jog menu: VRB deflection moves one row up");

    // Source picking: a deflected pot selects VRA and does not move the list.
    s.radio.adc_modes[4] = static_cast<uint8_t>(in::AdcMode::Pot);
    ms.screen = mn::Screen::AuxChannels;
    ms.aux = mn::ListPos{};
    ms.editing = true;
    s.active_model.aux_channels[0] = 4;
    tick(0, 0);
    tick(900, 0);
    check(s.active_model.aux_channels[0] == mn::kSourceVra && ms.aux.selected == 0,
          "jog menu: a deflected VRA pot is picked as source instead of stepping");
}

inline void i6s_auto_select_tests() {
    namespace as = wm::logic::auto_select;
    namespace mn = wm::logic::menu;
    namespace in = wm::logic::input;
    namespace st = wm::logic::storage;
    using wm::logic::Switches;
    using wm::logic::SwitchPos;

    check(as::condition_change(Switches{.swm = SwitchPos::Up}, Switches{.swm = SwitchPos::Down}) == 27 &&
              as::condition_change(Switches{.swm = SwitchPos::Down}, Switches{}) == 28 &&
              as::condition_change(Switches{.swk = SwitchPos::Down}, Switches{}) == 24,
          "i6s auto: SWK/SWM changes report conditions 24/27/28");
    as::Tracker tracker{};
    as::Sample sample{};
    tracker.step(sample);
    sample.switches.swm = SwitchPos::Down;
    check(tracker.step(sample).sw_cond == 28, "i6s auto: tracker reports SWMv");

    st::RadioStorage s = st::default_storage();
    s.radio.ext_adc = 1;
    s.radio.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::TwoPos);
    mn::State ms{};
    mn::open(ms);
    ms.screen = mn::Screen::ModelSetup;
    ms.setup.selected = static_cast<uint8_t>(mn::SetupRow::ArmSwitch);
    ms.editing = true;
    s.active_model.arm_switch = 0;
    mn::update(ms, mn::Input{0, 0, 0}, s);
    mn::update(ms, mn::Input{0, 0, 1, as::Changes{.sw_cond = 28}}, s);
    check(s.active_model.arm_switch == 28, "i6s auto: arm switch follows SWMv");
    s.radio.adc_modes[10] = static_cast<uint8_t>(in::AdcMode::Pot);
    s.active_model.arm_switch = 0;
    mn::update(ms, mn::Input{0, 0, 2, as::Changes{.sw_cond = 27}}, s);
    check(s.active_model.arm_switch == 0, "i6s auto: SWM as pot is not an arm switch");
}

inline void i6s_tests() {
    i6s_storage_tests();
    i6s_input_tests();
    i6s_mixer_tests();
    i6s_availability_tests();
    i6s_radio_setup_tests();
    i6s_jog_tests();
    i6s_auto_select_tests();
}

#endif  // WM_PCB_I6S
