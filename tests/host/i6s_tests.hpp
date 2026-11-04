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

    check(sizeof(RadioConfig) == 148 && sizeof(st::RadioStorage) == 720 && sizeof(ModelConfig) == 572,
          "i6s storage: model appends MultiSwitch addresses, captions and four mode masks");
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

    s.active_model.multiswitch_addresses = {-2, -1, 255, 256};
    s.active_model.multiswitch_labels[3][7] = {'a', 0xFF, 'Z', '9', '-', '_'};
    st::sanitize(s);
    check(s.active_model.multiswitch_addresses == std::array<int16_t, 4>{-1, -1, 255, -1},
          "i6s sanitize: invalid MultiSwitch addresses become Off");
    check(s.active_model.multiswitch_labels[3][7] == st::MultiSwitchLabel{' ', ' ', 'Z', '9', '-', '_'},
          "i6s sanitize: captions retain supported characters and replace invalid bytes with spaces");
    using Dev = test_eeprom::FakeEeprom;
    auto old = st::to_bytes(st::default_model_config(17));
    old[366] = 255;
    old[362] = 12;
    old[363] = 13;
    const auto legacy_slot = *st::model_slot_addr(17);
    test_eeprom::write_slot_raw(legacy_slot, st::kSchemaVersion, 368, old);
    ModelConfig migrated{};
    check(st::load_slot<Dev>(legacy_slot, migrated) == st::Status::Ok &&
              migrated.multiswitch_addresses == std::array<int16_t, 4>{255, -1, -1, -1} &&
              migrated.multiswitch_labels == st::MultiSwitchLabels{} &&
              migrated.telem_slots == std::array<uint8_t, 4>{12, 13, 2, 3} &&
              test_eeprom::read_header(legacy_slot).len == sizeof(ModelConfig),
          "i6s storage: single-page models keep their address and telemetry while new pages default Off");
    migrated.multiswitch_addresses = {-1, 0, 42, 255};
    old = st::to_bytes(migrated);
    test_eeprom::write_slot_raw(legacy_slot, st::kSchemaVersion, 376, old);
    check(st::peek_model_name<Dev>(17) == migrated.name,
          "i6s storage: previous four-address model names remain discoverable before migration");
    check(st::load_slot<Dev>(legacy_slot, migrated) == st::Status::Ok &&
              migrated.multiswitch_addresses == std::array<int16_t, 4>{-1, 0, 42, 255} &&
              migrated.multiswitch_labels == st::MultiSwitchLabels{} &&
              test_eeprom::read_header(legacy_slot).len == sizeof(ModelConfig),
          "i6s storage: previous four-address models retain all addresses and gain empty captions");
    migrated.multiswitch_labels[0][0] = {'L', 'I', 'G', 'H', 'T', 'S'};
    migrated.multiswitch_labels[3][7] = {'H', 'O', 'R', 'N', 0, 0};
    old = st::to_bytes(migrated);
    test_eeprom::write_slot_raw(legacy_slot, st::kSchemaVersion, 568, old);
    check(st::peek_model_name<Dev>(17) == migrated.name,
          "i6s storage: caption-era model names remain discoverable before migration");
    ModelConfig caption_model{};
    check(st::load_slot<Dev>(legacy_slot, caption_model) == st::Status::Ok &&
              caption_model.multiswitch_addresses == migrated.multiswitch_addresses &&
              caption_model.multiswitch_labels == migrated.multiswitch_labels &&
              caption_model.multiswitch_momentary == st::MultiSwitchModes{} &&
              test_eeprom::read_header(legacy_slot).len == sizeof(ModelConfig),
          "i6s storage: caption-era models keep all settings and default every function to toggle");
    migrated.multiswitch_momentary = {0x81, 0x24, 0xFF, 0};
    check(st::save_model_to_slot<Dev>(18, migrated), "i6s storage: save all optional MultiSwitch addresses");
    ModelConfig restored{};
    check(st::load_slot<Dev>(*st::model_slot_addr(18), restored) == st::Status::Ok &&
              restored.multiswitch_addresses == migrated.multiswitch_addresses &&
              restored.multiswitch_labels == migrated.multiswitch_labels &&
              restored.multiswitch_momentary == migrated.multiswitch_momentary,
          "i6s storage: addresses, captions and modes survive reload even when MultiSwitch is compiled out");
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

#if WM_USE_MULTISWITCH
inline void i6s_multiswitch_tests() {
    namespace ms = wm::logic::multiswitch;
    namespace tn = wm::logic::touch_nav;
    namespace mn = wm::logic::menu;
    namespace st = wm::logic::storage;
    using tn::TouchEventKind;

    check(ms::build_set4(0, 0) == std::array<uint8_t, 12>{
              0xC8, 0x0A, 0x32, 0x00, 0xEA, 0xA1, 0x07, 0x00, 0x00, 0x00, 0x7E, 0xDE},
          "multiswitch: all OFF has command CRC BA and frame CRC D5");
    check(ms::build_set4(0, 1) == std::array<uint8_t, 12>{
              0xC8, 0x0A, 0x32, 0x00, 0xEA, 0xA1, 0x07, 0x00, 0x00, 0x01, 0xC4, 0x9A},
          "multiswitch: output 1 uses the low state bits");
    check(ms::build_set4(255, 0x80) == std::array<uint8_t, 12>{
              0xC8, 0x0A, 0x32, 0x00, 0xEA, 0xA1, 0x07, 0xFF, 0x40, 0x00, 0x1C, 0x1E},
          "multiswitch: output 8 uses high state bits, address accepts 255");
    check(ms::build_set4(42, 0xFF) == std::array<uint8_t, 12>{
              0xC8, 0x0A, 0x32, 0x00, 0xEA, 0xA1, 0x07, 0x2A, 0x55, 0x55, 0xFE, 0xC0},
          "multiswitch: all ON uses code 01 for every output in big-endian order");

    ms::State state{};
    check(state.due(0), "multiswitch: initial OFF state is sent immediately");
    state.sent(100);
    check(!state.due(199) && state.due(200), "multiswitch: states repeat every 100 ms");
    state.toggle(7);
    state.toggle(0);
    check(state.switches == 0x81 && state.due(101) && state.due(102),
          "multiswitch: changes stay pending until actually sent, preserving other switches");
    state.toggle(7);
    check(state.switches == 1, "multiswitch: second tap switches OFF without state 2");
    state.sent(UINT32_MAX - 49);
    check(!state.due(49) && state.due(50), "multiswitch: repeat timer handles millis wraparound");

    ms::Pages pages{};
    pages.configure({0, 1, 2, 255});
    for (uint8_t slot = 0; slot < ms::kMaxPages; ++slot) {
        check(pages.due(0) == slot && pages.due(1) == slot,
              "multiswitch pages: every active address is sent in order, failed sends remain pending");
        pages.sent(slot, 1);
        if (slot == 0) {
            pages.states[0].toggle(0);
        }
    }
    check(pages.due(2) == 0, "multiswitch pages: new changes do not starve other pending pages");
    pages.sent(0, 2);
    check(!pages.due(100) && pages.due(101) == 1, "multiswitch pages: every address has its own repeat deadline");
    pages.states[1].toggle(7);
    pages.states[2].toggle(3);
    pages.configure({0, 1, 42, -1});
    check(pages.states[0].switches == 1 && pages.states[1].switches == 0x80 &&
              pages.states[2].switches == 0 && pages.states[2].pending,
          "multiswitch pages: address changes reset only the affected page");
    for (int i = 0; i < 3; ++i) {
        const auto due = pages.due(200);
        check(due && *due != 3, "multiswitch pages: Off addresses are never transmitted");
        if (due) {
            pages.sent(*due, 200);
        }
    }
    pages.configure({-1, -1, -1, -1});
    check(!pages.due(1000), "multiswitch pages: all Off disables transmission entirely");

    tn::SwipeTracker tracker{};
    const auto sample = [](const uint8_t x, const uint8_t y, const TouchEventKind event) {
        return tn::TouchSample{.point = tn::TouchPoint{.x = x, .y = y, .event = event}};
    };
    const auto tick = [&](const std::optional<tn::TouchSample>& touch, const uint32_t now,
                          const bool menu_active = false) {
        return tn::update_inputs(touch, {}, menu_active, 0, 0, true, {}, tracker, now, true);
    };
    for (uint8_t i = 0; i < ms::kSwitchCount; ++i) {
        tracker = {};
        const auto r = ms::switch_rect(i);
        const auto x = static_cast<uint8_t>(r.pos.x + r.size.w / 2);
        const auto y = static_cast<uint8_t>(r.pos.y + r.size.h / 2);
        const auto down = tick(sample(x, y, TouchEventKind::PressDown), 10);
        const auto hold = tick(sample(x, y, TouchEventKind::Contact), 1200);
        check(down == tn::kKeyTouchActivity && hold == tn::kKeyTouchActivity && !tracker.tapped_switch,
              "multiswitch touch: pressing or holding a field neither switches nor trims nor opens menu");
        tick(sample(x, y, TouchEventKind::LiftUp), 1210);
        check(tracker.tapped_switch == i, "multiswitch touch: release selects the correct one of eight fields");
        tick(std::nullopt, 1260);
        check(!tracker.tapped_switch, "multiswitch touch: release toggles exactly once");
    }

    tick(sample(5, 20, TouchEventKind::PressDown), 2000);
    tick(std::nullopt, 2039);
    check(!tracker.tapped_switch, "multiswitch touch: missing packets do not release before timeout");
    tick(std::nullopt, 2040);
    check(tracker.tapped_switch == 0, "multiswitch touch: packet timeout also completes a tap");
    tick(sample(5, 20, TouchEventKind::Contact), 2050);
    tick(std::nullopt, 2100);
    check(!tracker.tapped_switch, "multiswitch touch: late contacts cannot repeat a timed-out tap");

    tick(sample(45, 20, TouchEventKind::PressDown), 2200);
    check((tick(sample(25, 20, TouchEventKind::Contact), 2210) & tn::kKeyNavDown) != 0,
          "multiswitch touch: a drag navigates to the next page");
    tick(sample(25, 20, TouchEventKind::LiftUp), 2220);
    check(!tracker.tapped_switch, "multiswitch touch: lifting after a swipe never toggles a field");
    tick(sample(5, 20, TouchEventKind::PressDown), 2300);
    tick(sample(12, 20, TouchEventKind::Contact), 2310);
    tick(sample(5, 20, TouchEventKind::LiftUp), 2320);
    check(!tracker.tapped_switch, "multiswitch touch: small drags cancel taps even after returning to start");
    tick(sample(5, 20, TouchEventKind::PressDown), 2400);
    tick(tn::TouchSample{.gesture = tn::Gesture::SwipeLeft}, 2410);
    check(!tracker.tapped_switch, "multiswitch touch: hardware gestures cancel switch taps");

    tick(sample(64, 5, TouchEventKind::PressDown), 2500);
    check((tick(sample(64, 5, TouchEventKind::LiftUp), 2510) & tn::kKeyNavDown) != 0,
          "multiswitch touch: header tap advances the dashboard");
    check((tick(sample(64, 58, TouchEventKind::PressDown), 2600) & tn::kKeyMenuOpen) != 0,
          "multiswitch touch: holding footer opens menu");
    tick(sample(64, 58, TouchEventKind::LiftUp), 2610);
    check(!tracker.tapped_switch, "multiswitch touch: footer never selects a switch");
    tick(sample(45, 20, TouchEventKind::PressDown), 2700, true);
    tick(sample(45, 20, TouchEventKind::LiftUp), 2710, true);
    check(!tracker.tapped_switch, "multiswitch touch: menu taps do not operate switches");

    auto storage = st::default_storage();
    mn::State menu{};
    menu.screen = mn::Screen::ModelSetup;
    menu.setup.selected = static_cast<uint8_t>(mn::SetupRow::MultiSwitch);
    menu.setup.scroll = 7;
    uint32_t now = 3000;
    auto edit_cue = wm::logic::buzzer::Cue::None;
    const auto press = [&](const uint16_t key) {
        const auto event = mn::update(menu, mn::Input{key, key, now++}, storage);
        edit_cue = menu.cue;
        mn::update(menu, mn::Input{0, 0, now++}, storage);
        return event;
    };
    const auto addresses_before = storage.active_model.multiswitch_addresses;
    check(press(mn::kKeyOk).action == mn::Action::None && menu.screen == mn::Screen::MultiSwitchSetup &&
              menu.multiswitch.selected == 0 && !menu.editing &&
              storage.active_model.multiswitch_addresses == addresses_before,
          "multiswitch setup: model setup opens the address sub-dialog without changing or saving addresses");
    press(mn::kKeyUp);
    check(menu.multiswitch.selected == 3, "multiswitch setup: navigation wraps from the first to the fourth address");
    press(mn::kKeyDown);
    check(menu.multiswitch.selected == 0, "multiswitch setup: navigation wraps back to the first address");
    wm::logic::display::Framebuffer setup_fb{};
    const mn::ModelNames names{};
    mn::render(setup_fb, menu, names, storage, mn::Live{});
    check(lit_pixels(setup_fb) > 0, "multiswitch setup: sub-dialog renders through the menu dispatcher");
    for (uint8_t slot = 0; slot < ms::kMaxPages; ++slot) {
        check(menu.multiswitch.selected == slot, "multiswitch setup: Down selects the next address row");
        auto& address = storage.active_model.multiswitch_addresses[slot];
        address = 0;
        check(press(mn::kKeyOk).action == mn::Action::None &&
                  menu.screen == mn::Screen::MultiSwitchPageSetup &&
                  menu.multiswitch_page.selected == 0 && !menu.editing,
              "multiswitch setup: each entry opens its own address and caption settings without saving");
        press(mn::kKeyOk);
        press(mn::kKeyDown);
        check(menu.editing && address == -1 && edit_cue == wm::logic::buzzer::Cue::ValueStep,
              "multiswitch setup: all four address rows step from zero to Off with a beep");
        press(mn::kKeyDown);
        check(address == -1 && edit_cue == wm::logic::buzzer::Cue::None,
              "multiswitch setup: no change or beep below Off");
        press(mn::kKeyUp);
        check(address == 0 && edit_cue == wm::logic::buzzer::Cue::ValueStep,
              "multiswitch setup: Up enables a page at address zero with a beep");
        address = 255;
        press(mn::kKeyUp);
        check(address == 255 && edit_cue == wm::logic::buzzer::Cue::None,
              "multiswitch setup: address edit clamps at 255 without a beep");
        press(mn::kKeyDown);
        check(address == 254 && edit_cue == wm::logic::buzzer::Cue::ValueStep,
              "multiswitch setup: decrement confirms with value beep");
        press(mn::kKeyUp | mn::kKeyDown);
        check(address == 254 && edit_cue == wm::logic::buzzer::Cue::None,
              "multiswitch setup: simultaneous Up and Down do not change the address");
        check(press(mn::kKeyOk).action == mn::Action::None && !menu.editing,
              "multiswitch setup: confirming an address does not write EEPROM");
        press(mn::kKeyDown);
        press(mn::kKeyOk);
        check(menu.screen == mn::Screen::MultiSwitchButtonSetup && !menu.editing,
              "multiswitch setup: OK on a function opens its text/type sub-dialog");
        press(mn::kKeyOk);
        auto& label = storage.active_model.multiswitch_labels[slot][0];
        if (slot == 0) {
            constexpr std::string_view alphabet{" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"};
            for (std::size_t i = 1; i < alphabet.size(); ++i) {
                check(press(mn::kKeyUp).action == mn::Action::None && label[0] == alphabet[i],
                      "multiswitch setup: caption editor cycles the supported alphabet without saving");
            }
            press(mn::kKeyUp);
            check(label[0] == ' ', "multiswitch setup: caption alphabet wraps to space");
            press(mn::kKeyDown);
            check(label[0] == '_', "multiswitch setup: reverse caption editing wraps to underscore");
            press(mn::kKeyUp);
            press(mn::kKeyUp | mn::kKeyDown);
            check(label[0] == ' ', "multiswitch setup: simultaneous Up and Down do not edit a caption");
        }
        for (std::size_t pos = 0; pos < st::kMultiSwitchLabelLen; ++pos) {
            for (uint8_t step = 0; step < (pos == 0 ? slot + 1 : 1); ++step) {
                press(mn::kKeyUp);
            }
            check(press(mn::kKeyOk).action == mn::Action::None,
                  "multiswitch setup: advancing or finishing the caption cursor never saves");
        }
        check(!menu.editing && label == st::MultiSwitchLabel{
                  static_cast<uint8_t>('A' + slot), 'A', 'A', 'A', 'A', 'A'},
              "multiswitch setup: all six characters are editable independently for each MS entry");
        press(mn::kKeyDown);
        press(mn::kKeyOk);
        check(press(mn::kKeyUp).action == mn::Action::None &&
                  storage.active_model.multiswitch_momentary[slot] == 1 &&
                  edit_cue == wm::logic::buzzer::Cue::ValueStep,
              "multiswitch setup: type edit enables momentary mode for only the selected entry/function");
        press(mn::kKeyUp | mn::kKeyDown);
        check(storage.active_model.multiswitch_momentary[slot] == 1,
              "multiswitch setup: simultaneous Up/Down does not change type");
        press(mn::kKeyDown);
        check(storage.active_model.multiswitch_momentary[slot] == 0,
              "multiswitch setup: type edit can return to toggle");
        press(mn::kKeyUp);
        check(press(mn::kKeyOk).action == mn::Action::None && !menu.editing,
              "multiswitch setup: confirming the type does not write EEPROM");
        check(press(mn::kKeyCancel).action == mn::Action::None &&
                  menu.screen == mn::Screen::MultiSwitchPageSetup && menu.multiswitch_page.selected == 1,
              "multiswitch setup: leaving function settings preserves its row and defers saving");
        check(press(mn::kKeyCancel).action == mn::Action::SaveActiveModel &&
                  menu.screen == mn::Screen::MultiSwitchSetup && menu.multiswitch.selected == slot,
              "multiswitch setup: leaving entry settings saves address and captions together and preserves focus");
        press(mn::kKeyDown);
    }
    press(mn::kKeyOk);
    check(press(mn::kKeyCancel).action == mn::Action::None &&
              menu.screen == mn::Screen::MultiSwitchSetup,
          "multiswitch setup: leaving unchanged entry settings does not write EEPROM");
    storage.active_model.multiswitch_addresses[0] = 0;
    const auto captions_before = storage.active_model.multiswitch_labels;
    press(mn::kKeyOk);
    press(mn::kKeyOk);
    press(mn::kKeyDown);
    check(press(mn::kKeyCancel).action == mn::Action::None && !menu.editing &&
              menu.screen == mn::Screen::MultiSwitchPageSetup &&
              storage.active_model.multiswitch_addresses[0] == -1 &&
              storage.active_model.multiswitch_labels == captions_before,
          "multiswitch setup: ESC ends address editing without saving; disabling a page keeps captions");
    press(mn::kKeyUp);
    check(menu.multiswitch_page.selected == 8 && menu.multiswitch_page.scroll > 0,
          "multiswitch setup: navigation wraps and scrolls to the eighth caption even when Off");
    press(mn::kKeyOk);
    press(mn::kKeyOk);
    press(mn::kKeyDown);
    for (unsigned i = 0; i < 5; ++i) {
        press(mn::kKeyOk);
    }
    press(mn::kKeyUp);
    setup_fb.clear();
    mn::render(setup_fb, menu, names, storage, mn::Live{});
    check(menu.editing && menu.name_cursor == 5 && lit_pixels(setup_fb) > 0,
          "multiswitch setup: the eighth function's final character renders with its edit cursor");
    check(press(mn::kKeyCancel).action == mn::Action::None && !menu.editing &&
              storage.active_model.multiswitch_labels[0][7] == st::MultiSwitchLabel{'_', ' ', ' ', ' ', ' ', 'A'},
          "multiswitch setup: ESC finishes caption editing without saving or undoing its text");
    press(mn::kKeyDown);
    press(mn::kKeyOk);
    press(mn::kKeyUp);
    check(storage.active_model.multiswitch_momentary[0] == 0x81,
          "multiswitch setup: the eighth function has its own type bit without changing the first");
    check(press(mn::kKeyCancel).action == mn::Action::None && !menu.editing,
          "multiswitch setup: ESC finishes type editing without saving");
    check(press(mn::kKeyCancel).action == mn::Action::None &&
              menu.screen == mn::Screen::MultiSwitchPageSetup,
          "multiswitch setup: ESC leaves button settings without saving");
    check(press(mn::kKeyCancel).action == mn::Action::SaveActiveModel &&
              menu.screen == mn::Screen::MultiSwitchSetup,
          "multiswitch setup: ESC saves all collected edits only when leaving the entry");
    check(press(mn::kKeyCancel).action == mn::Action::None && menu.screen == mn::Screen::ModelSetup &&
              menu.setup.selected == static_cast<uint8_t>(mn::SetupRow::MultiSwitch) && !menu.editing &&
              menu.setup.scroll <= menu.setup.selected &&
              menu.setup.selected < menu.setup.scroll + mn::kListVisible,
          "multiswitch setup: ESC returns to the visible MultiSwitch row in model setup");
    press(mn::kKeyOk);
    check(menu.screen == mn::Screen::MultiSwitchSetup && menu.multiswitch.selected == 0 && !menu.editing,
          "multiswitch setup: reopening starts at the first address");

    using Dev = test_eeprom::FakeEeprom;
    storage.active_model.multiswitch_addresses = {-1, 0, 42, 255};
    check(st::save_storage<Dev>(storage), "multiswitch storage: save model address");
    st::RadioStorage loaded{};
    st::load_storage<Dev>(loaded);
    check(loaded.active_model.multiswitch_addresses == storage.active_model.multiswitch_addresses &&
              loaded.active_model.multiswitch_labels == storage.active_model.multiswitch_labels &&
              loaded.active_model.multiswitch_momentary == storage.active_model.multiswitch_momentary,
          "multiswitch storage: all addresses, captions and types survive reload");
    check(wm::logic::model_copy::copy_model<Dev>(
              storage, wm::logic::model_copy::active_slot(storage), wm::logic::model_copy::DstSlot{1}),
          "multiswitch storage: copy model with address");
    st::ModelConfig copy{};
    check(st::load_slot<Dev>(*st::model_slot_addr(1), copy) == st::Status::Ok &&
              copy.multiswitch_addresses == storage.active_model.multiswitch_addresses &&
              copy.multiswitch_labels == storage.active_model.multiswitch_labels &&
              copy.multiswitch_momentary == storage.active_model.multiswitch_momentary,
          "multiswitch storage: copied model keeps all page addresses, captions and types");
    auto reset_storage = storage;
    mn::State reset_menu{};
    reset_menu.screen = mn::Screen::ModelSetup;
    reset_menu.setup.selected = static_cast<uint8_t>(mn::SetupRow::Reset);
    check(mn::update(reset_menu, mn::Input{mn::kKeyOk, mn::kKeyOk, now++}, reset_storage).action ==
                  mn::Action::ResetActiveModel &&
              reset_storage.active_model.multiswitch_labels == st::MultiSwitchLabels{} &&
              reset_storage.active_model.multiswitch_momentary == st::MultiSwitchModes{},
          "multiswitch setup: model reset clears captions and restores toggle mode");
    auto old = st::to_bytes(st::default_model_config(2));
    const auto address = *st::model_slot_addr(2);
    test_eeprom::write_slot_raw(address, st::kSchemaVersion, 364, old);
    check(st::load_slot<Dev>(address, copy) == st::Status::Ok &&
              copy.multiswitch_addresses == ms::Addresses{0, -1, -1, -1},
          "multiswitch storage: older models migrate with only the first page enabled at address zero");

    wm::logic::input::InputState input{};
    wm::logic::trim::Controller trims{};
    std::array<uint16_t, wm::logic::kNumChannels> channels{};
    wm::logic::dashboard::View view{input, storage, trims, channels, true};
    wm::logic::display::Framebuffer off{}, on{};
    storage.active_model.multiswitch_momentary.fill(0);
    wm::logic::dashboard::draw_multiswitch_page(off, view, 1, 5);
    view.multiswitch_states[1] = 1;
    wm::logic::dashboard::draw_multiswitch_page(on, view, 1, 5);
    check(off.data() != on.data(), "multiswitch dashboard: ON is visibly distinct from OFF");
    auto expected = off;
    constexpr std::array<std::string_view, 8> captions{"BAAAAA", "2", "3", "4", "5", "6", "7", "8"};
    for (uint8_t i = 0; i < ms::kSwitchCount; ++i) {
        const auto r = ms::switch_rect(i);
        expected.fill_rect(r, false);
        expected.draw_rect(r, true);
        expected.draw_str_4x6({r.pos.x + (30 - static_cast<int>(captions[i].size()) * 4) / 2, r.pos.y + 5},
                              captions[i], false);
    }
    check(off.data() == expected.data(),
          "multiswitch dashboard: buttons contain only centered configured captions or fallback digits, no ON/OFF");
    const auto first_button = ms::switch_rect(0);
    expected.fill_rect(first_button, true);
    expected.draw_str_4x6({first_button.pos.x + 3, first_button.pos.y + 5}, "BAAAAA", true);
    check(on.data() == expected.data(),
          "multiswitch dashboard: an active button inverts the same centered six-character caption");
    storage.active_model.multiswitch_momentary[1] = 1;
    wm::logic::display::Framebuffer momentary_off{}, momentary_on{};
    wm::logic::dashboard::draw_multiswitch_page(momentary_on, view, 1, 5);
    auto expected_momentary = expected;
    expected_momentary.draw_rect({{first_button.pos.x + 2, first_button.pos.y + 2}, {26, 13}}, false);
    check(momentary_on.data() == expected_momentary.data() && momentary_on.data() != on.data(),
          "multiswitch dashboard: momentary ON has a contrasting inner frame without obscuring six characters");
    view.multiswitch_states[1] = 0;
    wm::logic::dashboard::draw_multiswitch_page(momentary_off, view, 1, 5);
    expected_momentary = off;
    expected_momentary.draw_rect({{first_button.pos.x + 2, first_button.pos.y + 2}, {26, 13}}, true);
    check(momentary_off.data() == expected_momentary.data() && momentary_off.data() != off.data(),
          "multiswitch dashboard: momentary OFF is distinguishable by a double frame");
    check(ms::label_text(st::MultiSwitchLabel{' ', ' ', 0, ' ', 0, ' '}, 7).view() == "8" &&
              ms::label_text(st::MultiSwitchLabel{' ', 'A', 0, 'B', ' ', ' '}, 0).view() == "A B",
          "multiswitch captions: blank text falls back to its number, surrounding blanks are trimmed");
    view.crsf = false;
    off.clear();
    on.clear();
    wm::logic::dashboard::draw_multiswitch_page(on, view, 1, 5);
    view.multiswitch_states[1] = 0;
    wm::logic::dashboard::draw_multiswitch_page(off, view, 1, 5);
    check(off.data() == on.data(), "multiswitch dashboard: AFHDS shows CRSF requirement, not active fields");

    namespace dash = wm::logic::dashboard;
    dash::PageState dashboard{};
    for (unsigned mask = 0; mask < 16; ++mask) {
        auto& addresses = storage.active_model.multiswitch_addresses;
        addresses.fill(-1);
        uint8_t enabled = 0;
        for (uint8_t slot = 0; slot < ms::kMaxPages; ++slot) {
            if ((mask & (1u << slot)) != 0) {
                addresses[slot] = slot;
                check(ms::slot_at(addresses, enabled) == slot,
                      "multiswitch dashboard: enabled entries map consecutively without Off gaps");
                ++enabled;
            }
        }
        dashboard.page = 0;
        dash::configure_pages(dashboard, storage.active_model);
        check(dashboard.count == 5 + enabled && !ms::slot_at(addresses, enabled),
              "multiswitch dashboard: every combination exposes exactly the configured pages");
        dash::prev_page(dashboard);
        check(dashboard.page == dashboard.count - 1, "multiswitch dashboard: backward wrap reaches last active page");
        dash::next_page(dashboard);
        check(dashboard.page == 0, "multiswitch dashboard: forward wrap skips disabled pages");
        for (uint8_t page = 0; page < dashboard.count; ++page) {
            check(dashboard.page == page, "multiswitch dashboard: page navigation stays consecutive");
            dash::handle_keys(dashboard, dash::kKeyPageDown);
        }
        check(dashboard.page == 0, "multiswitch dashboard: a full cycle returns to P1");
    }

    storage.active_model.multiswitch_addresses = {-1, -1, -1, 255};
    dashboard.page = 8;
    dash::configure_pages(dashboard, storage.active_model);
    check(dashboard.count == 6 && dashboard.page == 5 && dash::page_label(view, 5).view() == "P6/6",
          "multiswitch dashboard: disabling pages clamps the selection and updates page totals");
    view.crsf = true;
    view.multiswitch_states[3] = 0x80;
    on.clear();
    off.clear();
    dash::render(on, dashboard, view);
    dash::draw_status_bar(off, storage, input.battery_mv, view, 0);
    dash::draw_multiswitch_page(off, view, 3, 5);
    check(on.data() == off.data(), "multiswitch dashboard: P6 shows the fourth entry when entries 1-3 are Off");
    storage.active_model.multiswitch_addresses.fill(-1);
    dash::configure_pages(dashboard, storage.active_model);
    check(dashboard.count == 5 && dashboard.page == 4 && dash::page_label(view, 4).view() == "P5/5",
          "multiswitch dashboard: all Off leaves exactly the five original pages");
}

inline void i6s_multiswitch_momentary_tests() {
    namespace ms = wm::logic::multiswitch;
    namespace tn = wm::logic::touch_nav;
    using Kind = tn::TouchEventKind;
    ms::Pages pages{};
    pages.configure({0, 1, -1, 255}, {0xFF, 1, 0xFF, 0x80});
    tn::SwipeTracker tracker{};
    const auto sample = [](const uint8_t x, const uint8_t y, const Kind event) {
        return tn::TouchSample{.point = tn::TouchPoint{.x = x, .y = y, .event = event}};
    };
    const auto tick = [&](const std::optional<tn::TouchSample>& touch, const uint32_t now,
                          const bool menu = false, const bool page = true) {
        const auto keys = tn::update_inputs(touch, {}, menu, 0, 0, true, {}, tracker, now, page, pages.momentary[0]);
        pages.update_touch(!menu && page ? std::optional<uint8_t>{0} : std::nullopt,
                           tracker.held_switch, tracker.tapped_switch);
        return keys;
    };
    for (uint8_t i = 0; i < ms::kSwitchCount; ++i) {
        tracker = {};
        const auto r = ms::switch_rect(i);
        const auto x = static_cast<uint8_t>(r.pos.x + r.size.w / 2);
        const auto y = static_cast<uint8_t>(r.pos.y + r.size.h / 2);
        const uint32_t now = 100 * i;
        tick(sample(x, y, Kind::PressDown), now);
        check(pages.states[0].switches == (1u << i) && pages.states[0].pending && !tracker.tapped_switch,
              "multiswitch momentary: every button turns ON at press, not at release");
        pages.sent(0, now);
        const auto keys = tick(tn::TouchSample{.gesture = tn::Gesture::SwipeLeft,
                                              .point = tn::TouchPoint{.x = x, .y = y, .event = Kind::Contact}}, now + 14);
        check(keys == tn::kKeyTouchActivity && pages.states[0].switches == (1u << i) &&
                  !pages.states[0].pending,
              "multiswitch momentary: stationary contact stays ON without repeated toggles or swipe navigation");
        tick(sample(x, y, Kind::LiftUp), now + 20);
        check(pages.states[0].switches == 0 && pages.states[0].pending && !tracker.tapped_switch,
              "multiswitch momentary: release queues OFF without generating a latching tap");
    }

    pages.configure({0, 1, -1, 255}, {0x81, 1, 0xFF, 0x80});
    tick(sample(16, 20, Kind::PressDown), 1000);
    check(tick(sample(48, 20, Kind::Contact), 1010) == tn::kKeyTouchActivity &&
              pages.states[0].switches == 0 && tracker.momentary_touch,
          "multiswitch momentary: leaving the button releases it without sliding onto another button or page");
    tick(sample(16, 20, Kind::Contact), 1020);
    check(pages.states[0].switches == 0, "multiswitch momentary: sliding back in does not reactivate");
    tick(sample(16, 20, Kind::LiftUp), 1030);
    tick(sample(16, 20, Kind::PressDown), 1040);
    check(pages.states[0].switches == 1, "multiswitch momentary: a new press reactivates after leaving");
    tick(std::nullopt, 1040 + tn::kReleaseTimeoutMs - 1);
    check(pages.states[0].switches == 1, "multiswitch momentary: a short missing packet does not flicker OFF");
    tick(std::nullopt, 1040 + tn::kReleaseTimeoutMs);
    check(pages.states[0].switches == 0 && !tracker.held_switch && !tracker.tapped_switch,
          "multiswitch momentary: touch timeout safely releases without latching");
    tick(sample(16, 20, Kind::Contact), 1090);
    check(pages.states[0].switches == 0, "multiswitch momentary: late contact after timeout cannot reactivate");
    for (const bool menu : {false, true}) {
        tracker = {};
        tick(sample(16, 20, Kind::PressDown), 1200);
        const auto keys = tick(sample(16, 20, Kind::Contact), 1210, menu, menu);
        check(keys == tn::kKeyTouchActivity && pages.states[0].switches == 0 &&
                  !tracker.held_switch && tracker.momentary_touch,
              "multiswitch momentary: leaving the dashboard or opening a menu releases and consumes the touch");
        tick(sample(16, 20, Kind::Contact), 1220);
        check(pages.states[0].switches == 0, "multiswitch momentary: returning to a page needs a fresh press");
        tick(sample(16, 20, Kind::LiftUp), 1230);
    }
    tick(sample(16, 20, Kind::PressDown), 1300);
    tick(tn::TouchSample{}, 1310);
    check(pages.states[0].switches == 0, "multiswitch momentary: an explicit no-contact packet releases immediately");
    tick(sample(48, 20, Kind::PressDown), 1400);
    check(pages.states[0].switches == 0, "multiswitch mixed modes: a toggle still waits for release");
    tick(sample(48, 20, Kind::LiftUp), 1410);
    check(pages.states[0].switches == 2, "multiswitch mixed modes: toggle latches on release");
    tick(sample(16, 20, Kind::PressDown), 1420);
    check(pages.states[0].switches == 3, "multiswitch mixed modes: pressing momentary preserves latched output");
    pages.update_touch(std::nullopt, tracker.held_switch, {});
    check(pages.states[0].switches == 2 && pages.states[0].pending,
          "multiswitch momentary: input inhibition releases momentary outputs but preserves latches");
    pages.update_touch(3, 7, {});
    check(pages.states[0].switches == 2 && pages.states[3].switches == 0x80,
          "multiswitch momentary: holding a function affects its configuration slot, not the dashboard ordinal");
    pages.update_touch(2, 0, {});
    check(pages.states[2].switches == 0 && pages.states[3].switches == 0,
          "multiswitch momentary: disabled entries cannot activate and other held entries are released");
    pages.states[0].set_switches(6);
    pages.configure({0, 1, -1, 255}, {0x83, 1, 0xFF, 0x80});
    check(pages.states[0].switches == 4 && pages.states[0].pending,
          "multiswitch modes: changing one function's type resets only that function to OFF");
    pages.states[0].set_switches(5);
    pages.configure({0, 1, -1, 255}, {0x82, 1, 0xFF, 0x80});
    check(pages.states[0].switches == 4, "multiswitch modes: changing momentary to toggle also resets it to OFF");
}
#endif

inline void i6s_tests() {
    i6s_storage_tests();
    i6s_input_tests();
    i6s_mixer_tests();
    i6s_availability_tests();
    i6s_radio_setup_tests();
    i6s_jog_tests();
    i6s_auto_select_tests();
#if WM_USE_MULTISWITCH
    i6s_multiswitch_tests();
    i6s_multiswitch_momentary_tests();
#endif
}

#endif  // WM_PCB_I6S
