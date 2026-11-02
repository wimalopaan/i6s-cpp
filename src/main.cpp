#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "wm/chip/mcu.hpp"
#include "wm/chip/watchdog.hpp"
#include "wm/hw/adc.hpp"
#include "wm/hw/buzzer.hpp"
#include "wm/hw/crsf.hpp"
#include "wm/hw/eeprom.hpp"
#include "wm/hw/gpio.hpp"
#include "wm/hw/keys.hpp"
#include "wm/hw/rf.hpp"
#include "wm/hw/st7567.hpp"
#include "wm/hw/trainer.hpp"
#include "wm/logic/dashboard.hpp"
#include "wm/logic/auto_select.hpp"
#include "wm/logic/backlight.hpp"
#include "wm/logic/bind.hpp"
#include "wm/logic/brr.hpp"
#include "wm/logic/buzzer.hpp"
#include "wm/logic/buzzer_inputs.hpp"
#include "wm/logic/calib.hpp"
#include "wm/logic/flight.hpp"
#include "wm/logic/framebuffer.hpp"
#include "wm/logic/input.hpp"
#include "wm/logic/menu.hpp"
#include "wm/logic/mixer.hpp"
#include "wm/logic/model_copy.hpp"
#include "wm/logic/power.hpp"
#include "wm/logic/protocol.hpp"
#include "wm/logic/safety.hpp"
#include "wm/logic/storage.hpp"
#include "wm/logic/sysinfo.hpp"
#include "wm/logic/flight_timer.hpp"
#include "wm/logic/trainer.hpp"
#include "wm/logic/trim.hpp"
#include "wm/time/systick.hpp"
#if WM_USE_USB
#include "wm/hw/usb.hpp"
#endif

namespace {

// Latest measurements, updated by the main loop (read by later UI/RF code).
volatile uint16_t g_battery_mv = 0;
volatile uint16_t g_keys = 0;
wm::logic::auto_select::Tracker g_auto_select{};

// Radio and active model settings loaded from the external EEPROM.
wm::logic::storage::RadioStorage g_storage{};

// Calibration and filter state derived from g_storage.radio.
wm::logic::input::Calibration g_calib{};

// Stick and pot endpoint capture, started from the main menu.
wm::logic::calib::Wizard g_calib_wizard{};

// Whether the ADC currently scans the four extra AD12-15 channels (Radio Setup "Ext ADC").
bool g_p7_enabled = false;

// Display content is rendered into this buffer and flushed to the ST7567.
wm::logic::display::Framebuffer g_fb{};

// RF radio init result, and the silence state last applied to the A7105. Empty until the first
// apply, so a CRSF boot really puts the chip to sleep (Rust starts unsilenced and silences it).
bool g_rf_ok = false;
wm::logic::dashboard::SessionStats g_rf_stats{};
std::optional<bool> g_rf_silenced{};
#if WM_USE_USB
// USB sim mode as last applied to the RF link, CRSF UART and PPM out.
bool g_sim_mode = false;
#endif
// Protocol the CRSF UART and A7105 mute were last applied for. Lets the loop follow edits to the active model.
wm::logic::protocol::Protocol g_link_protocol{};
// Trainer safety handover between the instructor sticks and the student frame.
wm::logic::trainer::Handover g_handover{};

wm::logic::input::BoardSwitches read_board_switches() {
    using wm::hw::Pin;
    using wm::hw::Port;
    return {.se_high = Pin<Port::C, 12>::read(), .sf_high = Pin<Port::C, 15>::read()};
}

wm::logic::input::InputState sample_inputs() {
    return wm::logic::input::process(wm::hw::adc::read_raw(), read_board_switches(), g_calib);
}

// Holding OK this long on the dashboard opens the menu.
constexpr uint32_t kMenuHoldMs = 1200;

// Model names shown in the model list, filled from EEPROM when the list is opened.
wm::logic::menu::ModelNames g_model_names{};

// EEPROM probe and boot-read result for diagnostics page 4; write_err is refreshed at render time.
wm::logic::storage::EepromStatus g_eeprom{};

// Version and git hash for the system info screen, passed in by CMake.
constexpr wm::logic::sysinfo::Build kBuild{{WM_FW_VERSION}, {WM_GIT_HASH}};

// Buzzer playback logic and the PWM output that follows it.
wm::logic::buzzer::Player g_buzzer{};
wm::hw::buzzer::Output g_buzzer_out{};

// Battery and RSSI alarm cadences. Only stepped while the dashboard is shown, as in Rust.
wm::logic::buzzer::BatteryWarnTimer g_battery_warn{};
wm::logic::buzzer::RssiWarnTimer g_rssi_warn{};

// Trim key state machine. Values come from the active model; throttle_enabled from the radio setup.
wm::logic::trim::Controller g_trims{};
// Trim auto-save trigger; decides when g_trims is written to the active model's EEPROM slot.
wm::logic::trim::AutoSave g_trim_save{};
// Rising edge of an InstantTrim aux switch; tracked even while a menu is open, as in Rust.
wm::logic::trim::InstantEdge g_instant_edge{};

// Flight timer state machine; runs on the active model and resets when the model changes.
wm::logic::flight_timer::State g_flight_timer{};
// BIND key hold timer; starts and stops AFHDS 2A binding on the dashboard.
wm::logic::bind::Controller g_bind{};

wm::logic::flight_timer::Config timer_config(const wm::logic::storage::ModelConfig& m) {
    return {.limit = {m.timer_secs}, .source = {m.timer_source}, .arm = {m.arm_switch}};
}

void load_trims_from_model() {
    const auto& t = g_storage.active_model.trims;
    g_trims.values = {t[0], t[1], t[2], t[3]};
    g_trim_save.reset(g_trims.values);
}

void store_trims_in_model() {
    g_storage.active_model.trims = {g_trims.values.roll, g_trims.values.pitch, g_trims.values.throttle,
                                    g_trims.values.yaw};
}

// Plays a cue in the stored tone style. Cue::None is a no-op.
void play_buzzer_cue(const wm::logic::buzzer::Cue cue) {
    g_buzzer.play_cue(cue, wm::logic::buzzer::tone_style_from_u8(g_storage.radio.tone_style));
}

// Advances the buzzer and its output during the blocking boot screens, where the main loop does not run.
void pump_buzzer(uint32_t& last_ms) {
    const uint32_t now = wm::time::millis();
    g_buzzer.tick(wm::logic::buzzer::Millis{static_cast<uint16_t>(std::min<uint32_t>(now - last_ms, 20))});
    last_ms = now;
    g_buzzer_out.sync(g_buzzer.output());
}

// TX ID is the XOR of the three 32-bit words of the MCU UID, as in the Rust firmware.
wm::logic::afhds2a::TxId tx_id_from_uid() {
    using namespace wm;
    const auto uid = chip::read_uid(chip::kProfile);
    auto word = [&uid](const std::size_t i) {
        return static_cast<uint32_t>(uid[i]) | (static_cast<uint32_t>(uid[i + 1]) << 8) |
               (static_cast<uint32_t>(uid[i + 2]) << 16) | (static_cast<uint32_t>(uid[i + 3]) << 24);
    };
    return wm::logic::afhds2a::TxId{word(0) ^ word(4) ^ word(8)};
}

// Silences the A7105 for CRSF models. Only changes state when the protocol changes.
void set_rf_silenced(const bool silenced) {
    if (!g_rf_ok || g_rf_silenced == silenced) {
        return;
    }
    wm::hw::rf::set_silenced(silenced);
    g_rf_silenced = silenced;
}

// Rust `is_sim_mode`: the joystick is connected. Always false without USB.
bool usb_sim_active() {
#if WM_USE_USB
    return wm::hw::usb::is_sim_mode();
#else
    return false;
#endif
}

// Sim state last applied to the RF link. Constant false without USB, so the no-USB build keeps its code.
bool sim_applied() {
#if WM_USE_USB
    return g_sim_mode;
#else
    return false;
#endif
}

void set_sim_applied(const bool sim) {
#if WM_USE_USB
    g_sim_mode = sim;
#else
    (void)sim;
#endif
}

// Applies power, RX output settings and the silence state of the active model. Keeps the RX ID.
// Sim mode also silences the A7105 (Rust `rf::set_silenced(sim_mode)`).
void apply_rf_link() {
    using namespace wm;
    if (!g_rf_ok) {
        return;
    }
    const auto& model = g_storage.active_model;
    hw::rf::set_rf_power(model.rf_power);
    hw::rf::set_rx_settings(model.servo_rate_hz, {model.rx_out_mode == 1}, {model.rx_serial_proto == 1});
    set_rf_silenced(logic::protocol::link_for(logic::protocol::from_raw(model.rf_protocol)).rf_silenced ||
                    sim_applied());
}

// Full RF setup for the active model, used at boot and on model switch.
void apply_rf_model() {
    if (!g_rf_ok) {
        return;
    }
    wm::hw::rf::set_rx_id(wm::logic::afhds2a::RxId{g_storage.active_model.rx_id});
    apply_rf_link();
}

// Applies the RF protocol of the active model and the USB sim state: CRSF UART on or off, and the A7105 mute.
// Idempotent. Sim mode keeps the CRSF UART off (Rust `crsf::set_enabled(!sim_mode)`).
void apply_protocol_link() {
    using namespace wm;
    const auto& model = g_storage.active_model;
    const auto protocol = logic::protocol::from_raw(model.rf_protocol);
    g_link_protocol = protocol;
    set_sim_applied(usb_sim_active());
    if (logic::protocol::link_for(protocol).crsf_uart && !sim_applied()) {
        const uint8_t baud_idx =
            model.crsf_baud < logic::kBaudRates.size() ? model.crsf_baud : logic::kDefaultBaudIndex;
        hw::crsf::set_enabled(true, baud_idx,
                              model.crsf_half_duplex != 0 ? hw::uart::Duplex::Half : hw::uart::Duplex::Full);
    } else {
        hw::crsf::set_enabled(false, 0, hw::uart::Duplex::Full);
    }
    apply_rf_link();
}

void refresh_model_names() {
    using namespace wm;
    for (std::size_t i = 0; i < logic::storage::kNumModels; ++i) {
        g_model_names[i].fill(' ');
        logic::storage::ModelConfig model{};
        const auto addr = logic::storage::model_slot_addr(i);
        if (addr && logic::storage::load_slot<hw::I2c2Eeprom>(*addr, model) == logic::storage::Status::Ok) {
            g_model_names[i] = model.name;
        }
        chip::watchdog::feed();
    }
}

// Applies the stored LCD backlight level and contrast.
void apply_lcd_settings(const wm::logic::storage::RadioConfig& r) {
    wm::hw::lcd::set_backlight_level(wm::logic::backlight::on_level(r));
    wm::hw::lcd::set_contrast(r.lcd_contrast);
}

// Starts or stops the trainer PPM in and out from the stored radio config.
void apply_trainer_config(const wm::logic::storage::RadioConfig& r) {
    using namespace wm::logic::menu;
    wm::hw::trainer::set_enabled((r.trainer_config & kTrainerPpmIn) != 0,
                                     (r.trainer_config & kTrainerPpmOut) != 0);
}

// Stores the current model, then loads the model at `index` as the active one.
void switch_model(const uint8_t index) {
    using namespace wm::logic::storage;
    using Dev = wm::hw::I2c2Eeprom;
    store_trims_in_model();
    save_active_model<Dev>(g_storage);
    g_storage.radio.active_model = index;
    save_radio_config<Dev>(g_storage);
    load_active_model<Dev>(g_storage);
    load_trims_from_model();
}

// Blocks until sticks and switches are safe (or the cancel key is pressed), sending failsafe pulses meanwhile.
void run_preflight(const wm::logic::storage::ModelConfig& model) {
    using namespace wm;
    using namespace logic;
    std::array<safety::ChannelCalibration, 4> cals{};
    for (std::size_t i = 0; i < cals.size(); ++i) {
        cals[i] = safety::ChannelCalibration::from(g_storage.radio.sticks[i]);
    }
    const auto sm = safety::stick_mode_from(g_storage.radio.stick_mode);
    const auto mt = static_cast<mixer::ModelType>(model.model_type);
    buzzer::PreflightTimer preflight{};
    buzzer::PreflightWarning warning{};
    uint32_t buzz_ms = time::millis();

    for (;;) {
        chip::watchdog::feed();

        const auto in = sample_inputs();
        const std::array<uint16_t, 4> raw{in.raw[0], in.raw[1], in.raw[2], in.raw[3]};
        const bool hold = in.switches.sa == SwitchPos::Down;
        const bool ready = safety::check_preflight_throttle(raw, cals, sm, mt, hold) == safety::Preflight::Ok &&
                           safety::switches_safe(in.switches, mt);
        g_keys = hw::keys::scan_keys();
        if (ready || (g_keys & safety::kKeyCancel) != 0) {
            play_buzzer_cue(warning.exit_cue());
            return;
        }
        warning.unsafe();

        hw::crsf::update_channels(time::millis(), safety::kSafeIdleChannels,
                                  wm::logic::crsf::ArmModeSwitch{model.crsf_arm_mode != 0},
                                  wm::logic::crsf::ArmedState{false});
        hw::crsf::poll_telemetry(time::millis());
        if (preflight.step(buzzer::Millis{time::millis()})) {
            play_buzzer_cue(buzzer::Cue::WarnPreflight);
        }
        pump_buzzer(buzz_ms);
        __WFI();
    }
}

}  // namespace

int main() {
    using namespace wm;

    chip::watchdog::feed();

    const bool was_watchdog_reset = chip::check_and_clear_reset_flags();
    if (!was_watchdog_reset) {
        hw::keys::check_dfu_entry(chip::kProfile);
    } else {
        hw::keys::init_keys();
    }

    chip::init_system_clock();
    time::init();

    chip::watchdog::start();

    g_eeprom.detected = hw::I2c2Eeprom::init();
    if (g_eeprom.detected) {
        g_eeprom.addr = hw::I2c2Eeprom::probed_addr();
        g_eeprom.read_ok = logic::storage::load_storage<hw::I2c2Eeprom>(g_storage);
    } else {
        g_storage = logic::storage::default_storage();
    }

    hw::buzzer::init();

    g_p7_enabled = g_storage.radio.ext_adc != 0;
    hw::adc::init(g_p7_enabled);
    chip::watchdog::feed();

    hw::adc::wait_first_conversion();

    logic::input::apply_calibration(g_storage.radio, g_calib);
    // Prime the jitter and battery filters before the first real sample.
    for (int i = 0; i < 16; ++i) {
        (void)sample_inputs();
    }

    const auto& model = g_storage.active_model;
    load_trims_from_model();
    g_trims.throttle_enabled = g_storage.radio.throttle_trim != 0;
    // PC13 module power switch: configured (power OFF) before apply_protocol_link() may turn it on.
    hw::uart::init_power_switch(g_storage.radio.ext_module_pwr == 0);
    apply_protocol_link();

    const uint16_t boot_keys = hw::keys::scan_keys();
    const bool bind_on_boot = !was_watchdog_reset && (boot_keys & logic::kKeyBind) != 0;
    // OK held at power-on starts the calibration wizard directly.
    const bool calib_on_boot = !was_watchdog_reset && (boot_keys & logic::kKeyOk) != 0;
    g_rf_ok = hw::rf::init(tx_id_from_uid());
    apply_rf_model();
    if (bind_on_boot && g_rf_ok && model.rf_protocol == 0) {
        hw::rf::set_bind_mode(true);
    }

    hw::lcd::init();
    apply_lcd_settings(g_storage.radio);
#if WM_USE_USB
    if (const auto usb_mode = logic::usb::mode_from_stored(g_storage.radio.usb_mode);
        usb_mode != logic::usb::UsbMode::Off) {
        hw::usb::start(usb_mode);
    }
#endif

    hw::trainer::init();
    apply_trainer_config(g_storage.radio);

    using namespace logic::dashboard;
    PageState page_state{};
    g_buzzer.set_enabled(g_storage.radio.audio_enabled != 0);
    if (was_watchdog_reset) {
        // Resume immediately after a watchdog reset: no splash, only the warning chirp.
        play_buzzer_cue(logic::buzzer::Cue::WatchdogRecover);
    } else {
        g_fb.clear();
        draw_splash(g_fb);
        hw::lcd::flush(g_fb);
        play_buzzer_cue(logic::buzzer::Cue::Welcome);
    }
    uint32_t buzz_ms = time::millis();
    if (calib_on_boot) {
        logic::calib::start(g_calib_wizard);
        play_buzzer_cue(g_calib_wizard.cue);
    }
    if (!was_watchdog_reset && !logic::calib::active(g_calib_wizard)) {
        const uint32_t splash_start_ms = time::millis();
        while (time::millis() - splash_start_ms < 1200) {
            chip::watchdog::feed();
            pump_buzzer(buzz_ms);
            __WFI();
        }
        run_preflight(model);
    }

    uint32_t last_draw_ms = 0;
    uint16_t prev_keys = calib_on_boot ? boot_keys : 0;  // the held OK must not step the wizard
    logic::menu::State menu_state{};
    bool ok_armed = false;
    uint32_t ok_press_ms = 0;
    uint32_t last_loop_ms = time::millis();
    bool rx_id_dirty = false;
    const auto boot_input = sample_inputs();
    // Arm state as in Rust: only an assigned arm switch can arm. flight::is_armed (true without a switch)
    // is meant for the timer trigger, not for save inhibits, chimes or the CRSF arm flag.
    bool prev_armed = logic::mixer::eval_arm_switch(model.arm_switch, boot_input.switches);
    logic::buzzer_inputs::ActivityTracker activity{};
    activity.prime(boot_input.raw, boot_input.switches);
    logic::buzzer_inputs::PotCenterTracker<6> pot_centre{};
    logic::buzzer::InactivityTimer inactivity{};
    logic::backlight::Dimmer dimmer{};
    logic::vario::State vario{};

    for (;;) {
        chip::watchdog::feed();

        const auto input = sample_inputs();
        const auto stick_mode = logic::safety::stick_mode_from(g_storage.radio.stick_mode);
        const auto auto_changes = g_auto_select.step({input.switches, input.pots.vr1, input.pots.vr2});
        g_battery_mv = input.battery_mv;
        g_keys = hw::keys::scan_keys();

        const uint16_t pressed = static_cast<uint16_t>(g_keys & ~prev_keys);
        prev_keys = g_keys;
        const uint32_t now = time::millis();
        // Clamp as in Rust: long iterations (EEPROM writes, LCD flush) still advance the timers.
        const auto dt_ms = static_cast<uint16_t>(std::min<uint32_t>(now - last_loop_ms, 100));
        last_loop_ms = now;

        g_buzzer.set_enabled(g_storage.radio.audio_enabled != 0);
        g_buzzer.tick(logic::buzzer::Millis{dt_ms});

        const bool instant_rising =
            g_instant_edge.update(logic::trim::instant_switch_down(g_storage.radio, input.raw));

        if (logic::protocol::from_raw(g_storage.active_model.rf_protocol) != g_link_protocol ||
            usb_sim_active() != sim_applied()) {
            apply_protocol_link();
        }
        const bool sim = sim_applied();
        const bool afhds = g_rf_ok && g_storage.active_model.rf_protocol == 0;

        if (logic::calib::active(g_calib_wizard)) {
            const logic::calib::Input cal_in{pressed, g_keys, input.raw, dt_ms};
            if (logic::calib::update(g_calib_wizard, cal_in, g_storage.radio)) {
                logic::input::apply_calibration(g_storage.radio, g_calib);
                logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
            }
            play_buzzer_cue(g_calib_wizard.cue);
        } else if (menu_state.screen == logic::menu::Screen::Closed) {
            const uint8_t prev_page = page_state.page;
            handle_keys(page_state, pressed);
            if (page_state.page != prev_page) {
                play_buzzer_cue(logic::buzzer::Cue::NavStep);
            }
            if ((g_keys & logic::kKeyOk) == 0) {
                ok_armed = true;
            } else if ((pressed & logic::kKeyOk) != 0) {
                ok_press_ms = now;
            }
            if (ok_armed && (g_keys & logic::kKeyOk) != 0 && now - ok_press_ms >= kMenuHoldMs) {
                logic::menu::open(menu_state);
                play_buzzer_cue(logic::buzzer::Cue::Click);
                ok_armed = false;
            }
        } else {
            const auto ev = logic::menu::update(menu_state,
                                                logic::menu::Input{pressed, g_keys, now, auto_changes,
                                                                   &hw::crsf::menu_loader(), &hw::crsf::menu_command(),
                                                                   &hw::crsf::menu_devices()},
                                                g_storage);
            if (menu_state.nav_tick) {
                play_buzzer_cue(logic::buzzer::Cue::NavStep);
            }
            play_buzzer_cue(menu_state.cue);
            switch (ev.action) {
                case logic::menu::Action::EnterModelSelect: refresh_model_names(); break;
                case logic::menu::Action::SwitchModel:
                    switch_model(ev.index);
                    apply_protocol_link();
                    apply_rf_model();
                    prev_armed = logic::mixer::eval_arm_switch(g_storage.active_model.arm_switch, input.switches);
                    vario.reset();
                    break;
                case logic::menu::Action::SaveActiveModel:
                    logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
                    apply_protocol_link();
                    break;
                case logic::menu::Action::ResetActiveModel:
                    load_trims_from_model();
                    logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
                    apply_protocol_link();
                    apply_rf_model();
                    break;
                case logic::menu::Action::SaveRadioConfig:
                    logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
                    logic::input::apply_calibration(g_storage.radio, g_calib);
                    if ((g_storage.radio.ext_adc != 0) != g_p7_enabled) {
                        g_p7_enabled = g_storage.radio.ext_adc != 0;
                        hw::adc::set_p7_enabled(g_p7_enabled);
                    }
                    apply_lcd_settings(g_storage.radio);
                    apply_trainer_config(g_storage.radio);
                    hw::uart::set_power_polarity(g_storage.radio.ext_module_pwr == 0);
                    g_trims.throttle_enabled = g_storage.radio.throttle_trim != 0;
#if WM_USE_USB
                    hw::usb::set_mode(logic::usb::mode_from_stored(g_storage.radio.usb_mode));
#endif
                    break;
                case logic::menu::Action::CopyActiveModel:
                    logic::model_copy::copy_model<hw::I2c2Eeprom>(
                        g_storage, logic::model_copy::active_slot(g_storage), logic::model_copy::DstSlot{ev.index});
                    break;
                case logic::menu::Action::StartCalibration:
                    logic::calib::start(g_calib_wizard);
                    play_buzzer_cue(g_calib_wizard.cue);
                    break;
                case logic::menu::Action::CloseMenu:
                    play_buzzer_cue(logic::buzzer::Cue::Click);
                    break;
                case logic::menu::Action::StartBind:
                    if (g_rf_ok && afhds) {
                        hw::rf::set_bind_mode(true);
                    }
                    break;
                case logic::menu::Action::CrsfRequest: hw::crsf::menu_apply(menu_state.crsf_req, now); break;
                case logic::menu::Action::None: break;
            }
        }

        const bool armed = logic::mixer::eval_arm_switch(model.arm_switch, input.switches);
        if (g_rf_ok) {
            if (const auto id = hw::rf::take_pending_rx_save()) {
                if (id->value != 0 && id->value != 0xFFFF'FFFFu && model.rx_id != id->value) {
                    g_storage.active_model.rx_id = id->value;
                    rx_id_dirty = true;
                }
            }
        }
        if (rx_id_dirty && !armed) {
            if (logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage)) {
                play_buzzer_cue(logic::buzzer::Cue::RxSave);
                rx_id_dirty = false;
            }
        }

        const bool menu_open = menu_state.screen != logic::menu::Screen::Closed || logic::calib::active(g_calib_wizard);
        const auto bind_ev = logic::bind::update(
            g_bind, logic::bind::Input{
                        .link = {afhds, hw::rf::is_binding()},
                        .menu_active = menu_open,
                        .keys = g_keys,
                        .pressed = pressed,
                        .dt = logic::vario::Millis{dt_ms},
                    });
        switch (bind_ev.action) {
            case logic::bind::Action::Start: hw::rf::set_bind_mode(true); break;
            case logic::bind::Action::Stop: hw::rf::set_bind_mode(false); break;
            case logic::bind::Action::None: break;
        }
        play_buzzer_cue(bind_ev.cue);

        if (!menu_open) {
            const auto trim_ev = g_trims.update(g_keys, logic::vario::Millis{dt_ms});
            switch (trim_ev.kind) {
                case logic::trim::Event::Kind::Step:
                    g_buzzer.play_trim(logic::buzzer::TrimStep{trim_ev.value});
                    break;
                case logic::trim::Event::Kind::Center: play_buzzer_cue(logic::buzzer::Cue::TrimCenter); break;
                case logic::trim::Event::Kind::Limit: play_buzzer_cue(logic::buzzer::Cue::TrimLimit); break;
                case logic::trim::Event::Kind::None: break;
            }
            if (instant_rising) {
                const auto sticks = logic::flight::flight_controls(input.gimbals, stick_mode);
                const auto result = g_trims.instant_trim(logic::trim::InstantSticks{
                    .roll = sticks.aileron, .pitch = sticks.elevator, .yaw = sticks.rudder});
                switch (result) {
                    case logic::trim::InstantResult::Adjusted: play_buzzer_cue(logic::buzzer::Cue::InstantTrim); break;
                    case logic::trim::InstantResult::Reset: play_buzzer_cue(logic::buzzer::Cue::TrimCenter); break;
                    case logic::trim::InstantResult::None: break;
                }
            }
        }

        if (g_trim_save.tick(g_trims.values, prev_armed, logic::vario::Millis{dt_ms})) {
            store_trims_in_model();
            g_trim_save.saved(logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage));
        }

        const auto timer_cue = logic::flight_timer::update(
            g_flight_timer,
            logic::flight_timer::Input{
                .model = {g_storage.radio.active_model},
                .config = timer_config(model),
                .was_armed = prev_armed,
                .menu_active = menu_open,
                .cancel_held = (g_keys & logic::safety::kKeyCancel) != 0,
                .throttle = logic::flight::flight_controls(input.gimbals, stick_mode).throttle,
                .switches = input.switches,
                .dt = logic::vario::Millis{dt_ms},
            });
        play_buzzer_cue(timer_cue);

        auto channels =
            logic::flight::rf_channels(input, stick_mode, model, g_trims.values, g_storage.radio.throttle_trim);
        const bool trainer_in = (g_storage.radio.trainer_config & logic::menu::kTrainerPpmIn) != 0;
        const bool trainer_out = (g_storage.radio.trainer_config & logic::menu::kTrainerPpmOut) != 0;
        hw::trainer::tick(logic::trainer::DeltaMs{dt_ms});
        if (trainer_in) {
            channels = g_handover.process(input.switches.sd == logic::SwitchPos::Down,
                                          hw::trainer::student_channels(), channels);
        }
        if (trainer_out && !sim) {
            hw::trainer::publish_out(channels);
        }
        const bool crsf = model.rf_protocol == 1;
#if WM_USE_USB
        hw::usb::poll(logic::usb::Millis{now}, channels, input.switches, input.battery_mv);
#endif
        if (crsf) {
            if (!sim) {
                hw::crsf::update_channels(now, channels, wm::logic::crsf::ArmModeSwitch{model.crsf_arm_mode != 0},
                                          wm::logic::crsf::ArmedState{armed});
            }
        } else if (afhds && !sim) {
            hw::rf::publish_channels(channels);
        }
        if (!sim) {
            hw::crsf::poll_telemetry(now);
        }

        // Link telemetry of the active protocol: CRSF link quality stands in for the AFHDS 2A RSSI.
        const auto link_telem = crsf    ? logic::telemetry::from_crsf(hw::crsf::telemetry())
                                : g_rf_ok ? hw::rf::get_telemetry()
                                          : logic::afhds2a::Telemetry{};

        if (now - last_draw_ms >= 100) {
            g_fb.clear();
            const bool rf_binding = logic::bind::binding_shown({afhds, hw::rf::is_binding()});
            const bool rf_bound = g_rf_ok && hw::rf::is_bound();
            const uint32_t rf_sent = g_rf_ok ? hw::rf::packets_sent() : 0u;
            logic::dashboard::track_session(g_rf_stats, link_telem);
            if (logic::calib::active(g_calib_wizard)) {
                logic::calib::render(g_fb, g_calib_wizard, input.raw, g_storage.radio, model.model_type);
            } else if (menu_state.screen == logic::menu::Screen::Closed) {
                render(g_fb, page_state,
                       View{input, g_storage, g_trims, channels, crsf, link_telem, rf_binding, g_rf_ok, rf_bound, now,
                            rf_sent, g_rf_stats,
                            logic::flight_timer::display(g_flight_timer, timer_config(model)),
                            logic::flight_timer::modal(g_flight_timer), hw::crsf::telemetry()});
            } else {
                auto eeprom = g_eeprom;
                eeprom.write_err = hw::I2c2Eeprom::write_failed();
                logic::menu::render(g_fb, menu_state, g_model_names, g_storage,
                                    logic::menu::Live{channels, input.raw, input.switches, eeprom,
                                                      chip::kProfile.name, kBuild, &hw::crsf::menu_loader(),
                                                      &hw::crsf::menu_command(), &hw::crsf::menu_devices()});
            }
            hw::lcd::flush(g_fb);
            last_draw_ms = now;
        }

        play_buzzer_cue(logic::buzzer::arm_cue(logic::buzzer::ArmEdge{
            .arm_switch = logic::buzzer::SwitchIndex{model.arm_switch},
            .is_armed = armed,
            .was_armed = prev_armed,
            .menu_active = menu_open,
        }));
        prev_armed = armed;

        const bool user_active = activity.step(input.raw, input.switches, g_keys);
        if (const auto level = dimmer.step(user_active, g_storage.radio, dt_ms)) {
            hw::lcd::set_backlight_level(*level);
        }
        const std::size_t pot_count = g_storage.radio.ext_adc != 0 ? 6 : 2;
        if (pot_centre.step(std::span<const int16_t>(input.aux_pots).subspan(4, pot_count))) {
            play_buzzer_cue(logic::buzzer::Cue::PotCenter);
        }
        if (inactivity.step(user_active, logic::buzzer::Millis{dt_ms})) {
            play_buzzer_cue(logic::buzzer::Cue::WarnInactivity);
        }

        // The Vario Setup screen previews the tone regardless of the vario switch, as in Rust.
        const bool vario_preview = menu_state.screen == logic::menu::Screen::VarioSetup;
        if (menu_open && !vario_preview) {
            g_buzzer.set_vario(logic::vario::kSilent);
        } else {
            const auto vario_telem = afhds ? hw::rf::get_telemetry() : logic::afhds2a::Telemetry{};
            const auto crsf_telem = hw::crsf::telemetry();
            const bool crsf_link = logic::protocol::from_raw(model.rf_protocol) == logic::protocol::Protocol::Crsf;
            const logic::vario::Sample sample{
                .now = {now},
                .alt_m = {crsf_link ? crsf_telem.baro_alt_m : vario_telem.alt_m},
                .alt_dm = {crsf_link ? crsf_telem.baro_alt_dm : vario_telem.alt_dm},
                .vspeed = {crsf_link ? crsf_telem.vspeed_cms : vario_telem.vspeed_cms},
                .has_native_vspeed = crsf_link ? crsf_telem.has_vspeed : vario_telem.has_vspeed,
                .telem_connected = crsf_link ? crsf_telem.connected : vario_telem.connected,
                .switch_active =
                    vario_preview || logic::buzzer_inputs::vario_switch_active(model.vario_switch, input.switches),
                .throttle = {channels[2]},
            };
            const logic::vario::Config vario_cfg{
                .switch_index = model.vario_switch,
                .climb_deadband = {model.vario_climb_deadband},
                .sink_deadband = {model.vario_sink_deadband},
                .sink_mode = logic::vario::sink_mode_from_u8(model.vario_sink_mode),
                .motor_mute = model.vario_motor_mute != 0,
            };
            g_buzzer.set_vario(vario.update(sample, vario_cfg));
        }

        if (!menu_open) {
            const bool battery_low = logic::buzzer::battery_low(logic::buzzer::BatteryMv{input.battery_mv},
                                                                logic::buzzer::WarnDeci{g_storage.radio.vbat_warn_deci});
            if (g_battery_warn.step(battery_low, logic::buzzer::Millis{dt_ms})) {
                play_buzzer_cue(logic::buzzer::Cue::WarnBattery);
            }
            const auto rssi_level =
                logic::buzzer::rssi_alarm(logic::buzzer::Rssi{link_telem.rssi}, link_telem.connected);
            if (g_rssi_warn.step(rssi_level, logic::buzzer::Millis{dt_ms})) {
                play_buzzer_cue(rssi_level == logic::buzzer::RssiAlarm::Critical ? logic::buzzer::Cue::WarnRssiCritical
                                                                                  : logic::buzzer::Cue::WarnRssiLow);
            }
        }

        g_buzzer_out.sync(g_buzzer.output());

        __WFI();
    }
}
