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
#include "wm/logic/channels.hpp"
#include "wm/logic/flight.hpp"
#include "wm/logic/framebuffer.hpp"
#include "wm/logic/input.hpp"
#include "wm/logic/menu.hpp"
#include "wm/logic/mixer.hpp"
#include "wm/logic/model_copy.hpp"
#include "wm/logic/power.hpp"
#include "wm/logic/preflight.hpp"
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
#if WM_PCB_I6S
#include "wm/hw/ft6236.hpp"
#include "wm/hw/power.hpp"
#include "wm/logic/board_idle.hpp"
#include "wm/logic/touch_nav.hpp"
#endif
#if WM_PCB_I6S && WM_USE_MULTISWITCH
#include "wm/logic/multiswitch.hpp"
#endif

namespace {

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

#if WM_PCB_I6S
// Menu or calibration wizard open in the current loop pass: the rear buttons then navigate instead of acting as SE/SF.
bool g_menu_active = false;

// Rust hw::update_flight_switches: rear buttons as SE/SF (function SWITCH, outside menus), J4 buttons as SG/SH.
wm::logic::input::BoardSwitches read_board_switches() {
    const auto& r = g_storage.radio;
    const auto rear = wm::hw::keys::read_front_buttons();
    wm::logic::input::BoardSwitches s{};
    s.se_high = !(!g_menu_active && r.rear_left_func == 0 && rear.left);
    s.sf_high = !(!g_menu_active && r.rear_right_func == 0 && rear.right);
    if (r.j4_sg_sh_en != 0) {
        const auto j4 = wm::hw::keys::read_j4_buttons();
        s.sg_high = !j4.left;
        s.sh_high = !j4.right;
    }
    return s;
}
#else
wm::logic::input::BoardSwitches read_board_switches() {
    using wm::hw::Pin;
    using wm::hw::Port;
    return {.se_high = Pin<Port::C, 12>::read(), .sf_high = Pin<Port::C, 15>::read()};
}
#endif

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
    if (wm::logic::mixer::is_tank(g_storage.active_model)) {
        wm::hw::rf::publish_channels(wm::logic::safety::safe_idle_channels(g_storage.active_model));
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
        // Header and name only (18 bytes instead of a full slot); empty slots show their default name (Rust).
        g_model_names[i] = logic::storage::peek_model_name<hw::I2c2Eeprom>(i).value_or(logic::storage::default_name(i));
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

// Blocks until sticks and switches are safe (or the cancel key is pressed), sending failsafe pulses and
// showing the safety warning meanwhile.
void run_preflight(const wm::logic::storage::ModelConfig& model) {
    using namespace wm;
    using namespace logic;
    std::array<safety::ChannelCalibration, 4> cals{};
    for (std::size_t i = 0; i < cals.size(); ++i) {
        cals[i] = safety::ChannelCalibration::from(g_storage.radio.sticks[i]);
    }
    const auto sm = safety::stick_mode_from(g_storage.radio.stick_mode);
    const auto mt = mixer::is_tank(model) ? mixer::ModelType::General :
        static_cast<mixer::ModelType>(model.model_type);
    const auto safe_channels = safety::safe_idle_channels(model);
    buzzer::PreflightTimer preflight{};
    buzzer::PreflightWarning warning{};
    uint32_t buzz_ms = time::millis();
    uint32_t last_render_ms = 0;
    bool rendered = false;
    const bool crsf = model.rf_protocol == 1;
    const bool afhds = g_rf_ok && !crsf;

    for (;;) {
        chip::watchdog::feed();

        const auto in = sample_inputs();
        const std::array<uint16_t, 4> raw{in.raw[0], in.raw[1], in.raw[2], in.raw[3]};
        const bool hold = in.switches.sa == SwitchPos::Down;
        const auto thr = mixer::is_tank(model) ? flight::tank_preflight(in, sm, model) :
            safety::check_preflight_throttle(raw, cals, sm, mt, hold);
        const auto sw = preflight::unsafe_switches(in.switches, mt);
        const uint16_t keys = hw::keys::scan_keys();
        if ((thr == safety::Preflight::Ok && !sw.any()) || (keys & safety::kKeyCancel) != 0) {
            play_buzzer_cue(warning.exit_cue());
            return;
        }
        warning.unsafe();

        const uint32_t now = time::millis();
#if WM_USE_USB
        hw::usb::poll(logic::usb::Millis{now}, safe_channels, in.switches, in.battery_mv);
#endif
        if (crsf) {
            hw::crsf::update_channels(now, safe_channels,
                                      wm::logic::crsf::ArmModeSwitch{model.crsf_arm_mode != 0},
                                      wm::logic::crsf::ArmedState{false});
        } else if (afhds) {
            hw::rf::publish_channels(safe_channels);
        }
        hw::crsf::poll_telemetry(now);
        if (!rendered || now - last_render_ms >= preflight::kRenderIntervalMs) {
            rendered = true;
            last_render_ms = now;
            preflight::render(g_fb, thr, sw);
            hw::lcd::flush(g_fb);
        }
        if (preflight.step(buzzer::Millis{time::millis()})) {
            play_buzzer_cue(buzzer::Cue::WarnPreflight);
        }
        pump_buzzer(buzz_ms);
        __WFI();
    }
}

using namespace wm;
using namespace logic::dashboard;

struct BootContext {
    bool was_watchdog_reset;
    uint16_t keys;
    bool bind_requested;
    bool calibration_requested;
};

bool init_platform() {
#if WM_PCB_I6S
    // Latch the power rail before the user releases the power buttons.
    hw::power::init();
#endif
    chip::watchdog::feed();

    const bool was_watchdog_reset = chip::check_and_clear_reset_flags();
    if (!was_watchdog_reset) {
        hw::keys::check_dfu_entry(chip::kProfile);
    } else {
        hw::keys::init_keys();
    }

    chip::init_system_clock();
    time::init();
#if WM_PCB_I6S
    // Blocks about 160 ms without feeding the watchdog, so it runs before the watchdog starts (as in Rust).
    hw::ft6236::init();
#endif

    chip::watchdog::start();
    return was_watchdog_reset;
}

void load_settings() {
    g_eeprom.detected = hw::I2c2Eeprom::init();
    if (g_eeprom.detected) {
        g_eeprom.addr = hw::I2c2Eeprom::probed_addr();
        g_eeprom.bus = hw::I2c2Eeprom::probed_bus();
        g_eeprom.read_ok = logic::storage::load_storage<hw::I2c2Eeprom>(g_storage);
    } else {
        g_storage = logic::storage::default_storage();
    }
}

void init_audio() { hw::buzzer::init(); }

void init_inputs() {
#if WM_PCB_I6S
    hw::keys::init_j4_keys(g_storage.radio.j4_sg_sh_en != 0);
#endif

    g_p7_enabled = g_storage.radio.ext_adc != 0;
    hw::adc::init(g_p7_enabled);
    chip::watchdog::feed();

    hw::adc::wait_first_conversion();

    logic::input::apply_calibration(g_storage.radio, g_calib);
    // Prime the jitter and battery filters before the first real sample.
    for (int i = 0; i < 16; ++i) {
        (void)sample_inputs();
    }
}

void init_protocol_link() {
    load_trims_from_model();
    g_trims.throttle_enabled = g_storage.radio.throttle_trim != 0;
    // Configure module power OFF before apply_protocol_link() may turn it on.
    hw::uart::init_power_switch(g_storage.radio.ext_module_pwr == 0);
    apply_protocol_link();
}

BootContext read_boot_requests(const bool was_watchdog_reset) {
    const uint16_t boot_keys = hw::keys::scan_keys();
    const bool bind_on_boot = !was_watchdog_reset && (boot_keys & logic::kKeyBind) != 0;
    // OK held at power-on starts the calibration wizard directly.
    const bool calib_on_boot = !was_watchdog_reset && (boot_keys & logic::kKeyOk) != 0;
    return {was_watchdog_reset, boot_keys, bind_on_boot, calib_on_boot};
}

void init_rf(const BootContext& boot) {
    const auto& model = g_storage.active_model;
    g_rf_ok = hw::rf::init(tx_id_from_uid());
    apply_rf_model();
    if (boot.bind_requested && g_rf_ok && model.rf_protocol == 0) {
        hw::rf::set_bind_mode(true);
    }
}

void init_display() {
    hw::lcd::init();
    apply_lcd_settings(g_storage.radio);
}

void init_usb_and_trainer() {
#if WM_USE_USB
    if (const auto usb_mode = logic::usb::mode_from_stored(g_storage.radio.usb_mode);
        usb_mode != logic::usb::UsbMode::Off) {
        hw::usb::start(usb_mode);
    }
#endif

    hw::trainer::init();
    apply_trainer_config(g_storage.radio);
}

BootContext initialize_application() {
    const bool was_watchdog_reset = init_platform();
    load_settings();
    init_audio();
    init_inputs();
    init_protocol_link();
    const auto boot = read_boot_requests(was_watchdog_reset);
    init_rf(boot);
    init_display();
    init_usb_and_trainer();
    return boot;
}

void run_startup(const BootContext& boot) {
    g_buzzer.set_enabled(g_storage.radio.audio_enabled != 0);
    if (boot.was_watchdog_reset) {
        // Resume immediately after a watchdog reset: no splash, only the warning chirp.
        play_buzzer_cue(logic::buzzer::Cue::WatchdogRecover);
    } else {
        g_fb.clear();
        draw_splash(g_fb);
        hw::lcd::flush(g_fb);
        play_buzzer_cue(logic::buzzer::Cue::Welcome);
    }
    uint32_t buzz_ms = time::millis();
    if (boot.calibration_requested) {
        logic::calib::start(g_calib_wizard);
        play_buzzer_cue(g_calib_wizard.cue);
    }
    if (!boot.was_watchdog_reset && !logic::calib::active(g_calib_wizard)) {
        const uint32_t splash_start_ms = time::millis();
        while (time::millis() - splash_start_ms < 1200) {
            chip::watchdog::feed();
            pump_buzzer(buzz_ms);
            __WFI();
        }
        run_preflight(g_storage.active_model);
    }
}

struct RuntimeState {
    PageState pages{};
    uint32_t last_draw_ms = 0;
    uint16_t prev_keys = 0;
    logic::menu::State menu{};
    logic::mixer::TankState tank{};
#if WM_PCB_I6S
    logic::board_idle::State board_idle{};
    logic::touch_nav::SwipeTracker swipe{};
#if WM_USE_MULTISWITCH
    logic::multiswitch::Pages multiswitch{};
#endif
    uint16_t ok_hold_ms = 0;
    bool j4_enabled = false;
    logic::Switches prev_switches{};
#else
    bool ok_armed = false;
    uint32_t ok_press_ms = 0;
#endif
    uint32_t last_loop_ms = 0;
    bool rx_id_dirty = false;
    bool prev_armed = false;
    logic::buzzer_inputs::ActivityTracker activity{};
    logic::buzzer_inputs::PotCenterTracker<board::kAuxCount - 4> pot_centre{};
    logic::buzzer::InactivityTimer inactivity{};
    logic::backlight::Dimmer dimmer{};
    logic::vario::State vario{};
};

enum class UiMode { Dashboard, Menu, Calibration };

UiMode ui_mode(const RuntimeState& state) {
    if (logic::calib::active(g_calib_wizard)) {
        return UiMode::Calibration;
    }
    return state.menu.screen == logic::menu::Screen::Closed ? UiMode::Dashboard : UiMode::Menu;
}

struct InputContext {
#if WM_PCB_I6S
    bool menu_active_pre;
#if WM_USE_MULTISWITCH
    std::optional<uint8_t> multiswitch_page_slot;
#endif
#endif
};

struct LoopFrame {
    const logic::input::InputState& input;
    logic::safety::StickMode stick_mode;
    logic::auto_select::Changes auto_changes;
    uint16_t keys;
    uint16_t pressed;
    uint32_t now;
    uint16_t dt_ms;
#if WM_PCB_I6S
    bool menu_active_pre;
    hw::keys::FrontButtons rear;
#if WM_USE_MULTISWITCH
    std::optional<uint8_t> multiswitch_page_slot;
#endif
#endif
};

struct BoardControls {
    uint16_t frame_keys;
    uint16_t frame_pressed;
    bool instant_rising;
#if WM_PCB_I6S
    logic::board_idle::Result idle;
#endif
};

// Applied link state before this iteration's menu actions.
struct LinkState {
    bool sim;
    bool afhds;
};

using logic::Channels;

RuntimeState make_runtime_state(const BootContext& boot) {
    RuntimeState state{};
    // The held boot OK must not step the calibration wizard.
    state.prev_keys = boot.calibration_requested ? boot.keys : 0;
#if WM_PCB_I6S
    state.j4_enabled = g_storage.radio.j4_sg_sh_en != 0;
#endif
    state.last_loop_ms = time::millis();
    const auto boot_input = sample_inputs();
#if WM_PCB_I6S
    state.prev_switches = boot_input.switches;
#endif
    // Only an assigned arm switch arms chimes, save inhibits and the CRSF flag.
    state.prev_armed = logic::mixer::eval_arm_switch(g_storage.active_model.arm_switch, boot_input.switches);
    state.activity.prime(boot_input.raw, boot_input.switches);
    return state;
}

InputContext begin_iteration(RuntimeState& state) {
    const auto& model = g_storage.active_model;

    chip::watchdog::feed();
    configure_pages(state.pages, model);
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    const auto multiswitch_page_slot =
        state.pages.page >= kBasePageCount
            ? logic::multiswitch::slot_at(model.multiswitch_addresses, state.pages.page - kBasePageCount)
            : std::nullopt;
#endif

#if WM_PCB_I6S
    const bool menu_active_pre = ui_mode(state) != UiMode::Dashboard;
    g_menu_active = menu_active_pre;
#endif
    return {
#if WM_PCB_I6S
        .menu_active_pre = menu_active_pre,
#if WM_USE_MULTISWITCH
        .multiswitch_page_slot = multiswitch_page_slot,
#endif
#endif
    };
}

LoopFrame sample_frame(RuntimeState& state, const logic::input::InputState& input,
                       [[maybe_unused]] const InputContext& context) {
#if WM_PCB_I6S
    const bool menu_active_pre = context.menu_active_pre;
#if WM_USE_MULTISWITCH
    const auto& model = g_storage.active_model;
    const auto multiswitch_page_slot = context.multiswitch_page_slot;
#endif
#endif
    const auto stick_mode = logic::safety::stick_mode_from(g_storage.radio.stick_mode);
    const auto auto_changes = g_auto_select.step({input.switches, input.pots.vr1, input.pots.vr2});
#if WM_PCB_I6S
    const auto rear = hw::keys::read_front_buttons();
    const auto touch = g_storage.radio.touch_disabled == 0 && hw::ft6236::is_touch_int_asserted()
                           ? hw::ft6236::read_touch()
                           : std::nullopt;
    const uint16_t keys = logic::touch_nav::update_inputs(
        touch, input.gimbals, menu_active_pre, g_storage.radio.rear_left_func, g_storage.radio.rear_right_func,
        g_storage.radio.throttle_trim != 0, {rear.left, rear.right}, state.swipe, time::millis()
#if WM_USE_MULTISWITCH
        , state.pages.page >= kBasePageCount,
        multiswitch_page_slot ? model.multiswitch_momentary[*multiswitch_page_slot] : 0
#endif
    );
#else
    const uint16_t keys = hw::keys::scan_keys();
#endif

    const uint16_t pressed = static_cast<uint16_t>(keys & ~state.prev_keys);
    state.prev_keys = keys;
    const uint32_t now = time::millis();
    // Clamp as in Rust: long iterations (EEPROM writes, LCD flush) still advance the timers.
    const auto dt_ms = static_cast<uint16_t>(std::min<uint32_t>(now - state.last_loop_ms, 100));
    state.last_loop_ms = now;

    g_buzzer.set_enabled(g_storage.radio.audio_enabled != 0);
    g_buzzer.tick(logic::buzzer::Millis{dt_ms});

    return {
        .input = input,
        .stick_mode = stick_mode,
        .auto_changes = auto_changes,
        .keys = keys,
        .pressed = pressed,
        .now = now,
        .dt_ms = dt_ms,
#if WM_PCB_I6S
        .menu_active_pre = menu_active_pre,
        .rear = rear,
#if WM_USE_MULTISWITCH
        .multiswitch_page_slot = multiswitch_page_slot,
#endif
#endif
    };
}

BoardControls update_board_controls([[maybe_unused]] RuntimeState& state, const LoopFrame& frame) {
    const auto& input = frame.input;
    const uint16_t pressed = frame.pressed;
#if WM_PCB_I6S
    const uint32_t now = frame.now;
    const uint16_t dt_ms = frame.dt_ms;
    const bool menu_active_pre = frame.menu_active_pre;
    const auto& rear = frame.rear;
#endif

#if WM_PCB_I6S
    const auto idle = state.board_idle.update({.now_ms = now,
                                               .dt_ms = dt_ms,
                                               .keys = frame.keys,
                                               .menu_active = menu_active_pre,
                                               .buttons = {rear.left, rear.right},
                                               .power_pressed = hw::power::is_power_button_pressed(),
                                               .rear_left_func = g_storage.radio.rear_left_func,
                                               .rear_right_func = g_storage.radio.rear_right_func});
    if (idle.leds) {
        hw::power::set_leds(*idle.leds);
    }
    if (idle.shutdown) {
        play_buzzer_cue(logic::buzzer::Cue::PowerOff);
        g_buzzer_out.sync(g_buzzer.output());
        store_trims_in_model();
        logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
        logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
        hw::power::power_off();
    }
    // A swipe in menus and the calibration wizard must not also confirm (Rust take_frame_keys).
    const uint16_t frame_keys = logic::board_idle::menu_keys(frame.keys, menu_active_pre);
    const uint16_t frame_pressed = logic::board_idle::menu_keys(pressed, menu_active_pre);
#else
    const uint16_t frame_keys = frame.keys;
    const uint16_t frame_pressed = pressed;
#endif

    const bool instant_rising = g_instant_edge.update(logic::trim::instant_switch_down(g_storage.radio, input.raw));

    return {
        .frame_keys = frame_keys,
        .frame_pressed = frame_pressed,
        .instant_rising = instant_rising,
#if WM_PCB_I6S
        .idle = idle,
#endif
    };
}

LinkState synchronize_protocol_link() {
    if (logic::protocol::from_raw(g_storage.active_model.rf_protocol) != g_link_protocol ||
        usb_sim_active() != sim_applied()) {
        apply_protocol_link();
    }
    const bool sim = sim_applied();
    const bool afhds = g_rf_ok && g_storage.active_model.rf_protocol == 0;

    return {sim, afhds};
}

void handle_menu_action(RuntimeState& state, const LoopFrame& frame, const LinkState& links,
                        const logic::menu::Event& ev) {
    const auto& input = frame.input;
    const uint32_t now = frame.now;
    const bool afhds = links.afhds;

    switch (ev.action) {
        case logic::menu::Action::EnterModelSelect:
            refresh_model_names();
            break;
        case logic::menu::Action::SwitchModel:
            switch_model(ev.index);
            state.tank = {};
            refresh_model_names();
            apply_rf_model();
            apply_protocol_link();
#if WM_PCB_I6S && WM_USE_MULTISWITCH
            state.multiswitch = {};
#endif
            state.prev_armed = logic::mixer::eval_arm_switch(g_storage.active_model.arm_switch, input.switches);
            state.vario.reset();
            break;
        case logic::menu::Action::SaveActiveModel:
            store_trims_in_model();
            logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
            apply_protocol_link();
            break;
        case logic::menu::Action::ResetActiveModel:
            state.tank = {};
#if WM_PCB_I6S && WM_USE_MULTISWITCH
            state.multiswitch = {};
#endif
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
#if WM_PCB_I6S
            if ((g_storage.radio.j4_sg_sh_en != 0) != state.j4_enabled) {
                state.j4_enabled = g_storage.radio.j4_sg_sh_en != 0;
                hw::keys::init_j4_keys(state.j4_enabled);
            }
#endif
            g_trims.throttle_enabled = g_storage.radio.throttle_trim != 0;
#if WM_USE_USB
            hw::usb::set_mode(logic::usb::mode_from_stored(g_storage.radio.usb_mode));
#endif
            break;
        case logic::menu::Action::CopyActiveModel:
            // The copy gets the current trims, not the ones from the last trim auto-save (Rust keeps
            // active_model.trims in sync every loop).
            store_trims_in_model();
            logic::model_copy::copy_model<hw::I2c2Eeprom>(g_storage, logic::model_copy::active_slot(g_storage),
                                                          logic::model_copy::DstSlot{ev.index});
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
        case logic::menu::Action::CrsfRequest:
            hw::crsf::menu_apply(state.menu.crsf_req, now);
            break;
        case logic::menu::Action::None:
            break;
    }
}

std::optional<uint8_t> update_ui(RuntimeState& state, const LoopFrame& frame, const BoardControls& controls,
                                 const LinkState& links) {
    const auto& input = frame.input;
    const uint16_t pressed = frame.pressed;
    const uint32_t now = frame.now;
    const uint16_t dt_ms = frame.dt_ms;
    const auto& auto_changes = frame.auto_changes;
    const uint16_t frame_keys = controls.frame_keys;
    const uint16_t frame_pressed = controls.frame_pressed;
#if WM_PCB_I6S
    const auto& idle = controls.idle;
#endif

#if WM_PCB_I6S && WM_USE_MULTISWITCH
    std::optional<uint8_t> multiswitch_touch_slot{};
    const auto multiswitch_page_slot = frame.multiswitch_page_slot;
#endif
    switch (ui_mode(state)) {
        case UiMode::Calibration: {
            const logic::calib::Input cal_in{frame_pressed, frame_keys, input.raw, dt_ms};
            if (logic::calib::update(g_calib_wizard, cal_in, g_storage.radio)) {
                logic::input::apply_calibration(g_storage.radio, g_calib);
                logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
            }
            play_buzzer_cue(g_calib_wizard.cue);
            break;
        }
        case UiMode::Dashboard: {
            const uint8_t prev_page = state.pages.page;
            handle_keys(state.pages, pressed);
            if (state.pages.page != prev_page) {
                play_buzzer_cue(logic::buzzer::Cue::NavStep);
            }
#if WM_PCB_I6S
#if WM_USE_MULTISWITCH
            if (state.pages.page == prev_page && state.pages.page >= kBasePageCount) {
                multiswitch_touch_slot = multiswitch_page_slot;
            }
#endif
            // Long centre tap or the right rear button held for 1.2 s opens the menu.
            if (idle.ok_held) {
                state.ok_hold_ms =
                    static_cast<uint16_t>(std::min<uint32_t>(uint32_t{state.ok_hold_ms} + dt_ms, UINT16_MAX));
                if (state.ok_hold_ms >= kMenuHoldMs) {
                    logic::menu::open(state.menu);
#if WM_USE_MULTISWITCH
                    if (state.pages.page >= kBasePageCount) {
                        state.swipe.swiped = true;
                    }
#endif
                    play_buzzer_cue(logic::buzzer::Cue::Click);
                    state.ok_hold_ms = 0;
                    state.board_idle.clear_menu_open_state();
                }
            } else {
                state.ok_hold_ms = 0;
                state.board_idle.clear_menu_open_state();
            }
#else
            if ((frame.keys & logic::kKeyOk) == 0) {
                state.ok_armed = true;
            } else if ((pressed & logic::kKeyOk) != 0) {
                state.ok_press_ms = now;
            }
            if (state.ok_armed && (frame.keys & logic::kKeyOk) != 0 && now - state.ok_press_ms >= kMenuHoldMs) {
                logic::menu::open(state.menu);
                play_buzzer_cue(logic::buzzer::Cue::Click);
                state.ok_armed = false;
            }
#endif
            break;
        }
        case UiMode::Menu: {
            const auto ev =
                logic::menu::update(state.menu,
                                    logic::menu::Input{frame_pressed, frame_keys, now, auto_changes,
                                                       &hw::crsf::menu_loader(), &hw::crsf::menu_command(),
                                                       &hw::crsf::menu_devices(), input.pots.vr1, input.pots.vr2},
                                    g_storage);
            if (state.menu.nav_tick) {
                play_buzzer_cue(logic::buzzer::Cue::NavStep);
            }
            play_buzzer_cue(state.menu.cue);
            handle_menu_action(state, frame, links, ev);
            break;
        }
    }
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    return multiswitch_touch_slot;
#else
    return std::nullopt;
#endif
}

void process_model_and_bind_events(RuntimeState& state, const LoopFrame& frame, const LinkState& links,
                                   const bool armed, const bool menu_open) {
    const auto& model = g_storage.active_model;
    const uint16_t pressed = frame.pressed;
    const uint16_t dt_ms = frame.dt_ms;
    const bool afhds = links.afhds;

    if (g_rf_ok) {
        if (const auto id = hw::rf::take_pending_rx_save()) {
            if (id->value != 0 && id->value != 0xFFFF'FFFFu && model.rx_id != id->value) {
                g_storage.active_model.rx_id = id->value;
                state.rx_id_dirty = true;
            }
        }
    }
    if (state.rx_id_dirty && !armed) {
        store_trims_in_model();
        if (logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage)) {
            play_buzzer_cue(logic::buzzer::Cue::RxSave);
            state.rx_id_dirty = false;
        }
    }

    const auto bind_ev = logic::bind::update(g_bind, logic::bind::Input{
                                                         .link = {afhds, hw::rf::is_binding()},
                                                         .menu_active = menu_open,
                                                         .keys = frame.keys,
                                                         .pressed = pressed,
                                                         .dt = logic::vario::Millis{dt_ms},
                                                     });
    switch (bind_ev.action) {
        case logic::bind::Action::Start:
            hw::rf::set_bind_mode(true);
            break;
        case logic::bind::Action::Stop:
            hw::rf::set_bind_mode(false);
            break;
        case logic::bind::Action::None:
            break;
    }
    play_buzzer_cue(bind_ev.cue);
    if (bind_ev.next_page) {
        logic::dashboard::next_page(state.pages);
        play_buzzer_cue(logic::buzzer::Cue::NavStep);
    }
}

void update_trims_and_timer(RuntimeState& state, const LoopFrame& frame, const BoardControls& controls,
                            const bool menu_open) {
    const auto& model = g_storage.active_model;
    const auto& input = frame.input;
    const auto stick_mode = frame.stick_mode;
    const uint16_t dt_ms = frame.dt_ms;
    const bool instant_rising = controls.instant_rising;
#if WM_PCB_I6S
    const auto& idle = controls.idle;
#endif

    if (!menu_open) {
        const auto trim_ev = g_trims.update(frame.keys, logic::vario::Millis{dt_ms});
        switch (trim_ev.kind) {
            case logic::trim::Event::Kind::Step:
                g_buzzer.play_trim(logic::buzzer::TrimStep{trim_ev.value});
                break;
            case logic::trim::Event::Kind::Center:
                play_buzzer_cue(logic::buzzer::Cue::TrimCenter);
                break;
            case logic::trim::Event::Kind::Limit:
                play_buzzer_cue(logic::buzzer::Cue::TrimLimit);
                break;
            case logic::trim::Event::Kind::None:
                break;
        }
#if WM_PCB_I6S
        if (instant_rising || idle.instant_trim_clicked) {
#else
        if (instant_rising) {
#endif
            const auto sticks = logic::flight::flight_controls(input.gimbals, stick_mode);
            const auto result = g_trims.instant_trim(
                logic::trim::InstantSticks{.roll = sticks.aileron, .pitch = sticks.elevator, .yaw = sticks.rudder});
            switch (result) {
                case logic::trim::InstantResult::Adjusted:
                    play_buzzer_cue(logic::buzzer::Cue::InstantTrim);
                    break;
                case logic::trim::InstantResult::Reset:
                    play_buzzer_cue(logic::buzzer::Cue::TrimCenter);
                    break;
                case logic::trim::InstantResult::None:
                    break;
            }
        }
    }

    if (g_trim_save.tick(g_trims.values, state.prev_armed, logic::vario::Millis{dt_ms})) {
        store_trims_in_model();
        g_trim_save.saved(logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage));
    }

    const auto timer_cue = logic::flight_timer::update(
        g_flight_timer, logic::flight_timer::Input{
                            .model = {g_storage.radio.active_model},
                            .config = timer_config(model),
                            .was_armed = state.prev_armed,
                            .menu_active = menu_open,
                            .cancel_held = (frame.keys & logic::safety::kKeyCancel) != 0,
#if WM_PCB_I6S
                            .reset_clicked = idle.timer_reset_clicked,
#endif
                            .throttle = logic::flight::flight_controls(input.gimbals, stick_mode).throttle,
                            .switches = input.switches,
                            .dt = logic::vario::Millis{dt_ms},
                        });
    play_buzzer_cue(timer_cue);
}

Channels compute_channels_and_trainer(RuntimeState& state, const LoopFrame& frame, const LinkState& links) {
    const auto& model = g_storage.active_model;
    const auto& input = frame.input;
    const auto stick_mode = frame.stick_mode;
    const uint16_t dt_ms = frame.dt_ms;
    const bool sim = links.sim;

    auto channels =
        logic::flight::rf_channels(input, stick_mode, model, g_trims.values, g_storage.radio.throttle_trim, state.tank);
    const bool trainer_in = (g_storage.radio.trainer_config & logic::menu::kTrainerPpmIn) != 0;
    const bool trainer_out = (g_storage.radio.trainer_config & logic::menu::kTrainerPpmOut) != 0;
    hw::trainer::tick(logic::trainer::DeltaMs{dt_ms});
    if (trainer_in) {
        channels =
            g_handover.process(input.switches.sd == logic::SwitchPos::Down, hw::trainer::student_channels(), channels);
        logic::mixer::apply_tank_arming(channels, model, input.switches);
    }
    if (trainer_out && !sim) {
        hw::trainer::publish_out(channels);
    }
    return channels;
}

#if WM_PCB_I6S && WM_USE_MULTISWITCH
void update_multiswitch(RuntimeState& state, std::optional<uint8_t> multiswitch_touch_slot, const LinkState& links) {
    const auto& model = g_storage.active_model;
    const bool sim = links.sim;

    const bool crsf = model.rf_protocol == 1;
    state.multiswitch.configure(model.multiswitch_addresses, model.multiswitch_momentary);
    configure_pages(state.pages, model);
    if (state.menu.screen != logic::menu::Screen::Closed || logic::calib::active(g_calib_wizard) || !crsf || sim ||
        logic::flight_timer::modal(g_flight_timer) || state.board_idle.power.progress_pct() != 0 ||
        g_storage.radio.touch_disabled != 0) {
        multiswitch_touch_slot.reset();
    }
    if (!multiswitch_touch_slot) {
        state.swipe.held_switch.reset();
    }
    if (state.multiswitch.update_touch(multiswitch_touch_slot, state.swipe.held_switch, state.swipe.tapped_switch)) {
        play_buzzer_cue(logic::buzzer::Cue::Click);
    }
}

#endif

logic::afhds2a::Telemetry service_outputs_and_telemetry([[maybe_unused]] RuntimeState& state, const LoopFrame& frame,
                                                        const LinkState& links, const Channels& channels,
                                                        const bool armed) {
    const auto& model = g_storage.active_model;
    const uint32_t now = frame.now;
    const bool sim = links.sim;
    const bool afhds = links.afhds;

    const bool crsf = model.rf_protocol == 1;
#if WM_USE_USB
    const auto& input = frame.input;
    hw::usb::poll(logic::usb::Millis{now}, channels, input.switches, input.battery_mv);
#endif
    if (crsf) {
        if (!sim) {
            hw::crsf::update_channels(now, channels, wm::logic::crsf::ArmModeSwitch{model.crsf_arm_mode != 0},
                                      wm::logic::crsf::ArmedState{armed});
#if WM_PCB_I6S && WM_USE_MULTISWITCH
            hw::crsf::update_multiswitch(now, state.multiswitch);
#endif
        }
    } else if (afhds && !sim) {
        hw::rf::publish_channels(channels);
    }
    if (!sim) {
        hw::crsf::poll_telemetry(now);
    }

    // Link telemetry of the active protocol: CRSF link quality stands in for the AFHDS 2A RSSI.
    const auto link_telem = crsf      ? logic::telemetry::from_crsf(hw::crsf::telemetry())
                            : g_rf_ok ? hw::rf::get_telemetry()
                                      : logic::afhds2a::Telemetry{};

    return link_telem;
}

void render_if_due(RuntimeState& state, const LoopFrame& frame, const LinkState& links, const Channels& channels,
                   const logic::afhds2a::Telemetry& link_telem) {
    const auto& model = g_storage.active_model;
    const auto& input = frame.input;
    const uint32_t now = frame.now;
    const bool afhds = links.afhds;

    const bool crsf = model.rf_protocol == 1;
    if (now - state.last_draw_ms >= 100) {
        g_fb.clear();
        const bool rf_binding = logic::bind::binding_shown({afhds, hw::rf::is_binding()});
        const bool rf_bound = g_rf_ok && hw::rf::is_bound();
        const uint32_t rf_sent = g_rf_ok ? hw::rf::packets_sent() : 0u;
        logic::dashboard::track_session(g_rf_stats, link_telem);
        switch (ui_mode(state)) {
            case UiMode::Calibration: {
                logic::calib::render(g_fb, g_calib_wizard, input.raw, g_storage.radio, model.model_type);
                break;
            }
            case UiMode::Dashboard: {
                render(g_fb, state.pages,
                       View{input, g_storage, g_trims, channels, crsf, link_telem, rf_binding, g_rf_ok, rf_bound, now,
                            rf_sent, g_rf_stats, logic::flight_timer::display(g_flight_timer, timer_config(model)),
                            logic::flight_timer::modal(g_flight_timer), hw::crsf::telemetry(), usb_sim_active(),
                            hw::rf::last_chip_id()
#if WM_PCB_I6S && WM_USE_MULTISWITCH
                                ,
                            state.multiswitch.switches()
#endif
                       });
                break;
            }
            case UiMode::Menu: {
                const logic::storage::EepromStatus eeprom{
                    .addr = g_eeprom.addr,
                    .detected = g_eeprom.detected,
                    .read_ok = g_eeprom.read_ok,
                    .write_err = hw::I2c2Eeprom::write_failed(),
                    .bus = g_eeprom.bus,
                };
                logic::menu::render(g_fb, state.menu, g_model_names, g_storage,
                                    logic::menu::Live{channels, input.raw, input.switches, eeprom, chip::kProfile.name,
                                                      kBuild, &hw::crsf::menu_loader(), &hw::crsf::menu_command(),
                                                      &hw::crsf::menu_devices()});
                break;
            }
        }
#if WM_PCB_I6S
        logic::board_idle::render_overlay(g_fb, state.board_idle.power.progress_pct());
#endif
        hw::lcd::flush(g_fb);
        state.last_draw_ms = now;
    }
}

void update_feedback_and_alarms(RuntimeState& state, const LoopFrame& frame, const LinkState& links,
                                const Channels& channels, const logic::afhds2a::Telemetry& link_telem, const bool armed,
                                const bool menu_open) {
    const auto& model = g_storage.active_model;
    const auto& input = frame.input;
    const uint32_t now = frame.now;
    const uint16_t dt_ms = frame.dt_ms;
    const bool afhds = links.afhds;

    play_buzzer_cue(logic::buzzer::arm_cue(logic::buzzer::ArmEdge{
        .arm_switch = logic::buzzer::SwitchIndex{model.arm_switch},
        .is_armed = armed,
        .was_armed = state.prev_armed,
        .menu_active = menu_open,
    }));
    state.prev_armed = armed;

    const bool user_active = state.activity.step(input.raw, input.switches, frame.keys);
    if (const auto level = state.dimmer.step(user_active, g_storage.radio, dt_ms)) {
        hw::lcd::set_backlight_level(*level);
    }
#if WM_PCB_I6S
    if (input.switches != state.prev_switches && g_storage.radio.switch_beep != 0 && !menu_open) {
        play_buzzer_cue(logic::buzzer::Cue::SwitchBeep);
    }
    state.prev_switches = input.switches;
#endif
    const std::size_t pot_count = g_storage.radio.ext_adc != 0 ? board::kAuxCount - 4 : 2;
    if (state.pot_centre.step(std::span<const int16_t>(input.aux_pots).subspan(4, pot_count),
                              std::span<const uint8_t>(g_storage.radio.adc_modes).subspan(4, pot_count))) {
        play_buzzer_cue(logic::buzzer::Cue::PotCenter);
    }
    if (state.inactivity.step(user_active, logic::buzzer::Millis{dt_ms})) {
        play_buzzer_cue(logic::buzzer::Cue::WarnInactivity);
    }

    // The Vario Setup screen previews the tone regardless of the vario switch, as in Rust.
    const bool vario_preview = state.menu.screen == logic::menu::Screen::VarioSetup;
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
        g_buzzer.set_vario(state.vario.update(sample, vario_cfg));
    }

    if (!menu_open) {
        const bool battery_low = logic::buzzer::battery_low(logic::buzzer::BatteryMv{input.battery_mv},
                                                            logic::buzzer::WarnDeci{g_storage.radio.vbat_warn_deci});
        if (g_battery_warn.step(battery_low, logic::buzzer::Millis{dt_ms})) {
            play_buzzer_cue(logic::buzzer::Cue::WarnBattery);
        }
        const auto rssi_level = logic::buzzer::rssi_alarm(logic::buzzer::Rssi{link_telem.rssi}, link_telem.connected);
        if (g_rssi_warn.step(rssi_level, logic::buzzer::Millis{dt_ms})) {
            play_buzzer_cue(rssi_level == logic::buzzer::RssiAlarm::Critical ? logic::buzzer::Cue::WarnRssiCritical
                                                                             : logic::buzzer::Cue::WarnRssiLow);
        }
    }
}

void finish_iteration() {
    g_buzzer_out.sync(g_buzzer.output());

    __WFI();
}

void run_iteration(RuntimeState& state) {
    const auto context = begin_iteration(state);
    const auto input = sample_inputs();
    const auto frame = sample_frame(state, input, context);
    const auto controls = update_board_controls(state, frame);
    const auto links = synchronize_protocol_link();
    [[maybe_unused]] const auto multiswitch_touch_slot = update_ui(state, frame, controls, links);
    const bool armed = logic::mixer::eval_arm_switch(g_storage.active_model.arm_switch, frame.input.switches);
    const bool menu_open = ui_mode(state) != UiMode::Dashboard;
    process_model_and_bind_events(state, frame, links, armed, menu_open);
    update_trims_and_timer(state, frame, controls, menu_open);
    const auto channels = compute_channels_and_trainer(state, frame, links);
#if WM_PCB_I6S && WM_USE_MULTISWITCH
    update_multiswitch(state, multiswitch_touch_slot, links);
#endif
    const auto telemetry = service_outputs_and_telemetry(state, frame, links, channels, armed);
    render_if_due(state, frame, links, channels, telemetry);
    update_feedback_and_alarms(state, frame, links, channels, telemetry, armed, menu_open);
    finish_iteration();
}

}  // namespace

int main() {
    const auto boot = initialize_application();
    run_startup(boot);
    auto state = make_runtime_state(boot);
    for (;;) {
        run_iteration(state);
    }
}
