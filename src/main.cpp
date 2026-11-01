#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>

#include "ferrox/chip/mcu.hpp"
#include "ferrox/chip/watchdog.hpp"
#include "ferrox/hw/adc.hpp"
#include "ferrox/hw/crsf.hpp"
#include "ferrox/hw/eeprom.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/hw/keys.hpp"
#include "ferrox/hw/rf.hpp"
#include "ferrox/hw/st7567.hpp"
#include "ferrox/logic/dashboard.hpp"
#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/calib.hpp"
#include "ferrox/logic/flight.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/menu.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/trim.hpp"
#include "ferrox/time/systick.hpp"

namespace {

// Latest measurements, updated by the main loop (read by later UI/RF code).
volatile uint16_t g_battery_mv = 0;
volatile uint16_t g_keys = 0;

// Radio and active model settings loaded from the external EEPROM.
ferrox::logic::storage::RadioStorage g_storage{};

// Calibration and filter state derived from g_storage.radio.
ferrox::logic::input::Calibration g_calib{};

// Stick and pot endpoint capture, started from the main menu.
ferrox::logic::calib::Wizard g_calib_wizard{};

// Display content is rendered into this buffer and flushed to the ST7567.
ferrox::logic::display::Framebuffer g_fb{};

// RF radio init result, and whether the A7105 is currently silenced (starts silenced, as in hw::rf).
bool g_rf_ok = false;
bool g_rf_silenced = true;

ferrox::logic::input::BoardSwitches read_board_switches() {
    using ferrox::hw::Pin;
    using ferrox::hw::Port;
    return {.se_high = Pin<Port::C, 12>::read(), .sf_high = Pin<Port::C, 15>::read()};
}

ferrox::logic::input::InputState sample_inputs() {
    return ferrox::logic::input::process(ferrox::hw::adc::read_raw(), read_board_switches(), g_calib);
}

// Holding OK this long on the dashboard opens the menu.
constexpr uint32_t kMenuHoldMs = 1200;

// Holding BIND this long on the dashboard starts binding.
constexpr uint32_t kBindHoldMs = 1000;

// Model names shown in the model list, filled from EEPROM when the list is opened.
ferrox::logic::menu::ModelNames g_model_names{};

void apply_crsf_config() {
    using namespace ferrox;
    const auto& model = g_storage.active_model;
    const uint8_t baud_idx = model.crsf_baud < logic::kBaudRates.size() ? model.crsf_baud : logic::kDefaultBaudIndex;
    hw::crsf::set_enabled(true, baud_idx, model.crsf_half_duplex != 0);
}

// TX ID is the XOR of the three 32-bit words of the MCU UID, as in the Rust firmware.
uint32_t tx_id_from_uid() {
    using namespace ferrox;
    const auto uid = chip::read_uid(chip::kProfile);
    auto word = [&uid](std::size_t i) {
        return static_cast<uint32_t>(uid[i]) | (static_cast<uint32_t>(uid[i + 1]) << 8) |
               (static_cast<uint32_t>(uid[i + 2]) << 16) | (static_cast<uint32_t>(uid[i + 3]) << 24);
    };
    return word(0) ^ word(4) ^ word(8);
}

// Silences the A7105 for CRSF models. Only changes state when the protocol changes.
void set_rf_silenced(bool silenced) {
    if (!g_rf_ok || silenced == g_rf_silenced) {
        return;
    }
    ferrox::hw::rf::set_silenced(silenced);
    g_rf_silenced = silenced;
}

// Applies power, RX output settings and the silence state of the active model. Keeps the RX ID.
void apply_rf_link() {
    using namespace ferrox;
    if (!g_rf_ok) {
        return;
    }
    const auto& model = g_storage.active_model;
    hw::rf::set_rf_power(model.rf_power);
    hw::rf::set_rx_settings(model.servo_rate_hz, model.rx_out_mode == 1, model.rx_serial_proto == 1);
    set_rf_silenced(model.rf_protocol == 1);
}

// Full RF setup for the active model, used at boot and on model switch.
void apply_rf_model() {
    if (!g_rf_ok) {
        return;
    }
    ferrox::hw::rf::set_rx_id(g_storage.active_model.rx_id);
    apply_rf_link();
}

void refresh_model_names() {
    using namespace ferrox;
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
void apply_lcd_settings(const ferrox::logic::storage::RadioConfig& r) {
    ferrox::hw::lcd::set_backlight_level(static_cast<uint8_t>(r.backlight_brightness * 10));
    ferrox::hw::lcd::set_contrast(r.lcd_contrast);
}

// Stores the current model, then loads the model at `index` as the active one.
void switch_model(uint8_t index) {
    using namespace ferrox::logic::storage;
    using Dev = ferrox::hw::I2c2Eeprom;
    save_active_model<Dev>(g_storage);
    g_storage.radio.active_model = index;
    save_radio_config<Dev>(g_storage);
    load_active_model<Dev>(g_storage);
}

// Blocks until sticks and switches are safe (or the cancel key is pressed), sending failsafe pulses meanwhile.
void run_preflight(const ferrox::logic::storage::ModelConfig& model) {
    using namespace ferrox;
    using namespace logic;
    std::array<safety::ChannelCalibration, 4> cals{};
    for (std::size_t i = 0; i < cals.size(); ++i) {
        cals[i] = safety::ChannelCalibration::from(g_storage.radio.sticks[i]);
    }
    const auto sm = safety::stick_mode_from(g_storage.radio.stick_mode);
    const auto mt = static_cast<mixer::ModelType>(model.model_type);

    for (;;) {
        chip::watchdog::feed();

        const auto in = sample_inputs();
        const std::array<uint16_t, 4> raw{in.raw[0], in.raw[1], in.raw[2], in.raw[3]};
        const bool hold = in.switches.sa == SwitchPos::Down;
        const bool ready = safety::check_preflight_throttle(raw, cals, sm, mt, hold) == safety::Preflight::Ok &&
                           safety::switches_safe(in.switches, mt);
        g_keys = hw::keys::scan_keys();
        if (ready || (g_keys & safety::kKeyCancel) != 0) {
            return;
        }

        hw::crsf::update_channels(time::millis(), safety::kSafeIdleChannels, model.crsf_arm_mode != 0, false);
        hw::crsf::poll_telemetry(time::millis());
        __WFI();
    }
}

}  // namespace

int main() {
    using namespace ferrox;

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

    if (hw::I2c2Eeprom::init()) {
        logic::storage::load_storage<hw::I2c2Eeprom>(g_storage);
    } else {
        g_storage = logic::storage::default_storage();
    }

    hw::adc::init(g_storage.radio.ext_adc != 0);
    chip::watchdog::feed();

    hw::adc::wait_first_conversion();

    logic::input::apply_calibration(g_storage.radio, g_calib);
    // Prime the jitter and battery filters before the first real sample.
    for (int i = 0; i < 16; ++i) {
        (void)sample_inputs();
    }

    const auto& model = g_storage.active_model;
    apply_crsf_config();

    const bool bind_on_boot = !was_watchdog_reset && (hw::keys::scan_keys() & logic::kKeyBind) != 0;
    g_rf_ok = hw::rf::init(tx_id_from_uid());
    apply_rf_model();
    if (bind_on_boot && g_rf_ok && model.rf_protocol == 0) {
        hw::rf::set_bind_mode(true);
    }

    hw::lcd::init();
    apply_lcd_settings(g_storage.radio);

    using namespace logic::dashboard;
    PageState page_state{};
    g_fb.clear();
    draw_splash(g_fb);
    hw::lcd::flush(g_fb);
    const uint32_t splash_start_ms = time::millis();
    while (time::millis() - splash_start_ms < 1200) {
        chip::watchdog::feed();
        __WFI();
    }

    if (!was_watchdog_reset) {
        run_preflight(model);
    }

    const auto stick_mode = logic::safety::stick_mode_from(g_storage.radio.stick_mode);
    const logic::trim::TrimValues no_trim{};
    uint32_t last_draw_ms = 0;
    uint16_t prev_keys = 0;
    logic::menu::State menu_state{};
    bool ok_armed = false;
    uint32_t ok_press_ms = 0;
    uint32_t last_loop_ms = time::millis();
    uint32_t bind_hold_ms = 0;
    bool bind_was_held = false;
    bool rx_id_dirty = false;

    for (;;) {
        chip::watchdog::feed();

        const auto input = sample_inputs();
        g_battery_mv = input.battery_mv;
        g_keys = hw::keys::scan_keys();

        const uint16_t pressed = static_cast<uint16_t>(g_keys & ~prev_keys);
        prev_keys = g_keys;
        const uint32_t now = time::millis();
        const auto dt_ms = static_cast<uint16_t>(std::min<uint32_t>(now - last_loop_ms, 20));
        last_loop_ms = now;

        const bool afhds = g_rf_ok && g_storage.active_model.rf_protocol == 0;
        if (afhds && hw::rf::is_binding() && (pressed & (logic::kKeyBind | logic::calib::kKeyCancel)) != 0) {
            hw::rf::set_bind_mode(false);
        }

        if (logic::calib::active(g_calib_wizard)) {
            const logic::calib::Input cal_in{pressed, g_keys, input.raw, dt_ms};
            if (logic::calib::update(g_calib_wizard, cal_in, g_storage.radio)) {
                logic::input::apply_calibration(g_storage.radio, g_calib);
                logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
            }
        } else if (menu_state.screen == logic::menu::Screen::Closed) {
            handle_keys(page_state, pressed);
            if ((g_keys & logic::kKeyOk) == 0) {
                ok_armed = true;
            } else if ((pressed & logic::kKeyOk) != 0) {
                ok_press_ms = now;
            }
            if (ok_armed && (g_keys & logic::kKeyOk) != 0 && now - ok_press_ms >= kMenuHoldMs) {
                logic::menu::open(menu_state);
                ok_armed = false;
            }
            if (afhds && !hw::rf::is_binding()) {
                if ((g_keys & logic::kKeyBind) != 0) {
                    bind_hold_ms += dt_ms;
                    if (bind_hold_ms >= kBindHoldMs && !bind_was_held) {
                        hw::rf::set_bind_mode(true);
                        bind_was_held = true;
                    }
                } else {
                    bind_hold_ms = 0;
                    bind_was_held = false;
                }
            }
        } else {
            const auto ev = logic::menu::update(menu_state, logic::menu::Input{pressed, g_keys, now}, g_storage);
            switch (ev.action) {
                case logic::menu::Action::EnterModelSelect: refresh_model_names(); break;
                case logic::menu::Action::SwitchModel:
                    switch_model(ev.index);
                    apply_crsf_config();
                    apply_rf_model();
                    break;
                case logic::menu::Action::SaveActiveModel:
                    logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
                    apply_rf_link();
                    break;
                case logic::menu::Action::SaveRadioConfig:
                    logic::storage::save_radio_config<hw::I2c2Eeprom>(g_storage);
                    apply_lcd_settings(g_storage.radio);
                    break;
                case logic::menu::Action::StartCalibration: logic::calib::start(g_calib_wizard); break;
                case logic::menu::Action::CloseMenu:
                case logic::menu::Action::None: break;
            }
        }

        const bool armed = logic::flight::is_armed(model.arm_switch, input.switches);
        if (g_rf_ok) {
            if (const auto id = hw::rf::take_pending_rx_save()) {
                if (*id != 0 && *id != 0xFFFF'FFFFu && model.rx_id != *id) {
                    g_storage.active_model.rx_id = *id;
                    rx_id_dirty = true;
                }
            }
        }
        if (rx_id_dirty && !armed) {
            logic::storage::save_active_model<hw::I2c2Eeprom>(g_storage);
            rx_id_dirty = false;
        }

        const auto channels =
            logic::flight::rf_channels(input, stick_mode, model, no_trim, g_storage.radio.throttle_trim);
        const bool crsf = model.rf_protocol == 1;
        if (crsf) {
            hw::crsf::update_channels(now, channels, model.crsf_arm_mode != 0, armed);
        } else if (afhds) {
            hw::rf::publish_channels(channels);
        }
        hw::crsf::poll_telemetry(now);

        if (now - last_draw_ms >= 100) {
            g_fb.clear();
            if (logic::calib::active(g_calib_wizard)) {
                logic::calib::render(g_fb, g_calib_wizard, input.raw, g_storage.radio, model.model_type);
            } else if (menu_state.screen == logic::menu::Screen::Closed) {
                render(g_fb, page_state, View{input, g_storage, no_trim, channels, crsf});
            } else {
                logic::menu::render(g_fb, menu_state, g_model_names, g_storage,
                                    logic::menu::Live{channels, input.raw, chip::kProfile.name});
            }
            hw::lcd::flush(g_fb);
            last_draw_ms = now;
        }

        __WFI();
    }
}
