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
#include "ferrox/hw/st7567.hpp"
#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/flight.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/trim.hpp"
#include "ferrox/logic/widgets.hpp"
#include "ferrox/time/systick.hpp"

namespace {

// Latest measurements, updated by the main loop (read by later UI/RF code).
volatile uint16_t g_battery_mv = 0;
volatile uint16_t g_keys = 0;

// Radio and active model settings loaded from the external EEPROM.
ferrox::logic::storage::RadioStorage g_storage{};

// Calibration and filter state derived from g_storage.radio.
ferrox::logic::input::Calibration g_calib{};

// Display content is rendered into this buffer and flushed to the ST7567.
ferrox::logic::display::Framebuffer g_fb{};

// Temporary raw-stick debug screen until the LCD UI is ported.
void draw_debug_screen(const ferrox::logic::input::InputState& in) {
    using namespace ferrox::logic;
    g_fb.clear();
    widgets::draw_header(g_fb, "FS-i6X");
    constexpr std::array<std::string_view, 4> kLabels{"CH1", "CH2", "CH3", "CH4"};
    for (std::size_t i = 0; i < kLabels.size(); ++i) {
        const uint16_t raw = in.raw[i];
        const int fill = static_cast<int>(raw) * 40 / 4096;
        widgets::draw_gauge_row(g_fb, 14 + static_cast<int>(i) * 12, kLabels[i], fill, raw);
    }
    ferrox::hw::lcd::flush(g_fb);
}

ferrox::logic::input::BoardSwitches read_board_switches() {
    using ferrox::hw::Pin;
    using ferrox::hw::Port;
    return {.se_high = Pin<Port::C, 12>::read(), .sf_high = Pin<Port::C, 15>::read()};
}

ferrox::logic::input::InputState sample_inputs() {
    return ferrox::logic::input::process(ferrox::hw::adc::read_raw(), read_board_switches(), g_calib);
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
    const uint8_t baud_idx = model.crsf_baud < logic::kBaudRates.size() ? model.crsf_baud : logic::kDefaultBaudIndex;
    hw::crsf::set_enabled(true, baud_idx, model.crsf_half_duplex != 0);

    hw::lcd::init();
    hw::lcd::set_backlight_level(100);

    if (!was_watchdog_reset) {
        run_preflight(model);
    }

    const auto stick_mode = logic::safety::stick_mode_from(g_storage.radio.stick_mode);
    const logic::trim::TrimValues no_trim{};
    uint32_t last_draw_ms = 0;

    for (;;) {
        chip::watchdog::feed();

        const auto input = sample_inputs();
        g_battery_mv = input.battery_mv;
        g_keys = hw::keys::scan_keys();

        const uint32_t now = time::millis();
        if (now - last_draw_ms >= 100) {
            draw_debug_screen(input);
            last_draw_ms = now;
        }
        if (model.rf_protocol == 1) {
            const bool armed = logic::flight::is_armed(model.arm_switch, input.switches);
            const auto channels =
                logic::flight::rf_channels(input, stick_mode, model, no_trim, g_storage.radio.throttle_trim);
            hw::crsf::update_channels(now, channels, model.crsf_arm_mode != 0, armed);
        }
        hw::crsf::poll_telemetry(now);

        __WFI();
    }
}
