#include <cstdint>

#include "ferrox/chip/mcu.hpp"
#include "ferrox/chip/watchdog.hpp"
#include "ferrox/hw/adc.hpp"
#include "ferrox/hw/crsf.hpp"
#include "ferrox/hw/eeprom.hpp"
#include "ferrox/hw/keys.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/time/systick.hpp"

namespace {

// Latest measurements, updated by the main loop (read by later UI/RF code).
volatile uint16_t g_battery_mv = 0;
volatile uint16_t g_keys = 0;

// Radio and active model settings loaded from the external EEPROM.
ferrox::logic::storage::RadioStorage g_storage{};

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

    // RC channel output stays off until the mixer and storage are ported.
    // Telemetry and the parameter menu run from the default ELRS baud rate.
    hw::crsf::set_enabled(true, logic::kDefaultBaudIndex, false);

    for (;;) {
        chip::watchdog::feed();

        const auto raw = hw::adc::read_raw();
        g_battery_mv = logic::battery_mv(raw[10]);
        g_keys = hw::keys::scan_keys();

        hw::crsf::poll_telemetry(time::millis());

        __WFI();
    }
}
