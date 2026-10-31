#include <cstdint>

#include "ferrox/chip/mcu.hpp"
#include "ferrox/chip/watchdog.hpp"
#include "ferrox/hw/adc.hpp"
#include "ferrox/hw/keys.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/time/systick.hpp"

namespace {

// Latest measurements, updated by the main loop (read by later UI/RF code).
volatile uint16_t g_battery_mv = 0;
volatile uint16_t g_keys = 0;

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

    // Storage (ext_adc flag) is not ported yet, so the 11-channel scan is used.
    hw::adc::init(false);
    chip::watchdog::feed();

    hw::adc::wait_first_conversion();

    for (;;) {
        chip::watchdog::feed();

        const auto raw = hw::adc::read_raw();
        g_battery_mv = logic::battery_mv(raw[10]);
        g_keys = hw::keys::scan_keys();

        __WFI();
    }
}
