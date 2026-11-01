# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust).

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Status: foundation, CRSF, storage, mixer, input, RC output, LCD, dashboard, menus, main-menu diagnostics, calibration, dual rate/expo, throttle curve, wing/mixer

Ported so far:
- Chip/CMSIS: system clock (48 MHz), reset flags, DFU entry and jump, UID
- Independent watchdog
- SysTick millisecond timer
- GPIO helpers, ADC scan (DMA), USART2 with BRR logic, keys matrix and DFU check
- Pure constexpr logic (BRR, battery voltage, key bits, SPSC ring) with host tests
- CRSF: CRC-8, channel scaling and RC frame (16 ch, optional ELRS arm flag), byte framer, telemetry parser (link stats, battery, vario, baro, cells, GPS, RPM, temp, attitude, flight mode), 100 Hz TX pacing, 1 s telemetry timeout. Logic in `logic/crsf.hpp` (host-tested), USART/power glue in `hw/crsf.hpp`. Wired into `main.cpp` (telemetry and menu polling; RC output off until mixer exists). Not tested on hardware.

- CRSF parameter menu (API only, no LCD UI): device discovery via ping and device info, parameter loading in folder order with chunks, retries and timeouts, value write (clamped, select cycling), folder navigation, and command state machine. Engine in `logic/crsf_menu.hpp` (host-tested, Loader ~4.8 KB RAM), glue in `hw/crsf.hpp`. Not tested on hardware.

- Storage: external I2C EEPROM driver (I2C2, PB10/PB11, probes 0x50..0x57), A/B radio slots with CRC-16 and fallback, model slots, sanitize rules, and legacy v1–v4 migration. Logic in `logic/storage.hpp` (host-tested, layout checked with static_asserts), driver in `hw/eeprom.hpp`. Loaded at boot in `main.cpp`. Saves are not yet triggered from a UI. Not tested on hardware.

- Mixer (pure logic, host-tested): channel constants, switch model, trim offsets (`logic/trim.hpp`), and the full channel pipeline with 32 matrix lines, wing/tail templates, dual rates and expo, dynamic weights from pots/GVars, heli/arm lockout, timers, and channel reversing (`logic/mixer.hpp`). Not yet wired into the RC output because the arm/safety/idle logic is not ported. Not tested on hardware.

- Input (pure logic, host-tested): stick and pot filtering and normalization with detent and Hall deadbands, ADC mode resolution, 2/3/6-position switch decoding, SE/SF pin decoding, extended ADC switches, and battery EMA filter (`logic/input.hpp`). Calibration comes from the radio storage. Wired into `main.cpp` (PC12/PC15 read as board switches). Not tested on hardware.

- RC output (pure logic, host-tested): startup preflight (throttle position per model type, heli throttle hold, safe switches, cancel key), safe-idle failsafe pulses, stick mode mapping, and the mixer call for RC channels with arm lockout (`logic/safety.hpp`, `logic/flight.hpp`). Wired into `main.cpp`: preflight runs at boot unless the watchdog reset, CRSF uses the model baud rate, and channels are sent when `rf_protocol == 1`. Not tested on hardware.
- LCD (ST7567 128x64, 6800-style bus on GPIOE data bus): CMSIS-only driver with init, backlight PWM (TIM3 CH4) and page flush (`hw/st7567.hpp`). Framebuffer primitives, 4x6 and 6x10 fonts (verified byte-for-byte against the Rust tables), and header, footer, list, gauge, progress and scrollbar widgets are constexpr and host-tested (`logic/framebuffer.hpp`, `logic/fonts.hpp`, `logic/widgets.hpp`). Glyphs, logo, and number formatting (`logic/glyphs.hpp`, `logic/format.hpp`) are host-tested. Pot and channel-gauge widgets are added to `logic/widgets.hpp`.
- Dashboard (pure logic, host-tested): status bar (model name, CRSF/no-RF, battery gauge with low-battery blink), page paging with keys (5 pages), gimbals page P1 (general and aircraft variants, default and adaptive switch/pot layout), channel monitor P2/P3, model page P4, splash (`logic/dashboard.hpp`). `main.cpp` shows the splash for 1.2 s, then renders at 10 Hz. Not tested on hardware.

- Menus (pure logic, host-tested): hold OK for 1.2 s on the dashboard to open the menu. Main menu with 3-row scrolling list and the 12 Rust 12x12 icons, model select (20 models, names read from EEPROM on entry, OK loads a model and reconfigures CRSF), and model setup for name (char editing), type, arm switch and timer. Cancel saves and steps back. Dual rate/expo (main menu item 2): switch (only usable switches, SA..SF), axis (Roll/Pitch/Yaw), Hi/Lo rate and Hi/Lo expo per axis with the Rust clamp ranges. Saved on cancel. Throttle curve (main menu item 3): Pts cycles 5, 7, 9 with the Rust resampling, Crv toggles linear and Catmull-Rom spline, each point 0..100% is edited with Up/Down and OK moves to the next point. The curve is evaluated in `logic/curve.hpp` (constexpr, host-tested, same math as Rust `curve.rs`) and applied to the throttle channel in `logic/flight.hpp`. Saved on cancel and on each commit. Wing & Mixer (main menu item 4): wing template (Normal, Elevon/Delta, V-Tail, Flaperon) cycles on OK, Diff (±100%, shown only for Elevon and Flaperon), a GVars entry (GV1..GV5, ±1000 in steps of 50), and 32 mix lines. Each mix line edits target (CH1..CH18 or Disabled), source, weight source, weight and offset (±100%), switch condition and mode (ADD, MULT, REPL). Unusable sources, weight sources and switch conditions are skipped using the same rules as Rust `storage.rs`. Cancel in a mix line returns to its row without saving. The Wing screen saves on cancel. Logic in `logic/menu.hpp`, names in `logic/labels.hpp` (host-tested). Channel reverse (per-model bit toggle, saved on cancel), aux channels (main menu item 5: CH5..CH18 source per channel, Up/Down steps through the sources usable under the Rust `storage.rs` rules, saved on cancel from the list; the Rust pot/jog auto-assign is not ported), channel monitor (3 pages of 6 channels, wrapping), analog diagnostics (2 pages, raw ADC values), and system info (MCU name, flash and profile count). Deviations from Rust: diagnostics pages 2–4 (external switches, external ADC, EEPROM) are not ported, and system info omits the firmware version and storage status lines. Key repeat after 300 ms, every 70 ms. Logic in `logic/menu.hpp`, shared labels in `logic/labels.hpp`, icon bitmaps in `logic/glyphs.hpp` (host-tested). Wired into `main.cpp`. Not tested on hardware.

- Calibration wizard (pure logic and rendering, host-tested): start from the main menu "Calibration" entry, which closes the menu. Steps: centre (OK captures), limits (move all sticks and configured pots, OK saves once every stick has enough travel), complete screen for 1.2 s. Stick endpoints get a 63/64 margin. Pots are saved only with at least 400 counts of travel. Pot centre and limits are kept in separate arrays from the sticks, which fixes a Rust bug where pot channels 0–3 overwrote the stick endpoints. Saved to EEPROM with `save_radio_config`. Logic and draw code in `logic/calib.hpp`, wiring in `main.cpp`. Deviations from Rust: no buzzer chimes yet, and the complete screen says "EEPROM updated OK" instead of "Flash updated OK". Not tested on hardware.

Radio setup (main menu item 7): 16 rows as in the Rust `setup.rs`. Throttle trim (OFF/IDLE/LINEAR), beeper, tones, backlight timer, backlight level (10%..100%, applied to the LCD at once), contrast (20..50, applied at once), battery warning (4.0..5.0 V), USB mode, PC13 power polarity, external switches, P7 analog, stick mode (MODE 1..4), gimbal type, PPM out and trainer in bits. The Input Setup row opens the Input Modes screen for SA..VRF (6 rows, or 10 with P7 analog) with DEFAULT..INST-TRIM. Each change saves the radio config at once, as in Rust. The menu returns `Action::SaveRadioConfig`, and `main.cpp` saves and applies the LCD settings. The stored LCD values are also applied at boot. Logic in `logic/menu.hpp`, host-tested in `radio_setup_test()`. Not tested on hardware.

Radio setup deviations: the hardware hooks of the Rust `hw/mod.rs` and `buzzer` are not ported. The values are stored, but PPM out and in, PC13 module power, external switches and ADC do not change any pins, the buzzer does not sound, the backlight timeout does not dim the screen, and USB mode does not re-init USB.

AFHDS 2A protocol (pure logic, host-tested): `logic/afhds2a.hpp` (constexpr, no heap, no virtual functions). It covers the 16-channel hopping table from the TX UID, the 38-byte stick packet (14 channels as 12-bit slots plus the 4 extended channels in the upper nibbles), the settings packet (50..400 Hz clamp, PWM/PPM, i-BUS/S.BUS), the failsafe packet (throttle to 988 µs, other channels hold), the four bind steps and the bind confirmation, RX ID capture and TX ID filtering, the downlink requests (settings and failsafe), telemetry sensors (voltage, temperature, RPM, vario, altitude, LQI), and the RSSI formula from the raw A7105 value. Host-tested in `afhds2a_test()`, with the Rust test cases as reference. Not tested on hardware.

AFHDS 2A deviations: the A7105 SPI driver, the GIO2 and TIM16 timer ISRs, the 3.85 ms hopping timer, the triple buffer and the `RF_DRIVER` glue of the Rust `rf/` modules are not ported. The power index is stored, but the A7105 power register values are not written. The link state is not yet wired into the dashboard or model page.
 the physical DR switch while the editor is open does not select the switch yet (Rust `dr_change`).

Wing/mixer deviations: the pot and switch auto-select while moving a control (Rust `pot_change` and `sw_change`) is not ported. Rust returns to the wrong row from a mix line when Diff is shown; the port returns to the mix row it came from.

Not yet ported: diagnostics pages 2–4 and system info firmware/storage lines, model setup rows copy, vario, telemetry, protocol and reset, bind, telemetry page P5 (placeholder only), AFHDS2A RF status and model page line, buzzer (including calibration chimes), USB, trainer, trim key state machine and trim footers, flight timer state machine and arm chimes, bind footer.


## Build

Firmware (clang-20 + arm-none-eabi-gcc 13 newlib for Cortex-M0):

    cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake -DCMAKE_BUILD_TYPE=MinSizeRel
    cmake --build build-fw

Host tests (clang-20 required for C++26):

    cmake -S . -B build-host -DCMAKE_CXX_COMPILER=clang++-20
    cmake --build build-host
    ./build-host/host_logic_tests

## Layout

- `src/ferrox/arch/`: CMSIS include, critical section
- `src/ferrox/chip/`: clock, reset, DFU, watchdog
- `src/ferrox/hw/`: GPIO, ADC, UART, keys, CRSF link
- `src/ferrox/time/`: SysTick millis
- `src/ferrox/logic/`, `src/ferrox/util/`: pure logic and ring buffer (host-testable)
- `third_party/cmsis/`: ARM CMSIS_5 core and ST CMSIS device files (Apache-2.0)

## Notes

- Target is single-core Cortex-M0 (ARMv6-M). `std::atomic` acquire/release and read-modify-write lower to libatomic calls there, so shared ISR state uses `volatile` plus compiler fences.
- Not verified on hardware yet.
