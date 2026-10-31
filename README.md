# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust).

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Status: foundation, CRSF, storage, mixer, input, RC output, LCD, dashboard

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

Not yet ported: LCD menus, telemetry page P5 (placeholder only), AFHDS2A RF status and model page line, buzzer, USB, trainer, trim key state machine and trim footers, throttle curve (linear pass-through for now), flight timer state machine and arm chimes, bind footer.

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
