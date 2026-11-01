# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust).

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Ported

- Chip: clock (48 MHz), reset flags, DFU entry, UID, watchdog, SysTick
- Peripherals: GPIO, ADC (DMA), USART (DMA, CRSF/half-duplex), keys, LCD (ST7567), buzzer (TIM1), trainer PPM (TIM15)
- CRSF: framing, telemetry parser, RC frames, parameter menu engine
- AFHDS 2A: protocol logic, A7105 driver, radio state machine, ISRs
- Storage: I2C EEPROM, A/B radio slots, model slots, legacy v1–v4 migration
- USB: device stack (HID gamepad, CDC serial with CLI and telemetry), mode switched from Radio Setup
- Mixer, input, RC output, safety/preflight, calibration, trim, flight timer, vario, bind
- Dashboard pages P1–P5 (P5: AFHDS 2A or CRSF diagnostics, or the custom 2×2 telemetry slot page)
- Menus: main menu, model setup (incl. copy), model select, dual rate/expo, throttle curve, wing/mixer, aux channels, diagnostics, system info, radio setup, vario setup, telemetry slot setup, protocol and reset

## Not yet ported

- USB sim mode (trainer)

## Known deviations from Rust

- CRSF: telemetry tiles 12 (VSPD) and 13 (ALT) use CRSF data instead of AFHDS 2A data.
- Trainer: CC2 is not an AF output (PF10 is driven from the ISR, as in Rust).
- Several Rust hardware hooks are omitted because they are always false on this board (manual timer reset, instant-trim click).

## Build

Firmware (clang + arm-none-eabi-gcc for Cortex-M0, MinSizeRel):

    cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake -DCMAKE_BUILD_TYPE=MinSizeRel
    cmake --build build-fw --target i6s_fw

`USE_USB` (default ON) sets `FERROX_USE_USB`, so USB code can be left out to save flash: `-DUSE_USB=OFF`.

Host tests (clang with C++26 support):

    cmake -S . -B build-host -DCMAKE_CXX_COMPILER=clang++
    cmake --build build-host --target host_logic_tests
    ./build-host/host_logic_tests

## Layout

- `src/ferrox/arch/`: CMSIS include, critical section
- `src/ferrox/chip/`: clock, reset, DFU, watchdog
- `src/ferrox/hw/`: drivers (GPIO, ADC, UART, LCD, RF, buzzer, trainer, EEPROM)
- `src/ferrox/time/`: SysTick millis
- `src/ferrox/logic/`, `src/ferrox/util/`: pure logic (host-testable)
- `tests/host/`: host logic tests
- `third_party/cmsis/`: ARM CMSIS_5 and ST device files (Apache-2.0)

## Notes

- Target is single-core Cortex-M0 (ARMv6-M). `std::atomic` read-modify-write lowers to libatomic calls there, so ISR-shared state uses `volatile` plus compiler fences.
- Not verified on hardware yet. Pin and timer assignments come from the Rust source and are not checked against the schematic.
