# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust), plus the FS-i6S variant
(`ferrox-rc/flysky-i6s-rs`), selected with the CMake option `PCB`.

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Ported

- Chip: clock (48 MHz), reset flags, DFU entry, UID, watchdog, SysTick
- Peripherals: GPIO, ADC (DMA), USART (DMA, CRSF/half-duplex), keys, LCD (ST7567), buzzer (TIM1), trainer PPM (TIM15)
- CRSF: framing, telemetry parser, RC frames, parameter menu engine
- AFHDS 2A: protocol logic, A7105 driver, radio state machine, ISRs
- Storage: I2C EEPROM, A/B radio slots, model slots, legacy v1–v4 migration
- USB: device stack (HID gamepad, CDC serial with CLI and telemetry), mode switched from Radio Setup
- USB sim mode (trainer): joystick mode with a host connected mutes the RF link, stops CRSF and PPM out
- Mixer, input, RC output, safety/preflight, calibration, trim, flight timer, vario, bind
- Dashboard pages P1–P5 (P5: AFHDS 2A or CRSF diagnostics, or the custom 2×2 telemetry slot page)
- FS-i6S (`PCB=I6S`): power latch and shutdown (PB15/PB14, LEDs PD10/PD11), FT6236 touch navigation (long centre tap
  opens the menu), rear pots VRA/VRB as menu jog wheels, rear buttons PA9/PA10 (SE/SF, timer reset, instant trim or
  trim modifier), J4 buttons PA13/PA14 as SG/SH, ext ADC PC1..PC5 (VRC..VRG / SWI..SWM), CRSF on USART1 PB6/PB7 with
  module power on PF6, EEPROM fallback to I2C1 (PB8/PB9), I6S EEPROM layout
- Menus: main menu, model setup (incl. copy), model select, dual rate/expo, throttle curve, wing/mixer, aux channels, diagnostics, system info, radio setup, vario setup, telemetry slot setup, protocol setup (CRSF baud, duplex, arm mode, module parameter browser) and reset

## Not yet ported

- None

## Known deviations from Rust

- CRSF: telemetry tiles 12 (VSPD) and 13 (ALT) use CRSF data instead of AFHDS 2A data.
- Trainer: CC2 is not an AF output (PF10 is driven from the ISR, as in Rust).
- FS-i6S:
  - Sanitize keeps the ADC mode InstantTrim (Rust clamps at > 6) and aux sources 41..44 (VRG, SWK..SWM).
  - SE/SF availability follows the rear-button function (`rear_*_func == 0`); Rust checks `ext_switches`, which has
    no setup row on the i6S.
  - Timer arm and mix switches accept all conditions up to SWM (28); Rust stops at 14 / 22.
  - Not ported: the Rust UI drift (Protocol/Vario row order, Vario "Audio Test" row, pot-centre hysteresis) and the
    unused GPIO extension switches PB6/PB7/PF10.

## Build

Firmware (clang + arm-none-eabi-gcc for Cortex-M0, MinSizeRel = -Oz):

    cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake -DCMAKE_BUILD_TYPE=MinSizeRel
    cmake --build build-fw --target i6s_fw

`USE_USB` (default ON) sets `WM_USE_USB`, so USB code can be left out to save flash: `-DUSE_USB=OFF`. Without USB the "USB Mode" row is also removed from Radio Setup.

`PCB` selects the board: `-DPCB=I6X` (default, also when unset) or `-DPCB=I6S`. It sets `WM_PCB_I6S` for firmware and
host tests; board constants live in `src/wm/board.hpp`.

Host tests (clang with C++26 support):

    cmake -S . -B build-host -DCMAKE_CXX_COMPILER=clang++ [-DPCB=I6S]
    cmake --build build-host --target host_logic_tests
    ./build-host/host_logic_tests

## Layout

- `src/wm/arch/`: CMSIS include, critical section
- `src/wm/chip/`: clock, reset, DFU, watchdog
- `src/wm/hw/`: drivers (GPIO, ADC, UART, LCD, RF, buzzer, trainer, EEPROM)
- `src/wm/time/`: SysTick millis
- `src/wm/logic/`, `src/wm/util/`: pure logic (host-testable)
- `tests/host/`: host logic tests
- `third_party/cmsis/`: ARM CMSIS_5 and ST device files (Apache-2.0)

## Notes

- Target is single-core Cortex-M0 (ARMv6-M). `std::atomic` read-modify-write lowers to libatomic calls there, so ISR-shared state uses `volatile` plus compiler fences.
- Not verified on hardware yet. Pin and timer assignments come from the Rust source and are not checked against the schematic.
