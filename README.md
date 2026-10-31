# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust).

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Status: foundation + CRSF link layer

Ported so far:
- Chip/CMSIS: system clock (48 MHz), reset flags, DFU entry and jump, UID
- Independent watchdog
- SysTick millisecond timer
- GPIO helpers, ADC scan (DMA), USART2 with BRR logic, keys matrix and DFU check
- Pure constexpr logic (BRR, battery voltage, key bits, SPSC ring) with host tests
- CRSF: CRC-8, channel scaling and RC frame (16 ch, optional ELRS arm flag), byte framer, telemetry parser (link stats, battery, vario, baro, cells, GPS, RPM, temp, attitude, flight mode), 100 Hz TX pacing, 1 s telemetry timeout. Logic in `logic/crsf.hpp` (host-tested), USART/power glue in `hw/crsf.hpp`. Not yet wired into `main.cpp`. Not tested on hardware.

Not yet ported: CRSF parameter menu (device discovery, parameter read/write), storage, LCD/UI, buzzer, USB, mixer, trainer.

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
