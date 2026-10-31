cmake_minimum_required(VERSION 3.20)

project(i6s_cpp LANGUAGES CXX C ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FERROX_BUILD_TESTS "Build host-side logic tests" ON)

if(CMAKE_CROSSCOMPILING)
    set(FERROX_SOURCES
        src/main.cpp
        src/isr.cpp
        src/system.cpp
        third_party/cmsis/Startup/startup_stm32f072xb.s
    )

    add_executable(i6s_fw ${FERROX_SOURCES})
    target_include_directories(i6s_fw PRIVATE
        src
        third_party/cmsis/Core
        third_party/cmsis/Device
    )
    target_compile_definitions(i6s_fw PRIVATE STM32F072xB)
    target_compile_options(i6s_fw PRIVATE
        -fno-exceptions
        -fno-rtti
        -fno-threadsafe-statics
        -fno-use-cxa-atexit
        -ffunction-sections
        -fdata-sections
        -Wall
        -Wextra
    )
    target_link_options(i6s_fw PRIVATE
        -T${CMAKE_SOURCE_DIR}/linker/stm32f072xb.ld
        -nostartfiles
        -nodefaultlibs
        -nostdlib++
        -fuse-ld=lld
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_BINARY_DIR}/i6s_fw.map
        -Wl,--print-memory-usage
    )
    target_link_libraries(i6s_fw PRIVATE -Wl,--start-group c m gcc -Wl,--end-group)

    add_custom_command(TARGET i6s_fw POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:i6s_fw> ${CMAKE_BINARY_DIR}/i6s_fw.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:i6s_fw>
        VERBATIM
    )
elseif(FERROX_BUILD_TESTS)
    enable_testing()
    add_executable(host_logic_tests tests/host/test_logic.cpp)
    target_include_directories(host_logic_tests PRIVATE src)
    target_compile_options(host_logic_tests PRIVATE -Wall -Wextra -fno-exceptions -fno-rtti)
    add_test(NAME host_logic_tests COMMAND host_logic_tests)
endif()
