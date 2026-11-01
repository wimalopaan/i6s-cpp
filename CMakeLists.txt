cmake_minimum_required(VERSION 3.20)

project(i6s_cpp VERSION 0.27.0 LANGUAGES CXX C ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FERROX_BUILD_TESTS "Build host-side logic tests" ON)
option(USE_USB "Include the USB device stack (removing it saves flash)" ON)

if(CMAKE_CROSSCOMPILING)
    # Short commit hash for the system info screen, as the Rust build.rs does. The reflog changes on every commit, so
    # re-configuring picks up the new hash.
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FERROX_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE FERROX_GIT_RESULT
    )
    if(NOT FERROX_GIT_RESULT EQUAL 0 OR FERROX_GIT_HASH STREQUAL "")
        set(FERROX_GIT_HASH "unknown")
    endif()
    if(EXISTS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)
    endif()

    set(FERROX_SOURCES
        src/main.cpp
        src/isr.cpp
        src/system.cpp
        third_party/cmsis/Startup/startup_stm32f072xb.s
    )

    add_executable(i6s_fw ${FERROX_SOURCES})
    # The assembler gets no optimisation or NDEBUG flags from the MinSizeRel defaults; clang warns that they are unused.
    set(CMAKE_ASM_FLAGS_MINSIZEREL "")
    target_include_directories(i6s_fw PRIVATE
        src
        third_party/cmsis/Core
        third_party/cmsis/Device
    )
    # C++-only flags: the assembler step would otherwise warn about them as unused.
    target_compile_definitions(i6s_fw PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:STM32F072xB>
        "$<$<COMPILE_LANGUAGE:CXX>:FERROX_FW_VERSION=\"${PROJECT_VERSION}\";FERROX_GIT_HASH=\"${FERROX_GIT_HASH}\">"
        $<$<AND:$<BOOL:${USE_USB}>,$<COMPILE_LANGUAGE:CXX>>:FERROX_USE_USB=1>)
    target_compile_options(i6s_fw PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
        $<$<COMPILE_LANGUAGE:CXX>:-ffunction-sections>
        $<$<COMPILE_LANGUAGE:CXX>:-fdata-sections>
        $<$<COMPILE_LANGUAGE:CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
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
    target_compile_definitions(host_logic_tests PRIVATE $<$<BOOL:${USE_USB}>:FERROX_USE_USB=1>)
    target_compile_options(host_logic_tests PRIVATE -Wall -Wextra -fno-exceptions -fno-rtti)
    add_test(NAME host_logic_tests COMMAND host_logic_tests)
endif()
