cmake_minimum_required(VERSION 3.20)

option(USE_GCC "Use ARM GCC instead of Clang for firmware builds" OFF)
if(USE_GCC AND NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake")
endif()

project(i6s_cpp VERSION 0.9 LANGUAGES CXX C ASM)

if(USE_GCC AND (NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    message(FATAL_ERROR "USE_GCC requires an ARM GCC firmware toolchain and a separate build directory")
endif()

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WM_BUILD_TESTS "Build host-side logic tests" ON)
option(USE_USB "Include the USB device stack (removing it saves flash)" ON)
option(USE_MULTISWITCH "Include the I6S MultiSwitch touch dashboard and CRSF SET4 protocol" OFF)

# Target board: I6X (FlySky FS-i6X, default) or I6S (FlySky FS-i6S: touch screen, rear buttons, power latch).
set(PCB "I6X" CACHE STRING "Target board (I6X or I6S)")
set_property(CACHE PCB PROPERTY STRINGS I6X I6S)
if(PCB STREQUAL "")
    set(PCB "I6X")
endif()
string(TOUPPER "${PCB}" PCB)
if(NOT PCB MATCHES "^(I6X|I6S)$")
    message(FATAL_ERROR "PCB must be I6X or I6S, got '${PCB}'")
endif()
if(PCB STREQUAL "I6S")
    set(WM_PCB_I6S 1)
else()
    set(WM_PCB_I6S 0)
endif()
string(TOLOWER "${PCB}_fw" WM_FW_NAME)
message(STATUS "Target board: ${PCB} (firmware: ${WM_FW_NAME})")

if(CMAKE_CROSSCOMPILING)
    # Short commit hash for the system info screen, as the Rust build.rs does. The reflog changes on every commit, so
    # re-configuring picks up the new hash.
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE WM_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE WM_GIT_RESULT
    )
    if(NOT WM_GIT_RESULT EQUAL 0 OR WM_GIT_HASH STREQUAL "")
        set(WM_GIT_HASH "unknown")
    endif()
    if(EXISTS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)
    endif()

    set(WM_SOURCES
        src/main.cpp
        src/isr.cpp
        src/system.cpp
        third_party/cmsis/Startup/startup_stm32f072xb.s
    )

    add_executable(${WM_FW_NAME} ${WM_SOURCES})
    set_property(TARGET ${WM_FW_NAME} APPEND PROPERTY LINK_DEPENDS "${CMAKE_SOURCE_DIR}/linker/stm32f072xb.ld")
    # The assembler gets no optimisation or NDEBUG flags from the MinSizeRel defaults; clang warns that they are unused.
    set(CMAKE_ASM_FLAGS_MINSIZEREL "")
    # -Oz instead of the CMake default -Os for MinSizeRel: about 14 % smaller.
    set(CMAKE_C_FLAGS_MINSIZEREL "-Oz -DNDEBUG")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "-Oz -DNDEBUG")
    target_include_directories(${WM_FW_NAME} PRIVATE
        src
        third_party/cmsis/Core
        third_party/cmsis/Device
    )
    # C++-only flags: the assembler step would otherwise warn about them as unused.
    target_compile_definitions(${WM_FW_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:STM32F072xB>
        "$<$<COMPILE_LANGUAGE:CXX>:WM_FW_VERSION=\"${PROJECT_VERSION}\";WM_GIT_HASH=\"${WM_GIT_HASH}\">"
        $<$<COMPILE_LANGUAGE:CXX>:WM_PCB_I6S=${WM_PCB_I6S}>
        $<$<AND:$<BOOL:${USE_MULTISWITCH}>,$<BOOL:${WM_PCB_I6S}>,$<COMPILE_LANGUAGE:CXX>>:WM_USE_MULTISWITCH=1>
        $<$<AND:$<BOOL:${USE_USB}>,$<COMPILE_LANGUAGE:CXX>>:WM_USE_USB=1>)
    target_compile_options(${WM_FW_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
        $<$<COMPILE_LANGUAGE:CXX>:-ffunction-sections>
        $<$<COMPILE_LANGUAGE:CXX>:-fdata-sections>
        $<$<COMPILE_LANGUAGE:CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    )
    target_link_options(${WM_FW_NAME} PRIVATE
        -T${CMAKE_SOURCE_DIR}/linker/stm32f072xb.ld
        -nostartfiles
        -nodefaultlibs
        -nostdlib++
        -fuse-ld=lld
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_BINARY_DIR}/${WM_FW_NAME}.map
        -Wl,--print-memory-usage
    )
    target_link_libraries(${WM_FW_NAME} PRIVATE -Wl,--start-group c m gcc -Wl,--end-group)

    add_custom_command(TARGET ${WM_FW_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${WM_FW_NAME}> ${CMAKE_BINARY_DIR}/${WM_FW_NAME}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${WM_FW_NAME}>
        VERBATIM
    )
elseif(WM_BUILD_TESTS)
    enable_testing()
    add_executable(host_logic_tests tests/host/test_logic.cpp)
    target_include_directories(host_logic_tests PRIVATE src)
    target_compile_definitions(host_logic_tests PRIVATE WM_PCB_I6S=${WM_PCB_I6S}
        $<$<BOOL:${USE_USB}>:WM_USE_USB=1>
        $<$<AND:$<BOOL:${USE_MULTISWITCH}>,$<BOOL:${WM_PCB_I6S}>>:WM_USE_MULTISWITCH=1>)
    target_compile_options(host_logic_tests PRIVATE -Wall -Wextra -fno-exceptions -fno-rtti)
    add_test(NAME host_logic_tests COMMAND host_logic_tests)
endif()
