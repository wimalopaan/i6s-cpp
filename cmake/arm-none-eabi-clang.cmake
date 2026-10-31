# Cross toolchain: clang (C++26) + arm-none-eabi binutils/newlib for STM32F072 (Cortex-M0).
# Usage: cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT DEFINED FERROX_CLANG_VERSION)
  set(FERROX_CLANG_VERSION 20)
endif()

set(CMAKE_C_COMPILER clang-${FERROX_CLANG_VERSION})
set(CMAKE_CXX_COMPILER clang++-${FERROX_CLANG_VERSION})
set(CMAKE_ASM_COMPILER clang-${FERROX_CLANG_VERSION})
set(CMAKE_C_COMPILER_TARGET arm-none-eabi)
set(CMAKE_CXX_COMPILER_TARGET arm-none-eabi)
set(CMAKE_ASM_COMPILER_TARGET arm-none-eabi)
set(CMAKE_AR llvm-ar-${FERROX_CLANG_VERSION})
set(CMAKE_RANLIB llvm-ranlib-${FERROX_CLANG_VERSION})
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(FERROX_ARM_SYSROOT_INC /usr/lib/arm-none-eabi/include)
set(FERROX_ARM_CXX_INC ${FERROX_ARM_SYSROOT_INC}/c++/13.2.1)
set(FERROX_MCU_FLAGS "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft")
set(FERROX_MCU_INCLUDES "-isystem ${FERROX_ARM_SYSROOT_INC} -isystem ${FERROX_ARM_CXX_INC} -isystem ${FERROX_ARM_CXX_INC}/arm-none-eabi/thumb/v6-m/nofp")
set(FERROX_MCU_LIBDIRS "-L/usr/lib/arm-none-eabi/newlib/thumb/v6-m/nofp -L/usr/lib/gcc/arm-none-eabi/13.2.1/thumb/v6-m/nofp")
set(CMAKE_CXX_FLAGS_INIT "${FERROX_MCU_FLAGS} ${FERROX_MCU_INCLUDES} -nostdinc++")
set(CMAKE_C_FLAGS_INIT "${FERROX_MCU_FLAGS} ${FERROX_MCU_INCLUDES}")
set(CMAKE_ASM_FLAGS_INIT "${FERROX_MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${FERROX_MCU_LIBDIRS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
