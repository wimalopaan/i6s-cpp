# Cross toolchain: clang (C++26) + arm-none-eabi binutils/newlib for STM32F072 (Cortex-M0).
# Usage: cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_ASM_COMPILER clang)
set(CMAKE_C_COMPILER_TARGET arm-none-eabi)
set(CMAKE_CXX_COMPILER_TARGET arm-none-eabi)
set(CMAKE_ASM_COMPILER_TARGET arm-none-eabi)
set(CMAKE_AR llvm-ar)
set(CMAKE_RANLIB llvm-ranlib)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(FERROX_MCU_FLAGS "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft")
separate_arguments(FERROX_MCU_FLAG_LIST UNIX_COMMAND "${FERROX_MCU_FLAGS}")

# Ask arm-none-eabi-g++ for its own include and library paths, so the
# toolchain works with any distribution layout (Debian, Arch, ...).
execute_process(
  COMMAND arm-none-eabi-g++ ${FERROX_MCU_FLAG_LIST} -E -x c++ -v /dev/null
  OUTPUT_QUIET
  ERROR_VARIABLE FERROX_GCC_VERBOSE
)
string(REPLACE "\n" ";" FERROX_GCC_VERBOSE_LINES "${FERROX_GCC_VERBOSE}")
set(FERROX_MCU_INCLUDES "")
set(FERROX_IN_SEARCH_LIST FALSE)
foreach(line IN LISTS FERROX_GCC_VERBOSE_LINES)
  if(line MATCHES "^#include <...> search starts here:")
    set(FERROX_IN_SEARCH_LIST TRUE)
  elseif(line MATCHES "^End of search list")
    set(FERROX_IN_SEARCH_LIST FALSE)
  elseif(FERROX_IN_SEARCH_LIST AND line MATCHES "^ ")
    string(STRIP "${line}" dir)
    # Skip GCC's own builtin headers: clang provides its own.
    if(NOT dir MATCHES "/[0-9][0-9.]*/include(-fixed)?$")
      string(APPEND FERROX_MCU_INCLUDES " -isystem ${dir}")
    endif()
  endif()
endforeach()

execute_process(
  COMMAND arm-none-eabi-g++ ${FERROX_MCU_FLAG_LIST} -print-file-name=libc.a
  OUTPUT_VARIABLE FERROX_LIBC_PATH OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND arm-none-eabi-g++ ${FERROX_MCU_FLAG_LIST} -print-file-name=libgcc.a
  OUTPUT_VARIABLE FERROX_LIBGCC_PATH OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(FERROX_LIBC_DIR "${FERROX_LIBC_PATH}" DIRECTORY)
get_filename_component(FERROX_LIBGCC_DIR "${FERROX_LIBGCC_PATH}" DIRECTORY)
set(FERROX_MCU_LIBDIRS "-L${FERROX_LIBC_DIR} -L${FERROX_LIBGCC_DIR}")
set(CMAKE_CXX_FLAGS_INIT "${FERROX_MCU_FLAGS} ${FERROX_MCU_INCLUDES} -nostdinc++")
set(CMAKE_C_FLAGS_INIT "${FERROX_MCU_FLAGS} ${FERROX_MCU_INCLUDES}")
set(CMAKE_ASM_FLAGS_INIT "${FERROX_MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${FERROX_MCU_LIBDIRS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
