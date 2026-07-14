# ARM GCC toolchain for Cortex-M3 (MM32F3273G8P)
# 用法: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 交叉编译器前缀，可通过环境变量或 -D 覆盖
set(TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING "GCC toolchain prefix")

find_program(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_AR           ${TOOLCHAIN_PREFIX}ar)
find_program(CMAKE_RANLIB       ${TOOLCHAIN_PREFIX}ranlib)
find_program(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
find_program(CMAKE_OBJDUMP      ${TOOLCHAIN_PREFIX}objdump)
find_program(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size)
find_program(CMAKE_NM           ${TOOLCHAIN_PREFIX}nm)

foreach(tool IN ITEMS CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_ASM_COMPILER
                      CMAKE_AR CMAKE_RANLIB CMAKE_OBJCOPY CMAKE_OBJDUMP CMAKE_SIZE CMAKE_NM)
    if(NOT ${tool})
        message(FATAL_ERROR "找不到 ${tool}，请确认 arm-none-eabi-gcc 已安装并在 PATH 中")
    endif()
endforeach()

# 不使用 host 系统的编译/链接标志
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS   "" CACHE STRING "C flags"   FORCE)
set(CMAKE_CXX_FLAGS "" CACHE STRING "CXX flags" FORCE)
set(CMAKE_ASM_FLAGS "" CACHE STRING "ASM flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "Linker flags" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
