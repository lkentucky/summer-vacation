cmake_minimum_required(VERSION 3.20)

# 默认使用项目内的 ARM GCC 工具链文件
# 配置: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake"
        CACHE FILEPATH "Toolchain file" FORCE)
endif()

project(smartcarmain C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 生成 compile_commands.json 供 clangd / Zed 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# 路径配置
# ---------------------------------------------------------------------------
set(LIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../libraries"
    CACHE PATH "Path to the SEEKFREE libraries folder")

if(NOT EXISTS ${LIB_ROOT})
    message(FATAL_ERROR
        "找不到 libraries 目录: ${LIB_ROOT}\n"
        "请把逐飞开源库的 libraries 文件夹放到与 smartcarmain 同级的目录下。")
endif()

set(USER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/user/src")
set(USER_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/user/inc")
set(CODE_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/code")

# ---------------------------------------------------------------------------
# 源文件
# ---------------------------------------------------------------------------
set(USER_SOURCES
    ${USER_SRC_DIR}/main.c
    ${USER_SRC_DIR}/isr.c
    ${USER_SRC_DIR}/menu.c
    ${USER_SRC_DIR}/mymenu.c
    ${USER_SRC_DIR}/key.c
    ${USER_SRC_DIR}/IMU.c
)

file(GLOB ZFC_SOURCES  "${LIB_ROOT}/zf_common/*.c")
file(GLOB ZFD_SOURCES  "${LIB_ROOT}/zf_driver/*.c")
file(GLOB ZFV_SOURCES  "${LIB_ROOT}/zf_device/*.c")
file(GLOB SDK_SOURCES  "${LIB_ROOT}/sdk/drivers/*.c")
file(GLOB FATFS_SOURCES "${LIB_ROOT}/components/fatfs/source/*.c")

set(SYSTEM_SOURCE "${LIB_ROOT}/sdk/system_mm32f3277g.c")

# 优先使用 GCC 启动文件和链接脚本；若不存在则回退到 MDK 版本并提示
set(STARTUP_ASM_GCC "${LIB_ROOT}/sdk/gcc/startup/startup_mm32f3277g.s")
set(STARTUP_ASM_MDK "${LIB_ROOT}/sdk/mdk/startup_mm32f3277g.s")
if(EXISTS ${STARTUP_ASM_GCC})
    set(STARTUP_ASM ${STARTUP_ASM_GCC})
else()
    set(STARTUP_ASM ${STARTUP_ASM_MDK})
    message(WARNING "未找到 GCC 启动文件，使用 MDK 版本: ${STARTUP_ASM}\n"
                    "若编译失败，请确认 libraries/sdk/gcc/startup/ 下存在对应文件。")
endif()

set(LINKER_SCRIPT_GCC "${LIB_ROOT}/sdk/gcc/linker/mm32f3277g_flash.ld")
set(LINKER_SCRIPT_MDK "${LIB_ROOT}/sdk/mdk/linker/mm32f3277g_flash.scf")
if(EXISTS ${LINKER_SCRIPT_GCC})
    set(LINKER_SCRIPT ${LINKER_SCRIPT_GCC})
elseif(EXISTS ${LINKER_SCRIPT_MDK})
    set(LINKER_SCRIPT ${LINKER_SCRIPT_MDK})
    message(WARNING "未找到 GCC 链接脚本，使用 Keil scatter 文件: ${LINKER_SCRIPT}\n"
                    "GCC 无法直接解析 .scf，请将其转换为 .ld 脚本或提供 GCC 专用链接脚本。")
else()
    message(FATAL_ERROR "找不到 GCC (.ld) 或 MDK (.scf) 链接脚本。")
endif()

set(ALL_SOURCES
    ${USER_SOURCES}
    ${ZFC_SOURCES}
    ${ZFD_SOURCES}
    ${ZFV_SOURCES}
    ${SDK_SOURCES}
    ${FATFS_SOURCES}
    ${SYSTEM_SOURCE}
    ${STARTUP_ASM}
)

# ---------------------------------------------------------------------------
# 目标
# ---------------------------------------------------------------------------
set(ELF_NAME ${PROJECT_NAME}.elf)
add_executable(${ELF_NAME} ${ALL_SOURCES})

# ---------------------------------------------------------------------------
# 头文件路径
# ---------------------------------------------------------------------------
target_include_directories(${ELF_NAME} PRIVATE
    ${USER_INC_DIR}
    ${CODE_DIR}
    ${LIB_ROOT}/zf_common
    ${LIB_ROOT}/zf_device
    ${LIB_ROOT}/zf_driver
    ${LIB_ROOT}/sdk
    ${LIB_ROOT}/sdk/CMSIS/Include
    ${LIB_ROOT}/sdk/drivers
    ${LIB_ROOT}/components/fatfs/source
)

# ---------------------------------------------------------------------------
# 编译选项（对应 MDK 中的 Cads 配置）
# ---------------------------------------------------------------------------
target_compile_options(${ELF_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mthumb-interwork
    -O1
    -g
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    -fshort-enums
    -fshort-wchar
    -fno-exceptions
    -fno-rtti
)

# ---------------------------------------------------------------------------
# 预处理宏
# ---------------------------------------------------------------------------
target_compile_definitions(${ELF_NAME} PRIVATE
    MM32F3273G8P
    USE_STDPERIPH_DRIVER
)

# ---------------------------------------------------------------------------
# 链接选项
# ---------------------------------------------------------------------------
target_link_options(${ELF_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mthumb-interwork
    -T${LINKER_SCRIPT}
    --specs=nosys.specs
    --specs=nano.specs
    -lc
    -lm
    -lnosys
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
)

target_link_libraries(${ELF_NAME} PRIVATE
    ${LIB_ROOT}/zf_device/zf_device_config.a
)

set_target_properties(${ELF_NAME} PROPERTIES
    SUFFIX ".elf"
)

# ---------------------------------------------------------------------------
# 生成 hex / bin
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${ELF_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${ELF_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${ELF_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${ELF_NAME}>
    COMMENT "生成 ${PROJECT_NAME}.hex / ${PROJECT_NAME}.bin"
)

# ---------------------------------------------------------------------------
# 输出提示
# ---------------------------------------------------------------------------
message(STATUS "项目: ${PROJECT_NAME}")
message(STATUS "工具链: ${CMAKE_C_COMPILER}")
message(STATUS "libraries 路径: ${LIB_ROOT}")
message(STATUS "用户源文件: ${USER_SOURCES}")
