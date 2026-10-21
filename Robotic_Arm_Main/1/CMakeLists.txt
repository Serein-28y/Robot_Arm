cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Test3)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# User 目录存放全部手写代码，按层组织：
#   app       应用层：初始化、主循环、UI、诊断页
#   control   控制/算法层：关节、正逆解、插值、命令
#   services  服务层：协议、环形缓冲、按键事件、软定时
#   drivers   驱动层：舵机 PWM、串口、按键 GPIO、OLED
#   config    集中配置：robot_params.h
#   diagnostics 手动标定页：test.c/h
#
# Core/Inc 已由 cmake/stm32cubemx/CMakeLists.txt 加入 include 路径；
# 这里只需把 User/Inc 加进来。头文件统一写成 "control/joint.h" 这种
# 带层路径，CMake 不需要为每个子目录单独配置 include。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Inc
)

# CMake 只编译 target_sources 中列出的 .c 文件。
# 新增、删除或移动手写模块时，必须同步修改这个列表；
# Core/Src 下的 main.c / 中断 / HAL 启动文件由 stm32cubemx 子目录管理。
set(USER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/app/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/app/ui.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/diagnostics/test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/control/joint.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/control/kinematics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/control/motion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/control/cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/services/ringbuf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/services/proto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/services/button.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/services/sched.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/servo_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/uart_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/key_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/display/oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/display/oled_data.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# oled_data.c 是江协科技第三方字模表（GBK 中文索引 + 大数组初始化）。
# 保持原数据不动，单独关闭该文件的警告，避免淹没本项目代码的警告。
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/drivers/display/oled_data.c
    PROPERTIES COMPILE_OPTIONS "-w"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
