
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(led_control_webserver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(led_control_webserver led_control_webserver.c)

pico_set_program_name(led_control_webserver "led_control_webserver")
pico_set_program_version(led_control_webserver "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(led_control_webserver 1)
pico_enable_stdio_usb(led_control_webserver 1)

# Add the standard library to the build
target_link_libraries(led_control_webserver
    pico_stdlib
    hardware_gpio
    hardware_adc
    pico_lwip_mqtt
    pico_cyw43_arch_lwip_threadsafe_background 
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4 
    hardware_i2c 
    hardware_pwm
)

# Add the standard include files to the build
target_include_directories(led_control_webserver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/lib/lwip/src/include/arch
    ${PICO_SDK_PATH}/lib/lwip/src/include/lwip
)

target_sources(led_control_webserver PRIVATE
    ${PICO_SDK_PATH}/lib/lwip/src/apps/http/httpd.c
    ${PICO_SDK_PATH}/lib/lwip/src/apps/http/fs.c
)


# Add any user requested libraries

pico_add_extra_outputs(led_control_webserver)
include_directories(${CMAKE_SOURCE_DIR}/libs)