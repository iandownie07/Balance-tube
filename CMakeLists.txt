#1. specify the CMake minimum version
cmake_minimum_required(VERSION 3.12)

#2. include the pico sdk here
include(pico_sdk_import.cmake)

#3. Specify the project name
project(MPU6050)

#4. initialize the pico sdk
pico_sdk_init()

#5. add executable
add_executable(MPU6050
    Combine-MPU&Step.cpp
    MPU6050.cpp
    pico_stepper.cpp
)



pico_enable_stdio_usb(MPU6050 1)
pico_enable_stdio_uart(MPU6050 0)
#6. Add extra outputs to the blink project
pico_add_extra_outputs(MPU6050)

#7. add target libraries
target_link_libraries(MPU6050 pico_stdlib hardware_gpio hardware_i2c pico_multicore pico_time)