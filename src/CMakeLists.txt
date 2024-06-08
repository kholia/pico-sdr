cmake_minimum_required(VERSION 3.18)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_sdr)
pico_sdk_init()

add_executable(pico_sdr main.c)

add_subdirectory(vendor/pico-stdio-usb-simple)

target_link_libraries(
  pico_sdr
  pico_divider
  pico_multicore
  pico_stdio_usb_simple
  pico_stdlib
  pico_util
  hardware_divider
  hardware_dma
  hardware_pio
  hardware_pwm
  hardware_timer
  hardware_vreg
  hardware_interp
)

pico_add_extra_outputs(pico_sdr)

set_property(TARGET pico_sdr PROPERTY C_STANDARD 11)
target_compile_options(pico_sdr PRIVATE -Wall -Wextra -Wnull-dereference)
target_compile_definitions(pico_sdr PUBLIC PICO_MAX_SHARED_IRQ_HANDLERS=8u)
target_compile_definitions(pico_sdr PUBLIC PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
target_compile_definitions(pico_sdr PUBLIC PICO_STDIO_DEFAULT_CRLF=0)

target_include_directories(pico_sdr PRIVATE include)

#pico_set_binary_type(pico_sdr no_flash)
pico_set_binary_type(pico_sdr copy_to_ram)
