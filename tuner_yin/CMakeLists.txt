# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(../pico_sdk_import.cmake)
include(../pico_extras_import_optional.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(
  -g
  -O0
  -Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  )

# PROJECT DEFINITION

project(main C CXX ASM)

# Add executables
add_executable(main 
  main.c
  ../pitch/_pitch_yin.c
  ../pitch/dsp/fft_algorithm.c
  ../pitch/dsp/flux_window.c
  ../pitch/util/flux_util.c
  ../pitch/util/flux_wave.c
  ../pitch/vector/flux_complex.c
  ../pitch/vector/flux_vector.c
  ../pitch/vector/flux_vectorOp.c
  ../pitch/vector/flux_vectorInt.c
  )

# Add a compilation subdirectory
include_directories(
  ../pitch
  ../pitch/dsp 
  ../pitch/util 
  ../pitch/vector)

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main 1)
pico_enable_stdio_usb(main 0)

# Add the standard library to the build
# Add the standard library to the build
target_link_libraries(
  main
  pico_stdlib
  hardware_adc
  hardware_dma)

# Add the standard include files to the build
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(main)
