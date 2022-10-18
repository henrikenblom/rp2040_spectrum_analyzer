cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(r2040_spectrum_analyzer)

set(CMAKE_C_STANDARD 11)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(r2040_spectrum_analyzer
	main.c ssd1306.c kiss_fftr.c kiss_fft.c
)

add_custom_target(build-time-make-directory ALL
		COMMAND ${CMAKE_COMMAND} -E make_directory cmake-build-debug)

target_include_directories(r2040_spectrum_analyzer
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/../
)

target_link_libraries(r2040_spectrum_analyzer PRIVATE pico_stdlib hardware_dma hardware_adc hardware_i2c)

pico_enable_stdio_usb(r2040_spectrum_analyzer 1) 
pico_enable_stdio_uart(r2040_spectrum_analyzer 0) 

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(r2040_spectrum_analyzer)

