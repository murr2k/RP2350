cmake_minimum_required(VERSION 3.16)

# ESP-IDF project for the Waveshare ESP32-S3-Touch-LCD-2.1.
# Work-alike port of the RP2350-LCD-1.28 demo collection in the repository root.
#
# Build:  idf.py set-target esp32s3 && idf.py build
# Flash:  idf.py -p <PORT> flash monitor

set(PROJECT_VER "2.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32s3_lcd_demo C ASM)
