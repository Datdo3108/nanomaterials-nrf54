cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(lmp91000_nrf)

target_sources(app PRIVATE src/main.c)

zephyr_library_sources(src/ble.c)
