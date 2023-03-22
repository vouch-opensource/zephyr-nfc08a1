cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nfc08a1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(libs_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib")
set(drivers_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers")
set(shared_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../shared")

# This makes sure drivers can find the relevant app config files
zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(app PRIVATE
    src/analogConfigTbl_NFC08A1.c
    src/main.c
    )

set(extra_includes
            "${shared_dir}"
            "${drivers_dir}"
            "src"
            )

target_include_directories(app PUBLIC ${extra_includes})
