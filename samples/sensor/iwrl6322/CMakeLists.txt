# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Register the TI IWRL6322 uDFP SDK as a Zephyr module.
# Must be set before find_package(Zephyr) so ZEPHYR_IWRL6322_UDFP_MODULE_DIR
# is available when the driver's CMakeLists.txt is processed.
list(APPEND ZEPHYR_EXTRA_MODULES $ENV{ZEPHYR_BASE}/../modules/ti/iwrl6322-udfp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ti_iwrl6322)

# Make iwrl6322.h available to the application for
# IWRL6322_MODE_*, IWRL6322_ATTR_*, and IWRL6322_CHAN_*.
zephyr_include_directories(${ZEPHYR_BASE}/drivers/sensor/ti/iwrl6322)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
