# Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

if(DEFINED ZEPHYR_IWRL6322_UDFP_MODULE_DIR)
    set(IWRL6322_UDFP_DIR ${ZEPHYR_IWRL6322_UDFP_MODULE_DIR})
else()
    # Fallback for standard west workspace layout
    set(IWRL6322_UDFP_DIR ${ZEPHYR_BASE}/../modules/ti/iwrl6322-udfp)
endif()

set(IWRL6322_UDFP_ULINK_DIR ${IWRL6322_UDFP_DIR}/mmwaveulink)

zephyr_library_sources(
    iwrl6322.c
    iwrl6322_decoder.c
    ${IWRL6322_UDFP_ULINK_DIR}/mmwaveulink/source/ml_dev.c
    ${IWRL6322_UDFP_ULINK_DIR}/mmwaveulink/source/ml_fe.c
    ${IWRL6322_UDFP_ULINK_DIR}/mmwaveulink/source/ml_smartfe.c
    ${IWRL6322_UDFP_ULINK_DIR}/radarlink/source/rl_host.c
)

# Include paths:
#   ${CMAKE_CURRENT_SOURCE_DIR}    — for iwrl6322.h
#   ${IWRL6322_UDFP_DIR}           — resolves <mmic-binaries/udfp.h>
#   ${IWRL6322_UDFP_ULINK_DIR}     — resolves <mmwaveulink/include/...>,
#                                     <radarlink/include/...>,
#                                     <ml_datatypes.h>, <ml_interface.h>
zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IWRL6322_UDFP_DIR}
    ${IWRL6322_UDFP_ULINK_DIR}
)

# ml_dev.c is a third-party TI source with benign warnings that become
# errors under Zephyr's -Werror build flags.  Suppress them per-file.
# Use the exact same path string as zephyr_library_sources() above so
# that CMake's source-file property lookup matches.
set_property(SOURCE "${IWRL6322_UDFP_ULINK_DIR}/mmwaveulink/source/ml_dev.c"
    APPEND PROPERTY COMPILE_OPTIONS
    "-Wno-missing-braces"
    "-Wno-unused-variable"
)
