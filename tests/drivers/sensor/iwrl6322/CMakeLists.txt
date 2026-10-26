# Copyright (c) 2026 Texas Instruments Incorporated
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(iwrl6322_tests)

if(CONFIG_IWRL6322_TEST_DECODER_ONLY)
  # ------------------------------------------------------------------
  # Decoder unit tests (native_sim)
  #
  # The decoder source is compiled directly into the test binary.
  # A minimal stub header placed under src/decoder/mmwaveulink/include/
  # satisfies the #include <mmwaveulink/include/ml_smartfe.h> in
  # iwrl6322_decoder.c without pulling in the full TI library.
  # ------------------------------------------------------------------
  target_sources(app PRIVATE
    src/decoder/main.c
    ${ZEPHYR_BASE}/drivers/sensor/ti/iwrl6322/iwrl6322_decoder.c
  )

  target_include_directories(app PRIVATE
    # Driver's own header (iwrl6322.h)
    ${ZEPHYR_BASE}/drivers/sensor/ti/iwrl6322
    # Stub root: resolves <mmwaveulink/include/ml_smartfe.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/src/decoder
  )

else()
  # ------------------------------------------------------------------
  # Hardware integration tests (cc1352p1_launchxl)
  #
  # The real driver is compiled by Zephyr's build system via
  # CONFIG_IWRL6322=y in boards/cc1352p1_launchxl.conf.
  # ------------------------------------------------------------------
  target_sources(app PRIVATE
    src/integration/main.c
  )

  target_include_directories(app PRIVATE
    ${ZEPHYR_BASE}/drivers/sensor/ti/iwrl6322
  )
endif()
