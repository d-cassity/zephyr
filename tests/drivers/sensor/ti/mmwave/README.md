# TI mmWave Sensor Test Suite

This directory contains hardware-in-the-loop test applications and pytest suites for testing TI mmWave sensor drivers, specifically the IWRL6432.

## Overview

The test framework consists of three phases:
1. **Common UART Library** - Shared utilities for UART communication
2. **UART Test Applications** - Validate UART communication with real hardware
3. **IWRL6432 Test Application** - Hardware-in-the-loop tests for the IWRL6432 radar

## Directory Structure

```
tests/drivers/sensor/ti/mmwave/
├── common/                     # Shared UART test library
│   ├── uart_test_lib.h
│   └── uart_test_lib.c
├── uart_basic_test/            # Basic UART communication tests
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── src/main.c
│   └── boards/README.txt
├── uart_integrity_test/        # UART data integrity tests
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── src/main.c
│   └── boards/README.txt
├── iwrl6432_test/              # IWRL6432 sensor tests
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── src/
│   │   ├── main.c
│   │   ├── test_init.c
│   │   ├── test_configure.c
│   │   ├── test_data_fetch.c
│   │   ├── test_reconfigure.c
│   │   ├── test_command_handler.c
│   │   ├── test_utils.h
│   │   └── test_utils.c
│   ├── pytest/
│   │   ├── conftest.py
│   │   ├── test_init_deinit.py
│   │   ├── test_configure.py
│   │   ├── test_data_fetch.py
│   │   ├── test_reconfigure.py
│   │   └── test_integration.py
│   └── boards/README.txt
└── pytest/                     # Pytest suite for UART tests
    ├── conftest.py
    ├── test_uart_basic.py
    ├── test_uart_integrity.py
    └── requirements.txt
```

## Building the Applications

### UART Test Applications

```bash
# Build UART basic test
cd zephyr
west build -b <target_board> tests/drivers/sensor/ti/mmwave/uart_basic_test

# Build UART integrity test
west build -b <target_board> tests/drivers/sensor/ti/mmwave/uart_integrity_test
```

### IWRL6432 Test Application

```bash
cd zephyr
west build -b <target_board> tests/drivers/sensor/ti/mmwave/iwrl6432_test
```

**Note:** You may need to create board-specific devicetree overlays in the `boards/` directories. See the README.txt files in each application's boards/ directory for examples.

## Running the Tests

### Prerequisites

1. Install Python dependencies:
```bash
cd tests/drivers/sensor/ti/mmwave/pytest
pip install -r requirements.txt
```

2. Install and configure the autouniflash tool (used for flashing devices)

3. Connect your target board to the PC via UART

### UART Tests

```bash
cd tests/drivers/sensor/ti/mmwave

# Run all UART tests
pytest pytest/ -v \
  --board=<target_board> \
  --device=<device_id> \
  --port=<serial_port> \
  --binary=<path_to_binary>

# Example:
pytest pytest/ -v \
  --board=cc1352r1_launchxl \
  --device=XDS110 \
  --port=COM3 \
  --binary=../../build/zephyr/zephyr.hex
```

### IWRL6432 Tests

```bash
cd tests/drivers/sensor/ti/mmwave/iwrl6432_test

# Run all IWRL6432 tests
pytest pytest/ -v \
  --board=<target_board> \
  --device=<device_id> \
  --port=<serial_port> \
  --binary=<path_to_binary>

# Skip flashing (if device already programmed)
pytest tests\drivers\sensor\ti\mmwave\iwrl6432_mmwave_function_tests\pytest -v --skip-flash  --port=COM4 --pdb

## Test Command Protocol

The test applications use a simple line-based protocol over UART:

### UART Test Commands
```
TEST_INIT                  - Initialize UART
TEST_PING                  - Ping (responds with PONG)
TEST_SEND:<data>          - Send data
TEST_ECHO:<data>          - Echo data back
TEST_INTEGRITY:<data>     - Data integrity test with checksum
TEST_PATTERN:<type>       - Send test pattern (ALPHA, NUMERIC, SPECIAL, etc.)
TEST_MULTI:<count>        - Send multiple messages
```

### IWRL6432 Test Commands
```
TEST_INIT                                - Initialize radar
TEST_DEINIT                              - Deinitialize radar
TEST_CONFIGURE:zone=0,az_min=-30,...    - Configure radar zone
TEST_FETCH_DATA                          - Fetch sensor data
TEST_START                               - Start radar
TEST_STOP                                - Stop radar
TEST_RECONFIGURE:zone=0,az_min=-45,...  - Runtime reconfiguration
TEST_PING                                - Ping (responds with PONG)
```

### Response Format
```
OK:<COMMAND>:<data>        - Success
ERROR:<COMMAND>:code=<n>,msg=<text>  - Error
DATA:<fields>              - Data line
END                        - End of data transmission
PONG                       - Ping response
```

## Test Coverage

### UART Tests
- Basic communication (init, send, receive, echo)
- Data integrity (checksums, patterns, special characters)
- Multiple consecutive messages
- Error handling

### IWRL6432 Tests
- Initialization and deinitialization
- Configuration with zone parameters (azimuth, elevation, range)
- Data fetch (point cloud data)
- Runtime reconfiguration (stop, reconfigure, restart)
- Integration workflows

## Extending the Tests

To add tests for additional IWRL6432 functions:

1. Create `test_<function>.c` in `iwrl6432_test/src/`
2. Add command handler in `test_command_handler.c`
3. Create `test_<function>.py` in `iwrl6432_test/pytest/`
4. Add source file to `CMakeLists.txt`

Example for adding start/stop tests (already included):
- Device side: `src/test_reconfigure.c` (contains start/stop functions)
- Host side: `pytest/test_reconfigure.py`
- Commands: `TEST_START`, `TEST_STOP`

## Troubleshooting

### Device not responding
- Check serial port connection
- Verify baudrate (default: 115200)
- Check that device is properly flashed
- Look for "READY" message on serial console

### Flash failures
- Verify autouniflash tool is installed
- Check device ID is correct
- Ensure device is connected and powered

### Test failures
- Check device logs (printk output)
- Verify sensor hardware is connected (for IWRL6432 tests)
- Check devicetree configuration
- Ensure board overlay is correct for your hardware

## Hardware Requirements

- Target board with UART capability (all tests)
- IWRL6432 mmWave radar module (IWRL6432 tests only)
- UART-to-USB adapter
- autouniflash tool

## Notes

- Tests use real hardware (not emulation)
- UART console must be available for communication
- IWRL6432 tests require actual radar hardware connected
- Some test functions use placeholder implementations and will need integration with actual driver API
