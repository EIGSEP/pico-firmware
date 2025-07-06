# picohost

Host control and management tools for Raspberry Pi Pico devices running the EIGSEP multi-application firmware.

## Overview

This package provides a Python library for communicating with Raspberry Pi Pico devices running the custom multi-application firmware. It includes:

- **Base communication classes** for serial JSON protocol
- **Specialized device classes** for motor, temperature control, and RF switch applications
- **Test scripts** for development and debugging
- **Device discovery and flashing tools**
- **Example code** showing usage patterns

## Features

- **Automatic device discovery** - Find connected Pico devices
- **JSON command protocol** - Send structured commands and receive responses
- **Background monitoring** - Continuous status updates in separate thread
- **Context manager support** - Easy connection management
- **Extensible design** - Easy to add support for new applications

## Installation

```bash
# Install in development mode
pip install -e .

# Or install with dev dependencies
pip install -e ".[dev]"
```

## Quick Start

### Basic Usage

```python
from picohost import PicoDevice

# Find available devices
ports = PicoDevice.find_pico_ports()
print(f"Found devices: {ports}")

# Connect and send commands
with PicoDevice(ports[0]) as device:
    device.send_command({"cmd": "status"})
    response = device.wait_for_response()
    print(f"Response: {response}")
```

### Motor Control

```python
from picohost import PicoMotor

# Connect to motor device (DIP switch set to 000)
with PicoMotor("/dev/ttyACM0") as motor:
    # Move 1000 steps on both axes
    motor.move(pulses_az=1000, pulses_el=1000)
    
    # Set individual axis positions
    motor.set_azimuth_position(5000)
    motor.set_elevation_position(2000)
```

### Temperature Control

```python
from picohost import PicoPeltier

# Connect to temperature controller (DIP switch set to 001)
with PicoPeltier("/dev/ttyACM0") as peltier:
    # Set temperature to 25C on channel 1
    peltier.set_temperature(25.0, channel=1)
    peltier.enable(channel=1)
    
    # Get current temperature readings
    status = peltier.get_status()
    print(f"Current temp: {status.get('temperature', 'N/A')}°C")
```

### RF Switch Control

```python
from picohost import PicoRFSwitch

# Connect to RF switch device (DIP switch set to 101)
with PicoRFSwitch("/dev/ttyACM0") as switch:
    # Set switch state to 5
    switch.set_switch_state(5)
    
    # Get current switch state
    state = switch.get_switch_state()
    print(f"Current switch state: {state}")
```

## Device Classes

### PicoDevice (Base Class)

The base class provides core functionality:

- **Connection management** - Connect/disconnect to serial ports
- **Command sending** - Send JSON commands with error handling
- **Response parsing** - Parse JSON responses from devices
- **Background monitoring** - Continuous reading of status updates
- **Custom handlers** - Set callbacks for responses

### Specialized Classes

**Currently implemented:**
- **PicoMotor** - Stepper motor control (APP_MOTOR = 0)
- **PicoPeltier** - Temperature control (APP_TEMPCTRL = 1)
- **PicoRFSwitch** - RF switch control (APP_RFSWITCH = 5)

**Firmware-only applications (no Python class yet):**
- **Temperature Monitor** (APP_TEMPMON = 2) - Temperature sensing only
- **IMU Sensor** (APP_IMU = 3) - BNO08x sensor interface
- **Lidar Sensor** (APP_LIDAR = 4) - Lidar sensor interface

## Command Protocol

All communication uses JSON over serial (115200 baud). Commands are sent as JSON objects followed by newline:

```json
{"cmd": "command_name", "param1": value1, "param2": value2}
```

Responses are JSON objects:

```json
{"status": "ok", "result": "success"}
```

## Test Scripts

The package includes several test scripts in `scripts/`:

- **test_motor_pico_v2.py** - Motor control testing
- **test_peltier_v2.py** - Temperature control testing  
- **test_rfswitch_pico_v2.py** - RF switch testing
- **monitor_picos.py** - General device monitoring
- **example_usage.py** - Usage examples

Run tests with:

```bash
# Motor test (requires DIP switch set to 000)
python scripts/test_motor_pico_v2.py /dev/ttyACM0

# Temperature test (requires DIP switch set to 001)
python scripts/test_peltier_v2.py -p /dev/ttyACM0

# RF switch test (requires DIP switch set to 101)
python scripts/test_rfswitch_pico_v2.py /dev/ttyACM0

# Monitor all connected devices
python scripts/monitor_picos.py
```

## Development

### Running Tests

```bash
# Run unit tests
pytest

# Run with coverage
pytest --cov=picohost
```

### Adding New Device Types

1. Create a new class inheriting from `PicoDevice`:

```python
class MyDevice(PicoDevice):
    def my_command(self, param):
        return self.send_command({"cmd": "my_cmd", "param": param})
        
    def get_status(self):
        """Get device status"""
        return self.send_command({"cmd": "status"})
```

2. Add to `__init__.py`:

```python
from .mydevice import MyDevice
__all__.append("MyDevice")
```

3. Create test script following the existing patterns in `scripts/`.

4. Update this README with usage examples.

## Architecture

The firmware implements a multi-app dispatch system where DIP switches (GPIO 2,3,4) select which application runs:

- **APP_MOTOR (0)** - Stepper motor control
- **APP_TEMPCTRL (1)** - Temperature control with Peltier elements
- **APP_TEMPMON (2)** - Temperature monitoring only
- **APP_IMU (3)** - IMU sensor interface
- **APP_LIDAR (4)** - Lidar sensor interface  
- **APP_RFSWITCH (5)** - RF switch control

Each app implements:
- `app_init()` - Initialize hardware
- `app_server()` - Process JSON commands
- `app_op()` - Continuous operations
- `app_status()` - Send status updates

## DIP Switch Configuration

Set GPIO pins 2, 3, 4 to select the application before powering on:

| DIP (2,3,4) | Binary | App ID | Application | Python Class |
|-------------|--------|--------|-------------|---------------|
| 000 | 0 | APP_MOTOR | Motor control | PicoMotor |
| 001 | 1 | APP_TEMPCTRL | Temperature controller | PicoPeltier |
| 010 | 2 | APP_TEMPMON | Temperature monitor | *None* |
| 011 | 3 | APP_IMU | IMU sensor | *None* |
| 100 | 4 | APP_LIDAR | Lidar sensor | *None* |
| 101 | 5 | APP_RFSWITCH | RF switch | PicoRFSwitch |
| 110 | 6 | Reserved | - | - |
| 111 | 7 | Reserved | - | - |

## Hardware Support

- **Raspberry Pi Pico 2** (RP2350) - Primary target
- **Raspberry Pi Pico** (RP2040) - Also supported
- **USB Serial** - CDC ACM interface (VID:0x2E8A, PID:0x0009)
- **Device naming** - Enumerates as PICO_000, PICO_001, etc.

## Flashing Tool

The package includes a flashing tool for managing multiple devices:

```bash
# Flash all connected Picos
python -m picohost.flash_picos --uf2 ../build/pico_multi.uf2

# List available devices
python -m picohost.flash_picos --list
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Run tests and ensure they pass
5. Submit a pull request

**Priority areas for contribution:**
- Python classes for APP_TEMPMON, APP_IMU, and APP_LIDAR
- Additional test scripts
- Enhanced device discovery and management

## License

MIT License - see LICENSE file for details.