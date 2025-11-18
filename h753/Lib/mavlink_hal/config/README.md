# MAVLink HAL Configuration System

This directory contains the configuration schema, examples, and validation tools for the MAVLink Hardware Abstraction Layer.

## Directory Structure

```
config/
├── schemas/          # JSON Schema definitions
│   └── device_config.schema.json
├── examples/         # Example configuration files
│   ├── stm32h753_rover.yaml
│   ├── arduino_drone.yaml
│   └── esp32_robot_arm.yaml
├── validation/       # Validation tools
│   └── config_validator.py
├── requirements.txt  # Python dependencies
└── README.md         # This file
```

## Setup

### 1. Create Python Virtual Environment

```bash
# Navigate to the config directory
cd Lib/mavlink_hal/config

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
# On Linux/Mac:
source venv/bin/activate
# On Windows:
# venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Verify Installation

```bash
# Validate the schema itself
python validation/config_validator.py --schema

# Validate all example configurations
python validation/config_validator.py --all

# Validate a specific configuration
python validation/config_validator.py examples/stm32h753_rover.yaml
```

## Usage

### Validating Configuration Files

**Validate schema:**
```bash
python validation/config_validator.py --schema
```

**Validate all examples:**
```bash
python validation/config_validator.py --all
```

**Validate specific file:**
```bash
python validation/config_validator.py examples/stm32h753_rover.yaml
```

**Quiet mode (only show errors):**
```bash
python validation/config_validator.py --all --quiet
```

### Creating New Configurations

1. **Copy an example:**
   ```bash
   cp examples/stm32h753_rover.yaml my_robot.yaml
   ```

2. **Edit the configuration:**
   - Update platform information
   - Configure MAVLink transport
   - Define your devices
   - Set safety limits and failsafes

3. **Validate:**
   ```bash
   python validation/config_validator.py my_robot.yaml
   ```

## Configuration Schema

The configuration schema supports:

### Platform Types
- `stm32` - STM32 microcontrollers (F4, F7, H7, L4)
- `arduino` - Arduino boards (AVR, ARM)
- `esp32` - ESP32 modules
- `linux` - Linux systems (for testing)

### Device Types
- **Motors:**
  - `servo` - PWM servo motors
  - `dc_motor` - DC motors with encoders
  - `bldc_motor` - BLDC motors (ESCs)
  - `stepper` - Stepper motors
  - `robomaster` - DJI RoboMaster motors (M3508, M2006, GM6020)
  - `rs485_motor` - RS485 motors (Ikeya MD protocol)

- **Sensors:**
  - `encoder` - Standalone encoders
  - `imu` - IMU sensors (accelerometer + gyroscope)
  - `gps` - GPS modules
  - `analog_sensor` - Analog sensors (voltage, current, temperature, etc.)
  - `digital_io` - Digital I/O pins

### MAVLink Transport
- **UART:** Serial communication (common on STM32, Arduino)
- **UDP:** Network communication (ESP32, Linux)
- **TCP:** Network communication
- **CAN:** CAN bus communication (STM32 with FDCAN)

### Motor ID Ranges (Convention)
- Servos: 1-9
- DC motors: 10-15
- RoboMaster motors: 20-29
- RS485 motors: 30-49
- Sensors: 100+

## Example Configurations

### STM32H753 Rover
**File:** `examples/stm32h753_rover.yaml`

A comprehensive rover configuration with:
- 2 servo motors (grippers)
- 2 DC motors with encoders (drive wheels)
- 4 RoboMaster motors (turret + flywheels)
- 2 RS485 motors (conveyor)
- IMU and analog sensors

**Use case:** Competition robot with multiple motor types

### Arduino Mega Quadcopter
**File:** `examples/arduino_drone.yaml`

A quadcopter configuration with:
- 4 BLDC motors (ESCs)
- IMU sensor (MPU6050)
- GPS module
- Battery monitoring

**Use case:** Arduino-based flight controller

### ESP32 Robot Arm
**File:** `examples/esp32_robot_arm.yaml`

A 6-DOF robot arm configuration with:
- 6 servo motors (joints + gripper)
- 1 stepper motor (optional precision base)
- IMU for stabilization
- Force sensor in gripper

**Use case:** Educational robot arm with WiFi control

## Configuration Features

### Hardware Pin Mapping
Map devices to physical pins:
```yaml
hardware:
  pins:
    pwm: PA8      # STM32 pin names
    # or
    pwm: 12       # Arduino/ESP32 pin numbers
  timer: TIM1
  channel: 1
```

### PID Control
Configure PID controllers for motors:
```yaml
config:
  dc_motor:
    pid_speed:
      kp: 0.1
      ki: 0.05
      kd: 0.01
      max_integral: 100
      max_output: 1.0
```

### Safety Limits
Set operational limits:
```yaml
limits:
  min_value: -90      # degrees
  max_value: 90       # degrees
  max_speed: 100      # RPM
  max_current_ma: 3000
  max_temperature_c: 80
```

### Failsafe Behavior
Define failsafe actions:
```yaml
failsafe:
  action: hold          # hold, neutral, disable, custom
  timeout_ms: 1000
  custom_value: 0.0     # For custom action
```

## Validation Rules

The schema enforces:

✅ **Type checking** - Correct data types for all fields
✅ **Range validation** - Values within acceptable limits
✅ **Enum validation** - Values from predefined lists
✅ **Required fields** - Essential configuration present
✅ **Format validation** - IP addresses, paths, etc.
✅ **Conditional requirements** - Context-dependent validation

## Extending the Schema

To add new device types:

1. **Add enum to device type:**
   ```json
   "type": {
     "enum": ["servo", "dc_motor", ..., "new_device"]
   }
   ```

2. **Create definition:**
   ```json
   "definitions": {
     "new_device_config": {
       "type": "object",
       "properties": { ... }
     }
   }
   ```

3. **Add to device config:**
   ```json
   "config": {
     "properties": {
       "new_device": {
         "$ref": "#/definitions/new_device_config"
       }
     }
   }
   ```

4. **Validate and test**

## Troubleshooting

### Common Validation Errors

**"Additional properties are not allowed"**
- Remove extra fields not in schema
- Check field name spelling

**"X is a required property"**
- Add missing required fields
- Check parent object structure

**"X is not one of [...]"**
- Use only allowed enum values
- Check for typos in enum values

**"X is not of type 'integer'"**
- Ensure numeric values have correct type
- Remove quotes from numbers in YAML

### Python Environment Issues

**"ModuleNotFoundError: No module named 'jsonschema'"**
```bash
# Activate venv and install dependencies
source venv/bin/activate
pip install -r requirements.txt
```

**"Permission denied"**
```bash
# Make validator executable
chmod +x validation/config_validator.py
```

## Additional Resources

- [JSON Schema Documentation](https://json-schema.org/)
- [YAML Specification](https://yaml.org/)
- [MAVLink Protocol](https://mavlink.io/)
- [MAVLink HAL Documentation](../../README.md)

---

**Date:** 2025-11-11
