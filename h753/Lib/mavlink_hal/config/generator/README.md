# MAVLink HAL Code Generator

Generates platform-specific C/C++ code from YAML/JSON configuration files for MAVLink-enabled robotics systems.

## Features

- **Template-based code generation** using Jinja2
- **Multi-platform support**: STM32, Arduino, ESP32
- **Configuration validation** with detailed error reporting
- **Incremental generation** - only regenerates changed files
- **Code optimization** - platform-specific optimizations
- **Comprehensive CLI** with dry-run and diff modes

## Generated Code

The generator creates the following files for each platform:

### STM32
- `mavlink_generated_config.h` - Configuration constants and type definitions
- `mavlink_generated_devices.c` - Device initialization and management functions
- `mavlink_generated_handlers.c` - MAVLink message handlers
- `mavlink_generated_params.h` - Runtime parameter definitions (PID gains, etc.)

### Arduino
- `mavlink_generated_config.h` - Configuration constants
- `mavlink_generated_devices.cpp` - Device initialization (Arduino style)
- `mavlink_generated_handlers.cpp` - MAVLink message handlers
- `mavlink_generated_params.h` - Parameter definitions

## Installation

The code generator requires Python 3.8+ and uses a virtual environment:

```bash
cd Lib/mavlink_hal/config

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt
```

## Quick Start

### 1. Validate Configuration

```bash
source venv/bin/activate
python generator/cli.py validate examples/stm32h753_rover.yaml
```

### 2. Generate Code

```bash
python generator/cli.py generate examples/stm32h753_rover.yaml \
    --platform stm32 \
    --output build/generated
```

### 3. Use Generated Code

Include the generated files in your project:

```c
#include "mavlink_generated_config.h"

int main() {
    // Initialize all devices from configuration
    mavlink_gen_init_all_devices();

    while (1) {
        // Update all devices (call at MAVLINK_GEN_LOOP_FREQUENCY_HZ)
        mavlink_gen_update_all_devices();

        // Handle MAVLink messages
        mavlink_message_t msg;
        if (mavlink_receive_message(&msg)) {
            mavlink_gen_handle_message(&msg, MAVLINK_COMM_0);
        }
    }
}
```

## CLI Commands

### Generate Code

```bash
python generator/cli.py generate <config_file> --platform <platform> --output <dir> [options]
```

**Options:**
- `--platform, -p` - Target platform (stm32, arduino, esp32) **[required]**
- `--output, -o` - Output directory **[required]**
- `--dry-run, -n` - Show what would be generated without writing files
- `--diff, -d` - Show diff of changes
- `--force, -f` - Force regeneration (disable incremental mode)
- `--no-optimize` - Disable code optimization
- `--validate, -V` - Run additional validation before generation
- `--ignore-warnings, -w` - Ignore warnings during validation
- `--verbose, -v` - Verbose output

**Examples:**

```bash
# Basic generation
python generator/cli.py generate config.yaml -p stm32 -o build/generated

# Dry run to preview changes
python generator/cli.py generate config.yaml -p stm32 -o build/generated --dry-run

# Show diff of changes
python generator/cli.py generate config.yaml -p stm32 -o build/generated --diff

# Force regeneration with validation
python generator/cli.py generate config.yaml -p stm32 -o build/generated --force --validate
```

### Validate Configuration

```bash
python generator/cli.py validate <config_file> [options]
```

**Options:**
- `--verbose, -v` - Verbose output

**Example:**

```bash
python generator/cli.py validate examples/stm32h753_rover.yaml
```

**Validation checks:**
- JSON Schema compliance
- Motor ID uniqueness and range conventions
- Hardware resource conflicts (timers, pins, CAN IDs)
- Pin assignment validity for platform
- MAVLink configuration correctness
- Safety limits and failsafe definitions
- Platform compatibility

### Show Configuration Info

```bash
python generator/cli.py info <config_file>
```

**Example:**

```bash
python generator/cli.py info examples/stm32h753_rover.yaml
```

**Output:**
```
Configuration: stm32h753_rover.yaml
============================================================

Platform: stm32
  MCU: STM32H753ZI
  Clock: 480 MHz

MAVLink:
  System ID: 1
  Transport: udp

Devices: 14
  analog_sensor: 2
  dc_motor: 2
  imu: 1
  robomaster: 4
  rs485_motor: 2
  servo: 2
```

### Clean Generated Files

```bash
python generator/cli.py clean <output_dir> [options]
```

**Options:**
- `--dry-run, -n` - Show what would be removed without deleting

**Example:**

```bash
python generator/cli.py clean build/generated
```

### List Supported Platforms

```bash
python generator/cli.py platforms
```

## Configuration File Format

The generator uses YAML or JSON configuration files that follow the schema defined in `../schemas/device_config.schema.json`.

See the [Configuration Schema README](../README.md) for detailed configuration format documentation.

## Template Structure

Templates are organized by platform:

```
generator/templates/
├── stm32/
│   ├── config.h.j2
│   ├── devices.c.j2
│   ├── handlers.c.j2
│   └── params.h.j2
├── arduino/
│   ├── config.h.j2
│   ├── devices.cpp.j2
│   ├── handlers.cpp.j2
│   └── params.h.j2
└── esp32/
    └── (future)
```

### Template Variables

Templates have access to:

**Configuration data:**
- `config` - Full configuration dictionary
- `platform` - Platform information
- `mavlink` - MAVLink settings
- `devices` - List of all devices
- `devices_by_type` - Devices grouped by type
- `system` - System-wide settings

**Device counts:**
- `device_count` - Total devices
- `servo_count`, `dc_motor_count`, `robomaster_count`, etc.

**Metadata:**
- `generated_timestamp` - ISO timestamp
- `config_file` - Source configuration filename
- `generator_version` - Generator version

### Custom Jinja2 Filters

- `to_c_identifier` - Convert to valid C identifier
- `to_upper` - Convert to uppercase
- `to_hex` - Format as hexadecimal
- `format_float` - Format as C float literal (e.g., `1.234567f`)

## Incremental Generation

The generator supports incremental generation - it only regenerates files that have changed.

**How it works:**
1. Generator calculates SHA256 checksum of generated content
2. Checksum is embedded in generated file header
3. On next run, checksums are compared
4. File is only rewritten if checksum differs

**Benefits:**
- Faster generation
- Avoids unnecessary recompilation
- Preserves file timestamps

**Disable incremental mode:**
```bash
python generator/cli.py generate config.yaml -p stm32 -o build/generated --force
```

## Code Optimization

The generator includes platform-specific optimizations:

### Dead Code Elimination
- Excludes disabled devices
- Removes unused message handlers

### Memory Layout Optimization
- **STM32**: Cache-aligned structures
- **Arduino**: PROGMEM for constants (saves SRAM)
- **ESP32**: Flash storage for read-only data

### Constant Propagation
- Compile-time constant folding
- Reduced runtime calculations

**Disable optimization:**
```bash
python generator/cli.py generate config.yaml -p stm32 -o build/generated --no-optimize
```

## Integration with Build System

### CMake (STM32)

Add generated files to your CMakeLists.txt:

```cmake
# Add generated code
set(GENERATED_DIR ${CMAKE_SOURCE_DIR}/build/generated)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${GENERATED_DIR}/mavlink_generated_devices.c
    ${GENERATED_DIR}/mavlink_generated_handlers.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${GENERATED_DIR}
)

# Add custom command to regenerate if config changes
add_custom_command(
    OUTPUT ${GENERATED_DIR}/mavlink_generated_config.h
    COMMAND python3 ${CMAKE_SOURCE_DIR}/Lib/mavlink_hal/config/generator/cli.py
            generate ${CMAKE_SOURCE_DIR}/robot_config.yaml
            -p stm32 -o ${GENERATED_DIR}
    DEPENDS ${CMAKE_SOURCE_DIR}/robot_config.yaml
    COMMENT "Generating MAVLink HAL code"
)
```

### PlatformIO (Arduino/ESP32)

Add extra script to `platformio.ini`:

```ini
[env:myboard]
platform = ...
board = ...
extra_scripts = pre:generate_code.py
```

Create `generate_code.py`:

```python
Import("env")
import subprocess

subprocess.run([
    "python3", "Lib/mavlink_hal/config/generator/cli.py",
    "generate", "robot_config.yaml",
    "-p", "arduino",
    "-o", "build/generated"
])

env.Append(CPPPATH=["build/generated"])
```

## Examples

### Example 1: Simple Servo Robot (Arduino)

```yaml
platform:
  type: arduino
  mcu:
    model: "Arduino Mega 2560"
    clock_mhz: 16

mavlink:
  system_id: 1
  component_id: 1
  transport:
    type: uart
    uart:
      baudrate: 57600

devices:
  - id: 1
    type: servo
    name: gripper
    hardware:
      pins:
        pwm: 9
    config:
      servo:
        min_pulse_us: 1000
        max_pulse_us: 2000
        neutral_angle: 0.0
```

**Generate:**
```bash
python generator/cli.py generate servo_robot.yaml -p arduino -o src/generated
```

### Example 2: RoboMaster Robot (STM32)

```yaml
platform:
  type: stm32
  mcu:
    model: STM32F446RE
    clock_mhz: 180

mavlink:
  system_id: 1
  component_id: 1
  transport:
    type: uart
    uart:
      port: USART2
      baudrate: 115200

devices:
  - id: 20
    type: robomaster
    name: turret_motor
    hardware:
      can_id: 0x205
    config:
      robomaster:
        motor_type: GM6020
        pid_speed:
          kp: 50.0
          ki: 0.1
          kd: 0.0
        pid_angle:
          kp: 0.5
          ki: 0.0
          kd: 0.1
    limits:
      max_speed: 320
      max_current_ma: 10000
```

**Generate:**
```bash
python generator/cli.py generate robomaster_robot.yaml -p stm32 -o build/generated
```

## Troubleshooting

### Template Rendering Errors

**Problem:** `TypeError: unsupported format string passed to Undefined`

**Cause:** Template references a field that doesn't exist in configuration.

**Solution:** Make sure your configuration includes all required fields, or update the template to handle optional fields.

### Import Errors

**Problem:** `ModuleNotFoundError: No module named 'jinja2'`

**Cause:** Virtual environment not activated or dependencies not installed.

**Solution:**
```bash
source venv/bin/activate
pip install -r requirements.txt
```

### Validation Warnings

**Problem:** Warnings about motor IDs or pin conflicts.

**Cause:** Configuration doesn't follow best practices.

**Solution:** Review warnings and adjust configuration. Use `--ignore-warnings` to generate anyway (not recommended).

## Development

### Adding New Platform

1. Create template directory: `generator/templates/myplatform/`
2. Create templates: `config.h.j2`, `devices.c.j2`, `handlers.c.j2`, `params.h.j2`
3. Add platform to `CodeGenerator.SUPPORTED_PLATFORMS`
4. Add file list to `CodeGenerator.GENERATED_FILES`
5. Test with example configuration

### Adding New Device Type

1. Update schema: `schemas/device_config.schema.json`
2. Add device type definition to schema
3. Update templates to handle new device type
4. Add validation rules in `validators.py`
5. Test generation with new device type

## API Reference

### Python API

You can use the generator programmatically:

```python
from generator import create_generator

# Create generator
gen = create_generator(
    config_file='robot_config.yaml',
    platform='stm32',
    output_dir='build/generated',
    verbose=True
)

# Generate code
generated_files = gen.generate()

# Write files
gen.write_files()

# Get list of generated files
for gen_file in generated_files:
    print(f"Generated: {gen_file.path} ({len(gen_file.content)} bytes)")
```

## License

Part of the MAVLink HAL project.

## Related Documentation

- [Configuration Schema README](../README.md) - Configuration file format
- [MAVLink HAL README](../../README.md) - Overall project documentation
- [Epic2 Task2 Completion](../EPIC2_TASK2_COMPLETION.md) - Implementation details
