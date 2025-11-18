# RS485 3-Motor Protocol Update

## Summary

The RS485 motor protocol has been updated to support **3 motors per board** instead of 1. The payload size has increased from 4 bytes to 12 bytes (3 × 4-byte values).

## Protocol Changes

### Previous Protocol (Single Motor)
- **READ Request**: 4 bytes (header + id + CRC)
- **WRITE Command**: 8 bytes (header + id + 4-byte payload + CRC)
- **READ Response**: 8 bytes (header + id + 4-byte payload + CRC)
- **1 motor per board**

### New Protocol (3 Motors)
- **READ Request**: 4 bytes (unchanged - header + id + CRC)
- **WRITE Command**: 16 bytes (header + id + 12-byte payload + CRC)
- **READ Response**: 16 bytes (header + id + 12-byte payload + CRC)
- **3 motors per board**

### Packet Structures

#### Velocity Mode (3 motors)
```c
// WRITE packet - 16 bytes
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xBB
    uint8_t id;           // 1-8 (board ID)
    float target_rps[3];  // 3 × 4 bytes = 12 bytes
    uint16_t crc;         // CRC-16
} rs485_velocity_write_t;

// READ response - 16 bytes
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0x55
    uint8_t id;         // 1-8 (board ID)
    float rps[3];       // 3 × 4 bytes = 12 bytes
    uint16_t crc;       // CRC-16
} rs485_velocity_response_t;
```

#### Position Mode (3 motors)
```c
// WRITE packet - 16 bytes
typedef struct __attribute__((packed)) {
    uint8_t header;             // 0xBB
    uint8_t id;                 // 1-8 (board ID)
    int16_t target_count[3];    // 3 × 2 bytes = 6 bytes
    int16_t target_rotation[3]; // 3 × 2 bytes = 6 bytes
    uint16_t crc;               // CRC-16
} rs485_position_write_t;

// READ response - 16 bytes
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0x56
    uint8_t id;           // 1-8 (board ID)
    int16_t count[3];     // 3 × 2 bytes = 6 bytes
    int16_t rotation[3];  // 3 × 2 bytes = 6 bytes
    uint16_t crc;         // CRC-16
} rs485_position_response_t;
```

## Code Changes

### 1. Configuration Structure (`motor_config.h`)

Added `motor_index` field to `rs485_config_t`:

```c
typedef struct {
    uint8_t id;                         // Motor ID (software, for MAVLink mapping)
    uint8_t rs485_device_id;            // RS485 device ID (1-8, hardware DIP switch)
    uint8_t motor_index;                // Motor index on the board (0-2)  ← NEW
    uint8_t uart_id;                    // UART peripheral ID
    rs485_control_mode_t control_mode;
    float max_rps;
    // ... other fields
} rs485_config_t;
```

### 2. Example Configuration

```c
static const rs485_config_t RS485_CONFIGS[MAX_RS485] = {
    {
        // Motor 30: Board 1, Motor Index 0
        .id = 30,
        .rs485_device_id = 1,
        .motor_index = 0,        // First motor on the board
        .uart_id = 1,
        // ...
    },
    {
        // Motor 31: Board 1, Motor Index 1
        .id = 31,
        .rs485_device_id = 1,    // Same board as motor 30
        .motor_index = 1,        // Second motor on the board
        .uart_id = 1,
        // ...
    },
    {
        // Motor 32: Board 1, Motor Index 2
        .id = 32,
        .rs485_device_id = 1,    // Same board as motors 30-31
        .motor_index = 2,        // Third motor on the board
        .uart_id = 1,
        // ...
    }
};
```

### 3. Controller Implementation (`rs485_controller.c`)

- **Read Status**: Receives 16-byte responses containing data for all 3 motors, extracts the value for the specific `motor_index`
- **Write Command**: Builds 16-byte packets with values for all 3 motors (preserves last known values for other motors)
- **Private Data**: Stores feedback data for all 3 motors as arrays

```c
typedef struct {
    // ...
    uint8_t motor_index;           // Motor index (0-2)
    float current_rps[3];          // All 3 motors
    int16_t current_count[3];      // All 3 motors
    int16_t current_rotation[3];   // All 3 motors
    // ...
} rs485_private_t;
```

## YAML Configuration

### Example: 3 Motors on Same RS485 Board

```yaml
devices:
  # RS485 Board #1 - 3 motors (MAVLink IDs 30-32)
  - id: 30
    type: rs485_motor
    name: conveyor_motor_1
    hardware:
      uart: USART1
      pins:
        tx: PA9
        rx: PA10
    config:
      rs485_motor:
        device_id: 1        # RS485 board ID
        motor_index: 0      # First motor (0-2)
        control_mode: velocity
        max_velocity_rps: 100.0

  - id: 31
    type: rs485_motor
    name: conveyor_motor_2
    hardware:
      uart: USART1        # Same UART
      pins:
        tx: PA9
        rx: PA10
    config:
      rs485_motor:
        device_id: 1        # Same board as motor 30
        motor_index: 1      # Second motor
        control_mode: velocity
        max_velocity_rps: 80.0

  - id: 32
    type: rs485_motor
    name: lift_motor
    hardware:
      uart: USART1        # Same UART
      pins:
        tx: PA9
        rx: PA10
    config:
      rs485_motor:
        device_id: 1        # Same board as motors 30-31
        motor_index: 2      # Third motor
        control_mode: velocity
        max_velocity_rps: 50.0
```

## Code Generation

### Generate C Code from YAML

```bash
cd Lib/mavlink_hal/config

# Activate virtual environment
source venv/bin/activate

# Validate configuration
python generator/cli.py validate examples/h753_rs485_test.yaml

# Generate C code
python generator/cli.py generate examples/h753_rs485_test.yaml \
    --platform stm32 \
    --output generated/rs485_test
```

### Generated Files

- `mavlink_generated_config.h` - Configuration constants and struct definitions
- `mavlink_generated_devices.c` - Device initialization with `motor_index` field
- `mavlink_generated_handlers.c` - MAVLink message handlers
- `mavlink_generated_params.h` - Runtime parameter definitions

### Generated Config Structure

```c
typedef struct {
    uint8_t id;
    const char* name;
    uint8_t uart_port;
    uint8_t rs485_device_id;
    uint8_t motor_index;        /* Motor index on the board (0-2 for 3 motors) */
    rs485_control_mode_t control_mode;
    float max_rps;
} mavlink_gen_rs485_config_t;
```

### Generated Configuration Example

```c
const mavlink_gen_rs485_config_t mavlink_gen_rs485_configs[3] = {
    {
        .id = 30,
        .name = "conveyor_motor_1",
        .uart_port = 1,
        .rs485_device_id = 1,
        .motor_index = 0,           // First motor
        .control_mode = RS485_MODE_VELOCITY,
        .max_rps = 100.000000f,
    },
    {
        .id = 31,
        .name = "conveyor_motor_2",
        .uart_port = 1,
        .rs485_device_id = 1,
        .motor_index = 1,           // Second motor
        .control_mode = RS485_MODE_VELOCITY,
        .max_rps = 80.000000f,
    },
    {
        .id = 32,
        .name = "lift_motor",
        .uart_port = 1,
        .rs485_device_id = 1,
        .motor_index = 2,           // Third motor
        .control_mode = RS485_MODE_VELOCITY,
        .max_rps = 50.000000f,
    },
};
```

## Motor Coordination

### Important Notes

1. **Shared Board State**: When multiple motor controller instances control motors on the same RS485 board, they share the physical communication bus.

2. **Write Behavior**: When one motor receives a command:
   - It reads the last known values for all 3 motors
   - Updates only its own `motor_index` position
   - Sends all 3 values to the board

3. **Read Behavior**: When reading status:
   - All 3 motor values are received in one 16-byte packet
   - Each controller extracts data for its specific `motor_index`

4. **Coordination**: Motors on the same board coordinate writes by maintaining cached values for all motors.

## Testing

### MAVLink Testing with pymavlink

```python
from pymavlink import mavutil

# Connect to H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# Control motor 30 (Board 1, Index 0) - velocity mode, 50 RPS
master.mav.motor_command_send(
    motor_id=30,
    control_mode=1,        # 1 = velocity
    target_value=50.0,     # 50 RPS
    enable=1
)

# Control motor 31 (Board 1, Index 1) - velocity mode, 30 RPS
master.mav.motor_command_send(
    motor_id=31,
    control_mode=1,
    target_value=30.0,
    enable=1
)

# Control motor 32 (Board 1, Index 2) - velocity mode, 20 RPS
master.mav.motor_command_send(
    motor_id=32,
    control_mode=1,
    target_value=20.0,
    enable=1
)
```

## Migration Guide

### For Existing Configurations

If you have existing RS485 motor configurations, you need to:

1. **Add `motor_index` field** to each RS485 motor config:
   - Set to `0` if you have one motor per board
   - Set to `0`, `1`, `2` if you have 3 motors per board

2. **Update YAML configurations** to include `motor_index`:
   ```yaml
   config:
     rs485_motor:
       device_id: 1
       motor_index: 0  # Add this field
   ```

3. **Regenerate code** using the updated templates

4. **Rebuild firmware** with the new packet structures

## Files Modified

### Core Implementation
- `App/motors/rs485_controller.h` - Updated packet structures (4→12 byte payloads)
- `App/motors/rs485_controller.c` - Updated read/write functions for 3-motor arrays
- `App/config/motor_config.h` - Added `motor_index` field to `rs485_config_t`

### Code Generator Templates
- `Lib/mavlink_hal/config/generator/templates/stm32/config.h.j2` - Added `motor_index` to struct
- `Lib/mavlink_hal/config/generator/templates/stm32/devices.c.j2` - Added `motor_index` initialization

### Example Configurations
- `Lib/mavlink_hal/config/examples/h753_rs485_test.yaml` - Updated with 6 motors (2 boards × 3 motors)

## Compatibility

- **Backward Compatible**: If you set `motor_index = 0` for all motors, the system behaves like the old protocol (but with 16-byte packets instead of 8-byte)
- **Hardware Requirement**: The RS485 motor board firmware must also be updated to support the new 16-byte packet protocol

## Date

2025-11-14
