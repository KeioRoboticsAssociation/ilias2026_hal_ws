# RS485 Motor Control Implementation Summary

## ✅ Completed Components

### 1. RS485 Controller (App/motors/)
- **rs485_controller.h** - Header with protocol definitions
- **rs485_controller.c** - Full implementation with:
  - CRC-16 Modbus calculation
  - READ/WRITE protocol handlers
  - Velocity control mode (float RPS)
  - Position control mode (int16 count + rotation)
  - Auto-detection of motor control mode
  - Watchdog and error handling

### 2. Motor Configuration (App/config/motor_config.h)
- Added `MOTOR_TYPE_RS485` enum
- Added `rs485_control_mode_t` and `rs485_config_t` structures
- Configured 2 RS485 motors:
  - Motor ID 30: RS485 device 1, USART1, velocity control
  - Motor ID 31: RS485 device 2, USART2, position control
- Added `get_rs485_config()` helper function
- Updated `MAX_MOTORS` to include RS485 motors

### 3. Hardware Manager (App/hal/)
- **hardware_manager.h**:
  - Added `hw_uart_t` structure
  - Added UART API functions
  - Updated `hardware_manager_t` to include UART array
- **hardware_manager.c**:
  - Implemented `hw_uart_register()`
  - Implemented `hw_uart_transmit()`
  - Implemented `hw_uart_receive()`
  - Implemented `hw_uart_flush()`

## 🔧 Remaining Steps

### 1. Update Motor Registry (App/motors/motor_registry.c)
Add RS485 motor creation in the factory function:

```c
#include "rs485_controller.h"

// In motor_registry_create_all_motors(), add RS485 case:
case MOTOR_TYPE_RS485: {
    const rs485_config_t* rs485_cfg = get_rs485_config(instance->id);
    if (!rs485_cfg) {
        return ERROR_CONFIG_ERROR;
    }

    // Allocate private data
    rs485_private_t* priv = (rs485_private_t*)malloc(sizeof(rs485_private_t));
    if (!priv) {
        return ERROR_RESOURCE_EXHAUSTED;
    }

    // Create controller
    error_code_t err = rs485_controller_create(instance->id, rs485_cfg, controller, priv);
    if (err != ERROR_OK) {
        free(priv);
        return err;
    }
    break;
}
```

### 2. Update CMakeLists.txt
Add RS485 controller to sources:

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Existing sources...
    App/motors/servo_controller.c
    App/motors/dc_controller.c
    App/motors/robomaster_controller.c
    App/motors/motor_registry.c
    App/motors/rs485_controller.c  # ADD THIS LINE
)
```

### 3. Update FreeRTOS Task (Core/Src/freertos.c)
Register UART handles before motor creation:

```c
// Add extern declarations at top (after hfdcan1)
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// In StartDefaultTask(), after FDCAN registration (around line 318):
  // Register UART handles for RS485 motors
  hw_err = hw_uart_register(1, &huart1);  // USART1
  if (hw_err != ERROR_OK) {
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(60);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(60);
    }
  }

  hw_err = hw_uart_register(2, &huart2);  // USART2
  if (hw_err != ERROR_OK) {
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(60);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(60);
    }
  }
```

### 4. Configure USART1/2 in STM32CubeMX (if not already done)
- Baudrate: 500000 bps (500 kbps)
- Word Length: 8 Bits
- Stop Bits: 1
- Parity: None
- Mode: Asynchronous
- Hardware Flow Control: None

## 📡 MAVLink Control - Custom Protocol

Since RS485 motors (IDs 30-31) are beyond the 8-channel limit of RC_CHANNELS_OVERRIDE, they use a **custom MAVLink message** for control.

### Why Custom MAVLink Message?

**Advantages over RC_CHANNELS_OVERRIDE:**
- ✅ Support unlimited motor IDs (1-255)
- ✅ Direct control mode specification (position, velocity, current, duty cycle)
- ✅ Floating-point precision for target values
- ✅ Explicit enable/disable per motor
- ✅ Works with motors 1-8 too (unified interface)
- ✅ No PWM-to-value conversion needed

**Compared to MANUAL_CONTROL:**
- ✅ More intuitive (direct motor ID addressing)
- ✅ Not limited to 4 axes
- ✅ Designed specifically for motor control

### Custom Message Definition

✅ **COMPLETED** - Custom MAVLink message `MOTOR_COMMAND` (Message ID **12004**) has been added to `Lib/mavlink/robomaster_motor.xml`:

```xml
<!-- In Lib/mavlink/robomaster_motor.xml -->
<message id="12004" name="MOTOR_COMMAND">
  <description>Generic direct motor control command for extended motor IDs and RS485 motors</description>
  <field type="uint8_t" name="motor_id">Motor ID (1-255)</field>
  <field type="uint8_t" name="control_mode">Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)</field>
  <field type="float" name="target_value">Target value (units depend on control_mode)</field>
  <field type="uint8_t" name="enable">Enable motor (0=disable, 1=enable)</field>
</message>
```

**Note**: Message ID 12004 is used instead of 12915 because 12915 was already taken by OPEN_DRONE_ID_MESSAGE_PACK in common.xml.

### Message Mapping

| Motor | Motor ID | Control Mode | Message Field | Range |
|-------|----------|--------------|---------------|-------|
| RS485 #1 | 30 | Velocity (RPS) | `target_value` | -100.0 to +100.0 RPS |
| RS485 #2 | 31 | Position (count+rotation) | `target_value` | Position in radians or counts |

### Handler Implementation

Add to `freertos.c` in `mavlink_message_handler()`:

```c
case MAVLINK_MSG_ID_MOTOR_COMMAND:
{
    if (!motors_initialized) break;

    mavlink_motor_command_t motor_cmd_msg;
    mavlink_msg_motor_command_decode(msg, &motor_cmd_msg);

    motor_command_t cmd = {0};
    cmd.motor_id = motor_cmd_msg.motor_id;
    cmd.mode = (control_mode_t)motor_cmd_msg.control_mode;
    cmd.target_value = motor_cmd_msg.target_value;
    cmd.enable = motor_cmd_msg.enable;
    cmd.timestamp = HAL_GetTick();

    motor_registry_send_command(cmd.motor_id, &cmd);
    break;
}

## 🧪 Testing Procedure

### 1. Hardware Setup
- Connect Ikeya MD motor to USART1 (RS485 transceiver)
- Set DIP switch on motor:
  - Upper 3 bits: 001 (ID=1)
  - Lower 1 bit: 0 (velocity mode)

### 2. Firmware Flash
```bash
cd build
cmake --build .
STM32_Programmer_CLI --connect port=swd --download H753UDP.elf --hardRst --rst --start
```

### 3. Generate Custom MAVLink Messages (✅ COMPLETED)

The custom MOTOR_COMMAND message has already been generated! To regenerate if needed:

```bash
# Navigate to MAVLink directory
cd Lib/mavlink/

# Regenerate MAVLink library from robomaster_motor.xml:
../../mavlink_venv/bin/mavgen.py --lang=C --wire-protocol=2.0 \
    --output=c_library_v2 robomaster_motor.xml

# Go back to h753 directory
cd ../..
```

The generated files are in `Lib/mavlink/c_library_v2/robomaster_motor/mavlink_msg_motor_command.h`

### 4. Test with pymavlink
```python
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()
print("Connected to STM32H753!")

# Control RS485 motor #1 (velocity mode) - 50 RPS forward
master.mav.motor_command_send(
    motor_id=30,           # RS485 motor #1
    control_mode=1,        # 1 = velocity mode
    target_value=50.0,     # 50 RPS
    enable=1               # Enable motor
)
time.sleep(2)

# Control RS485 motor #2 (position mode) - target position
master.mav.motor_command_send(
    motor_id=31,           # RS485 motor #2
    control_mode=0,        # 0 = position mode
    target_value=3.14159,  # π radians (180 degrees)
    enable=1               # Enable motor
)
time.sleep(2)

# Stop motor #1
master.mav.motor_command_send(
    motor_id=30,
    control_mode=1,
    target_value=0.0,      # 0 RPS (stop)
    enable=1
)
```

### Alternative: Use Standard RC_CHANNELS_OVERRIDE for Motors 1-8
For motors with IDs 1-8, you can still use the standard RC_CHANNELS_OVERRIDE:
```python
# Control servo motor ID 1
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1750,  # Channel 1: 1750us = +45 degrees
    0, 0, 0, 0, 0, 0, 0)
```

## ⚠️ Important Notes

1. **RS485 Transceiver Required**: The STM32 UART pins need an RS485 transceiver IC (e.g., MAX485, SN65HVD75) for proper communication.

2. **Direction Control (DE/RE pins)**: RS485 typically requires direction control. The current implementation uses blocking transmit/receive, which works for full-duplex RS485 (4-wire). For half-duplex (2-wire), you'll need to add GPIO control for DE/RE pins.

3. **Baud Rate**: Ensure USART1/2 are configured for **500kbps** in CubeMX.

4. **Encoder Resolution**: Ikeya MD uses 2048 × 4 = 8192 counts/revolution.

5. **Custom MAVLink Message**: RS485 motors (IDs 30-31) use the custom `MOTOR_COMMAND` message (ID 12004) instead of RC_CHANNELS_OVERRIDE. The MAVLink C library has been successfully generated with this custom message in `Lib/mavlink/c_library_v2/robomaster_motor/`.

## 📋 Build Checklist

### Core Implementation
- [x] rs485_controller.h created
- [x] rs485_controller.c created
- [x] motor_config.h updated with RS485 configuration
- [x] hardware_manager.h updated with UART API
- [x] hardware_manager.c updated with UART implementation

### Integration
- [x] motor_registry.c updated with RS485 factory code (lines 87-94, 311-332)
- [x] CMakeLists.txt updated with rs485_controller.c (line 56)
- [x] freertos.c updated with UART registration (lines 323-342)

### Custom MAVLink Protocol
- [x] Add MOTOR_COMMAND message definition to `Lib/mavlink/robomaster_motor.xml` (ID 12004)
- [x] Regenerate MAVLink C library with custom message (in `Lib/mavlink/c_library_v2_custom/robomaster_motor/`)
- [x] Standardize MAVLink to robomaster_motor dialect (updated `App/comm/mavlink_udp.h` and `CMakeLists.txt`)
- [x] Add MOTOR_COMMAND handler to `freertos.c` in `mavlink_message_handler()` (lines 171-189)

### Documentation & Testing
- [x] Create STM32CubeMX configuration guide (`RS485_CUBEMX_CONFIGURATION.md`)
- [x] Create Python test program (`test_rs485_mavlink.py`)
- [x] Create Quick Start Guide (`RS485_QUICKSTART.md`)
- [x] Document MAVLink standardization (`MAVLINK_STANDARDIZATION.md`)

### Final Steps
- [ ] Configure USART1/2 in STM32CubeMX (500kbps) - See `RS485_CUBEMX_CONFIGURATION.md`
- [ ] Build firmware
- [ ] Flash and test with pymavlink - Use `test_rs485_mavlink.py`

## 🔗 Protocol Reference

See `rs485.md` for complete Ikeya MD RS485 protocol specification.

