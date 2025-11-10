# MAVLink Standardization Summary

## Overview

This document explains the MAVLink version standardization that has been implemented for the H753 firmware project.

## MAVLink Directory Structure

```
Lib/mavlink/
├── c_library_v2 → c_library_v2_custom (symlink)
├── c_library_v2_custom/               # ✅ ACTIVE - Current standardized version
│   ├── common/                         # Standard MAVLink messages
│   ├── robomaster_motor/               # Custom motor control messages
│   │   ├── mavlink.h                   # Main include (includes common + custom)
│   │   ├── mavlink_msg_robomaster_motor_control.h  (ID 12000)
│   │   ├── mavlink_msg_robomaster_motor_status.h   (ID 12001)
│   │   ├── mavlink_msg_dc_motor_control.h          (ID 12002)
│   │   ├── mavlink_msg_dc_motor_status.h           (ID 12003)
│   │   └── mavlink_msg_motor_command.h             (ID 12004) ⭐ NEW
│   ├── minimal/
│   └── standard/
├── c_library_v2_robomaster/           # Old version (not used)
├── message_definitions/               # XML source definitions
└── robomaster_motor.xml               # Custom message definitions
```

## Standardized MAVLink Dialect: robomaster_motor

The **robomaster_motor** dialect is now the standardized MAVLink version for this project. It includes:

### Standard Messages (from common.xml)
- HEARTBEAT (ID 0)
- RC_CHANNELS_OVERRIDE (ID 70)
- MANUAL_CONTROL (ID 69)
- PARAM_* messages
- All other standard MAVLink v2 messages

### Custom Motor Control Messages

| Message ID | Message Name | Purpose |
|------------|--------------|---------|
| 12000 | ROBOMASTER_MOTOR_CONTROL | Control RoboMaster motors (duty, speed, position modes) |
| 12001 | ROBOMASTER_MOTOR_STATUS | RoboMaster motor status feedback |
| 12002 | DC_MOTOR_CONTROL | Control DC motors with encoder feedback |
| 12003 | DC_MOTOR_STATUS | DC motor status feedback |
| **12004** | **MOTOR_COMMAND** | **Generic motor control for RS485 and extended IDs** ⭐ |

## MOTOR_COMMAND Message (ID 12004)

The new `MOTOR_COMMAND` message provides a unified interface for controlling motors beyond the 8-channel RC_CHANNELS_OVERRIDE limit.

### Message Fields

```c
typedef struct __mavlink_motor_command_t {
    float target_value;      // Target value (units depend on control_mode)
    uint8_t motor_id;        // Motor ID (1-255)
    uint8_t control_mode;    // 0=position, 1=velocity, 2=current, 3=duty_cycle
    uint8_t enable;          // 0=disable, 1=enable
} mavlink_motor_command_t;
```

### Message Length
- **7 bytes** (compact message)
- CRC: 212

### Control Modes

| Mode Value | Mode Name | Target Value Units | Example |
|------------|-----------|-------------------|---------|
| 0 | Position | Radians or Degrees | 3.14159 (180°) |
| 1 | Velocity | RPS or rad/s | 50.0 (50 RPS) |
| 2 | Current | Amps | 2.5 (2.5A) |
| 3 | Duty Cycle | -1.0 to +1.0 | 0.75 (75% duty) |

## Code Changes

### 1. Updated MAVLink Include (App/comm/mavlink_udp.h)

**Before:**
```c
#include "common/mavlink.h"  // Only standard messages
```

**After:**
```c
#include "robomaster_motor/mavlink.h"  // Standard + custom messages
```

### 2. Updated CMakeLists.txt

Added robomaster_motor include path:
```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Lib/mavlink/c_library_v2
    Lib/mavlink/c_library_v2/robomaster_motor  # Custom motor messages
    Lib/mavlink/c_library_v2/common
)
```

## Benefits of Standardization

✅ **Single MAVLink dialect** - No confusion about which version to use
✅ **All messages available** - Both standard and custom messages in one include
✅ **Extended motor IDs** - Support motors beyond 1-8 with MOTOR_COMMAND
✅ **Unified control** - Single message type works for all motor types
✅ **Forward compatible** - Easy to add more custom messages

## Usage in freertos.c

To use the MOTOR_COMMAND message, add this handler in `mavlink_message_handler()`:

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
```

## Testing with pymavlink

```python
from pymavlink import mavutil

# Connect to STM32H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# Control RS485 motor #30 (velocity mode, 50 RPS)
master.mav.motor_command_send(
    motor_id=30,
    control_mode=1,        # 1 = velocity
    target_value=50.0,     # 50 RPS
    enable=1
)
```

## Regenerating MAVLink Library

If you need to regenerate the MAVLink library after modifying `robomaster_motor.xml`:

```bash
cd Lib/mavlink/
../../mavlink_venv/bin/mavgen.py --lang=C --wire-protocol=2.0 \
    --output=c_library_v2_custom robomaster_motor.xml
cd ../..
```

**Note**: The symlink `c_library_v2 → c_library_v2_custom` ensures the code always uses the custom version.

## Migration Notes

### Old Code Pattern (Motors 1-8 only)
```c
// RC_CHANNELS_OVERRIDE - limited to 8 motors
mavlink_rc_channels_override_t rc;
// Channel 1 → Motor ID 1
// Channel 8 → Motor ID 8
```

### New Code Pattern (Motors 1-255)
```c
// MOTOR_COMMAND - supports motor IDs 1-255
mavlink_motor_command_t cmd;
cmd.motor_id = 30;  // RS485 motor #1
cmd.control_mode = 1;  // Velocity
cmd.target_value = 50.0;  // 50 RPS
cmd.enable = 1;
```

**Both patterns still work!** The system supports:
- RC_CHANNELS_OVERRIDE for motors 1-8 (backward compatible)
- MOTOR_COMMAND for motors 1-255 (new extended control)

## Summary

✅ **Standardized on**: `c_library_v2_custom/robomaster_motor/`
✅ **Active include**: `robomaster_motor/mavlink.h`
✅ **Custom message added**: MOTOR_COMMAND (ID 12004)
✅ **RS485 motors**: Can now be controlled via MAVLink
✅ **Backward compatible**: All existing code still works

---

**Last Updated**: 2025-11-03
**MAVLink Version**: v2.0
**Custom Dialect**: robomaster_motor (version 3)
