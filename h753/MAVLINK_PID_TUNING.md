# MAVLink PID Tuning Protocol - H753 Platform

## Overview

The H753 firmware now supports runtime PID gain adjustment via standard MAVLink parameter protocol (`PARAM_*` messages). This allows real-time tuning of motor controllers without reflashing firmware.

**Supported MAVLink Messages:**
- `PARAM_REQUEST_LIST` (ID 21) - Request all parameters
- `PARAM_REQUEST_READ` (ID 20) - Request specific parameter
- `PARAM_SET` (ID 23) - Set parameter value
- `PARAM_VALUE` (ID 22) - Parameter value response

**No custom MAVLink messages were added** - This implementation uses only standard MAVLink v2 parameter protocol.

---

## Available Parameters

### Motor ID Ranges

The firmware supports the following motor ID ranges:

| Motor Type | ID Range | Max Count |
|-----------|----------|-----------|
| Servo motors | 1-9 | 9 |
| DC motors | 10-15 | 6 |
| RoboMaster motors | 20-29 | 10 |
| RS485 motors | 30-49 | 20 |

### RoboMaster Motor Parameters

For each RoboMaster motor (IDs 20-29):

| Parameter Name | Description | Range | Default (Motor 20) |
|---------------|-------------|-------|-------------------|
| `RM_<ID>_SPD_KP` | Speed control proportional gain | 0.0 - 100.0 | 50.0 |
| `RM_<ID>_SPD_KI` | Speed control integral gain | 0.0 - 10.0 | 0.1 |
| `RM_<ID>_SPD_KD` | Speed control derivative gain | 0.0 - 10.0 | 0.0 |
| `RM_<ID>_ANG_KP` | Angle control proportional gain | 0.0 - 10.0 | 0.1 |
| `RM_<ID>_ANG_KI` | Angle control integral gain | 0.0 - 10.0 | 0.0 |
| `RM_<ID>_ANG_KD` | Angle control derivative gain | 0.0 - 10.0 | 0.0 |

**Examples:**
- `RM_20_SPD_KP` - RoboMaster motor 20, speed Kp
- `RM_25_ANG_KI` - RoboMaster motor 25, angle Ki

### DC Motor Parameters

For each DC motor (IDs 10-15):

| Parameter Name | Description | Range | Default (Motor 10) |
|---------------|-------------|-------|-------------------|
| `DC_<ID>_SPD_KP` | Speed control proportional gain | 0.0 - 10.0 | 0.1 |
| `DC_<ID>_SPD_KI` | Speed control integral gain | 0.0 - 10.0 | 0.05 |
| `DC_<ID>_SPD_KD` | Speed control derivative gain | 0.0 - 10.0 | 0.0 |
| `DC_<ID>_POS_KP` | Position control proportional gain | 0.0 - 10.0 | 0.5 |
| `DC_<ID>_POS_KI` | Position control integral gain | 0.0 - 10.0 | 0.0 |
| `DC_<ID>_POS_KD` | Position control derivative gain | 0.0 - 10.0 | 0.1 |

**Examples:**
- `DC_10_SPD_KP` - DC motor 10, speed Kp
- `DC_15_POS_KD` - DC motor 15, position Kd

---

## MAVLink Protocol Usage

### 1. Request All Parameters

**Request:**
```
PARAM_REQUEST_LIST
  target_system: 1
  target_component: 1
```

**Response:**
Multiple `PARAM_VALUE` messages, one for each parameter:
```
PARAM_VALUE
  param_id: "RM_20_SPD_KP"
  param_value: 50.0
  param_type: MAV_PARAM_TYPE_REAL32 (9)
  param_count: 24
  param_index: 0
```

### 2. Request Specific Parameter

**Request (by name):**
```
PARAM_REQUEST_READ
  target_system: 1
  target_component: 1
  param_id: "RM_20_SPD_KP"
  param_index: -1
```

**Request (by index):**
```
PARAM_REQUEST_READ
  target_system: 1
  target_component: 1
  param_id: ""
  param_index: 5
```

**Response:**
```
PARAM_VALUE
  param_id: "RM_20_SPD_KP"
  param_value: 50.0
  param_type: MAV_PARAM_TYPE_REAL32 (9)
  param_count: 24
  param_index: 5
```

### 3. Set Parameter Value

**Request:**
```
PARAM_SET
  target_system: 1
  target_component: 1
  param_id: "RM_20_SPD_KP"
  param_value: 55.0
  param_type: MAV_PARAM_TYPE_REAL32 (9)
```

**Response:**
```
PARAM_VALUE
  param_id: "RM_20_SPD_KP"
  param_value: 55.0  # Actual value (may be clamped)
  param_type: MAV_PARAM_TYPE_REAL32 (9)
  param_count: 24
  param_index: 5
```

**Important Notes:**
- Values outside valid range are automatically clamped
- Response always contains the **actual current value** (which may differ from requested value if clamped)
- Changes take effect **immediately** without reboot

---

## Python Examples

### Using pymavlink

```python
from pymavlink import mavutil

# Connect to STM32H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# ============================================================================
# Example 1: List all parameters
# ============================================================================
master.mav.param_request_list_send(
    master.target_system,
    master.target_component
)

# Receive all parameters
while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if not msg:
        break
    print(f"{msg.param_id:20} = {msg.param_value:8.4f}")

# ============================================================================
# Example 2: Read specific parameter
# ============================================================================
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'RM_20_SPD_KP',
    -1
)

msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
if msg:
    print(f"Current value: {msg.param_id} = {msg.param_value}")

# ============================================================================
# Example 3: Set parameter
# ============================================================================
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'RM_20_SPD_KP',
    55.0,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# Wait for confirmation
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
if msg:
    print(f"Parameter set: {msg.param_id} = {msg.param_value}")
    if abs(msg.param_value - 55.0) < 0.001:
        print("✓ Value accepted")
    else:
        print(f"⚠ Value clamped to {msg.param_value}")

# ============================================================================
# Example 4: Batch tune RoboMaster motor
# ============================================================================
gains = {
    'RM_20_SPD_KP': 55.0,
    'RM_20_SPD_KI': 0.15,
    'RM_20_SPD_KD': 0.05,
    'RM_20_ANG_KP': 0.12
}

for param_name, value in gains.items():
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    # Wait for confirmation
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg:
        print(f"✓ {param_name} = {msg.param_value}")
```

### Using the Provided Test Script

```bash
# Interactive mode
cd h753
python3 test_pid_tuning.py

# Example session:
>>> list                                      # List all parameters
>>> get RM_20_SPD_KP                          # Get current value
>>> set RM_20_SPD_KP 55.0                     # Set new value
>>> rm 20 speed_kp=55.0 speed_ki=0.15         # Tune multiple gains
>>> dc 10 speed_kp=0.12 pos_kp=0.55           # Tune DC motor
>>> quit
```

---

## QGroundControl Integration

QGroundControl natively supports MAVLink parameters:

1. **Connect** to `udp:192.168.11.4:14550`
2. Navigate to **Vehicle Setup** → **Parameters**
3. **Search** for `RM_` or `DC_` to filter motor parameters
4. **Click** on parameter to edit
5. **Save** - changes apply immediately to STM32

**Features:**
- Real-time parameter editing
- Parameter grouping by prefix
- Value validation (QGC respects min/max ranges)
- Export/import parameter files for backup

---

## Parameter Persistence

### Current Behavior

**Parameters are stored in RAM only:**
- Changes persist during runtime
- Reset to default values on power cycle
- Default values defined in `App/config/motor_config.h`

### Flash Persistence (Future Enhancement)

To enable persistent storage:

1. **Flash sector configuration** in `parameter_manager.c`:
   ```c
   #define PARAM_FLASH_SECTOR_ADDR 0x081E0000  // Adjust based on Flash layout
   ```

2. **Call save function** (via custom MAVLink command or after tuning):
   ```c
   error_code_t err = param_manager_save_to_flash();
   ```

3. **Auto-load on boot** (already implemented in `freertos.c`):
   ```c
   param_manager_load_from_flash();  // Called during initialization
   ```

**Flash storage structure:**
```c
typedef struct {
    uint32_t magic;              // 0x50415241 ("PARA")
    uint16_t param_count;        // Number of parameters
    uint16_t crc16;              // CRC-16 checksum
    float values[MAX_PARAMETERS]; // Parameter values
} param_flash_storage_t;
```

---

## Differences from Standard MAVLink Implementations

### Similarities
✅ Uses standard MAVLink parameter protocol (PARAM_*)
✅ Compatible with QGroundControl and pymavlink
✅ Follows MAVLink parameter naming conventions
✅ Returns actual values (with clamping) in PARAM_VALUE responses

### H753-Specific Features
- **Automatic parameter registration** during motor initialization
- **Value clamping** to safe ranges (prevents invalid PID gains)
- **Immediate effect** - no reboot required for changes
- **Per-motor namespacing** (e.g., `RM_20_*`, `DC_10_*`)

### Not Implemented (Yet)
- ❌ Flash persistence (framework ready, needs Flash HAL integration)
- ❌ Parameter metadata (description, units) via PARAM_EXT_*
- ❌ Parameter groups/categories
- ❌ Parameter change callbacks/notifications

---

## Message Flow Examples

### Typical Tuning Session

```
PC                                      STM32H753
│                                            │
├─ PARAM_REQUEST_LIST ────────────────────> │
│                                            ├─ Retrieve all 24 params
│ <──────────────────── PARAM_VALUE (0/24) ─┤
│ <──────────────────── PARAM_VALUE (1/24) ─┤
│ <──────────────────── PARAM_VALUE (2/24) ─┤
│            ... (all parameters) ...        │
│ <─────────────────── PARAM_VALUE (23/24) ─┤
│                                            │
├─ PARAM_SET: RM_20_SPD_KP = 55.0 ────────> │
│                                            ├─ Validate & clamp value
│                                            ├─ Update motor config
│ <────────────── PARAM_VALUE: 55.0 ────────┤
│                                            │
├─ MOTOR_COMMAND: Motor 20, velocity 50 ──> │
│                                            ├─ Use NEW PID gains!
│                                            ├─ Send CAN command
│ <────────────────────────────────────────┤
```

---

## Troubleshooting

### Parameters Not Showing Up

**Check:**
1. Motors initialized successfully (`motors_initialized = true`)
2. Parameter manager initialized (`param_manager_init()` called)
3. Parameters registered (`param_manager_register_motor_params()` called)

**Debug:**
```python
# Count received parameters
master.mav.param_request_list_send(1, 1)
count = 0
while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if not msg:
        break
    count += 1
print(f"Received {count} parameters")
# Expected: 12 params for RoboMaster + 12 for DC = 24 total
```

### Parameter Set Not Working

**Possible causes:**
- Value outside valid range (check returned value in PARAM_VALUE)
- Parameter name typo (case-sensitive!)
- Motor not initialized
- Network connectivity issues

**Verify:**
```python
# Send and verify
master.mav.param_set_send(1, 1, b'RM_20_SPD_KP', 55.0, 9)
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
if msg:
    print(f"Got: {msg.param_value}, Expected: 55.0")
    print(f"Match: {abs(msg.param_value - 55.0) < 0.001}")
```

### Changes Not Taking Effect

**All parameter changes apply immediately.** If motor behavior doesn't change:
- Verify motor is enabled and receiving commands
- Check control mode (velocity vs position vs duty cycle)
- Monitor motor feedback via CAN (for RoboMaster)
- Test with larger gain changes to see obvious effects

---

## Summary

The H753 firmware now supports **standard MAVLink parameter protocol** for runtime PID tuning:

- ✅ **24 parameters** (6 per RoboMaster motor + 6 per DC motor)
- ✅ **Standard MAVLink messages** (no custom protocol needed)
- ✅ **Immediate effect** (no reboot required)
- ✅ **QGroundControl compatible**
- ✅ **Safe value clamping**
- ✅ **Python test tools included**

**No MAVLink library changes were required** - all functionality uses existing MAVLink v2 parameter messages.

---

**Last Updated:** 2025-11-09
**Firmware Version:** H753UDP.elf
**MAVLink Version:** v2.0
**Compatible Ground Stations:** QGroundControl, pymavlink, Mission Planner
