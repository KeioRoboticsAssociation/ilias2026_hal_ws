# Motor ID Range Specification - H753 Platform

## Overview

This document defines the motor ID allocation scheme for the H753 STM32 firmware.

## Motor ID Allocation

| Motor Type | ID Range | Max Count | Configuration |
|-----------|----------|-----------|---------------|
| **Servo motors** | 1-9 | 9 | PWM-based position control |
| **DC motors** | 10-15 | 6 | PWM with encoder feedback |
| **RoboMaster motors** | 20-29 | 10 | CAN-based control (GM6020/M3508) |
| **RS485 motors** | 30-49 | 20 | RS485 Ikeya MD protocol |

**Total Capacity:** 45 motors

---

## ID Range Details

### Servo Motors (IDs 1-9)

**Hardware:** Timer PWM channels
**Control Mode:** Position (angle in degrees)
**Example IDs:**
- Motor 1: Front left gripper servo
- Motor 2: Front right gripper servo
- Motor 3-9: Additional servo actuators

**Configuration Location:** `App/config/motor_config.h` → `SERVO_CONFIGS[]`

### DC Motors (IDs 10-15)

**Hardware:** Timer PWM + Encoder
**Control Modes:** Position, Velocity, Duty Cycle
**Example IDs:**
- Motor 10: Left drive wheel
- Motor 11: Right drive wheel
- Motor 12-15: Additional DC motors

**Configuration Location:** `App/config/motor_config.h` → `DC_MOTOR_CONFIGS[]`

### RoboMaster Motors (IDs 20-29)

**Hardware:** FDCAN (CAN bus)
**Motor Types:**
- GM6020: Gimbal motor (voltage control)
- M3508: Drive motor (current control)
- M2006: Small drive motor (current control)

**Example IDs:**
- Motor 20: Turret yaw (GM6020)
- Motor 21: Turret pitch (GM6020)
- Motor 22-29: Drive wheels or other RoboMaster motors

**Configuration Location:** `App/config/motor_config.h` → `ROBOMASTER_CONFIGS[]`

**CAN ID Mapping:**
- GM6020: Feedback IDs 0x205-0x20B
- M3508/M2006: Feedback IDs 0x201-0x208

### RS485 Motors (IDs 30-49)

**Hardware:** UART (USART1, USART2, etc.)
**Protocol:** Ikeya MD RS485 (500 kbps, 8N1)
**Control Modes:** Velocity (RPS) or Position (count + rotation)

**Example IDs:**
- Motor 30: RS485 motor #1 on USART1 (velocity mode)
- Motor 31: RS485 motor #2 on USART2 (position mode)
- Motor 32-49: Additional RS485 motors

**Configuration Location:** `App/config/motor_config.h` → `RS485_CONFIGS[]`

**RS485 Device IDs:** Set via DIP switches (1-8 per UART)

---

## MAVLink Integration

### Motor Control Commands

All motors can be controlled via MAVLink messages:

**RC_CHANNELS_OVERRIDE (Message ID 70):**
- Maps RC channels 1-8 to motor IDs 1-8
- Limited to first 8 motors only

**MOTOR_COMMAND (Message ID 12004):**
- Supports motor IDs 1-255
- Recommended for motors > ID 8
- Supports all control modes

### PID Parameter Naming

**RoboMaster motors (IDs 20-29):**
```
RM_<ID>_SPD_KP, RM_<ID>_SPD_KI, RM_<ID>_SPD_KD  (Speed PID)
RM_<ID>_ANG_KP, RM_<ID>_ANG_KI, RM_<ID>_ANG_KD  (Angle PID)
```

**DC motors (IDs 10-15):**
```
DC_<ID>_SPD_KP, DC_<ID>_SPD_KI, DC_<ID>_SPD_KD  (Speed PID)
DC_<ID>_POS_KP, DC_<ID>_POS_KI, DC_<ID>_POS_KD  (Position PID)
```

**Examples:**
- `RM_20_SPD_KP` - RoboMaster motor 20, speed Kp
- `RM_25_ANG_KI` - RoboMaster motor 25, angle Ki
- `DC_10_SPD_KP` - DC motor 10, speed Kp
- `DC_15_POS_KD` - DC motor 15, position Kd

---

## Configuration Guide

### Adding a New Servo Motor

1. **Choose ID:** Select from range 1-9 (e.g., ID 5)

2. **Configure in `motor_config.h`:**
   ```c
   static const servo_config_t SERVO_CONFIGS[MAX_SERVOS] = {
       // ... existing servos ...
       {
           .id = 5,
           .initial_offset = 0.0f,
           .min_angle = -90.0f,
           .max_angle = 90.0f,
           .pulse_min_us = 500,
           .pulse_max_us = 2500,
           .pulse_neutral_us = 1500,
           // ... other settings ...
       }
   };
   ```

3. **Map to hardware in `MOTOR_INSTANCES`:**
   ```c
   {.id = 5, .type = MOTOR_TYPE_SERVO, .timer_id = 3, .channel = TIM_CHANNEL_1, .enabled = true},
   ```

4. **Register timer in `freertos.c`:**
   ```c
   extern TIM_HandleTypeDef htim3;
   hw_timer_register(3, &htim3);
   ```

### Adding a New RoboMaster Motor

1. **Choose ID:** Select from range 20-29 (e.g., ID 25)

2. **Configure in `motor_config.h`:**
   ```c
   static const robomaster_config_t ROBOMASTER_CONFIGS[MAX_ROBOMASTER] = {
       // ... existing motors ...
       {
           .id = 25,
           .can_id = 0x206,  // GM6020 motor 2
           .angle_kp = 0.1f,
           .speed_kp = 50.0f,  // GM6020: use >10.0 for detection
           // ... PID gains ...
       }
   };
   ```

3. **Map to hardware in `MOTOR_INSTANCES`:**
   ```c
   {.id = 25, .type = MOTOR_TYPE_ROBOMASTER, .timer_id = 0, .channel = 0, .enabled = true},
   ```

### Adding a New RS485 Motor

1. **Choose ID:** Select from range 30-49 (e.g., ID 35)

2. **Set motor DIP switch:**
   - Device ID: 3 (binary: 011)
   - Control mode: Velocity (0) or Position (1)
   - Example DIP: `0110` = ID 3, velocity mode

3. **Configure in `motor_config.h`:**
   ```c
   static const rs485_config_t RS485_CONFIGS[MAX_RS485] = {
       // ... existing motors ...
       {
           .id = 35,                    // MAVLink motor ID
           .rs485_device_id = 3,        // DIP switch ID
           .uart_id = 1,                // USART1
           .control_mode = RS485_MODE_VELOCITY,
           .max_rps = 100.0f,
           // ... other settings ...
       }
   };
   ```

4. **Map to hardware in `MOTOR_INSTANCES`:**
   ```c
   {.id = 35, .type = MOTOR_TYPE_RS485, .timer_id = 0, .channel = 0, .enabled = true},
   ```

---

## Memory Usage

With maximum motor configuration (45 motors):

| Memory Region | Used | Total | Usage |
|--------------|------|-------|-------|
| DTCMRAM | 45 KB | 128 KB | 34.44% |
| RAM_D2 | 281 KB | 288 KB | 95.44% |
| Flash | 149 KB | 2 MB | 7.10% |

**Note:** Actual usage depends on number of motors enabled. Each motor type has different memory footprint.

---

## Best Practices

### ID Assignment Strategy

1. **Group by function:**
   - IDs 1-3: Gripper servos
   - IDs 20-23: Turret motors
   - IDs 30-35: Conveyor RS485 motors

2. **Leave gaps for expansion:**
   - Don't use consecutive IDs if you might add more motors later
   - Example: Use IDs 20, 22, 24 instead of 20, 21, 22

3. **Document in code:**
   ```c
   // Motor ID Assignments
   // 1: Front left gripper servo
   // 2: Front right gripper servo
   // 20: Turret yaw (GM6020)
   // 21: Turret pitch (GM6020)
   // 30: Conveyor motor 1 (RS485)
   ```

### Validation

After adding motors, verify:

1. **Build succeeds:**
   ```bash
   cd build && cmake --build .
   ```

2. **Motor registry creates all motors:**
   - Check LED blink patterns on startup
   - 3 blinks = successful initialization

3. **MAVLink parameters registered:**
   ```bash
   python3 test_pid_tuning.py
   >>> list  # Should show parameters for all active motors
   ```

---

## Troubleshooting

### Motor ID Conflict

**Symptom:** Motor doesn't initialize or behaves unexpectedly

**Solution:** Verify ID uniqueness across all motor types
```bash
# Check for duplicate IDs in motor_config.h
grep "\.id = " App/config/motor_config.h | sort | uniq -d
```

### Exceeded MAX_MOTORS

**Symptom:** Build error or motors not created

**Solution:** Check total count doesn't exceed MAX_MOTORS (45)
```c
// In motor_config.h
#define MAX_MOTORS (MAX_SERVOS + MAX_DC_MOTORS + MAX_ROBOMASTER + MAX_RS485)
// Must be >= number of enabled motors in MOTOR_INSTANCES
```

### PID Parameters Not Showing

**Symptom:** Parameters missing in MAVLink list

**Solution:**
1. Verify motor initialized successfully
2. Check parameter registration in `freertos.c`
3. Ensure motor ID is within valid range

---

## Migration from Old ID Scheme

### Old ID Ranges (Legacy)

| Motor Type | Old Range |
|-----------|-----------|
| Servos | 1-4 |
| DC motors | 10-11 |
| RoboMaster | 20-21 |
| RS485 | 30-31 |

### Migration Steps

1. **No action needed for existing motors** - old IDs are still valid
2. **New motors** can use extended ranges
3. **Update documentation** to reflect actual motor assignments

---

## Related Documentation

- **`motor_config.h`** - Motor configuration definitions
- **`CLAUDE.md`** - Complete system documentation
- **`MAVLINK_PID_TUNING.md`** - PID parameter tuning guide
- **`RS485_QUICKSTART.md`** - RS485 motor setup guide

---

**Last Updated:** 2025-11-09
**Firmware Version:** H753UDP.elf
**Total Motor Capacity:** 45 motors (9 servo + 6 DC + 10 RoboMaster + 20 RS485)
