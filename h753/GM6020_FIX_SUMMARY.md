# GM6020 CAN Communication Fix Summary

## Problem Identified

The GM6020 motor could not be detected because **CAN receive functionality was not implemented**. While the firmware could send CAN commands to the motor, it had no way to receive feedback messages.

### Missing Components

1. **No CAN RX interrupt enabled** - `HAL_FDCAN_ActivateNotification()` was never called
2. **No RX callback** - `HAL_FDCAN_RxFifo0Callback()` was not implemented
3. **No interrupt handlers** - FDCAN interrupt handlers were missing in `stm32h7xx_it.c`

## Changes Made

### 1. Core/Src/freertos.c

**Added CAN RX interrupt activation** (line 435-437):
```c
// Enable FDCAN RX FIFO0 interrupt to receive motor feedback
if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
  Error_Handler();
}
```

**Added CAN filter expansion** (line 424-425):
```c
sFilterConfig.FilterID1 = 0x201;  // M3508 motor 1 / GM6020 range start
sFilterConfig.FilterID2 = 0x20B;  // GM6020 motor 7
```
- Previous filter: 0x201-0x208 (M3508 only)
- New filter: 0x201-0x20B (both M3508 and GM6020)

**Added RX callback function** (lines 282-315):
```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  // Receives CAN messages from GM6020/M3508
  // Routes them to appropriate motor controllers
}
```

**Added robomaster_controller.h include** (line 35)

### 2. Core/Src/stm32h7xx_it.c

**Added FDCAN interrupt handlers** (lines 228-239):
```c
void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}
```

**Added external FDCAN handle** (line 62)

### 3. Core/Inc/stm32h7xx_it.h

**Added function prototypes** (lines 60-61):
```c
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
```

## How It Works Now

1. **GM6020 sends feedback** → CAN bus at 0x205 (motor 1)
2. **FDCAN hardware** receives message into RX FIFO0
3. **Interrupt triggers** → `FDCAN1_IT0_IRQHandler()` called
4. **HAL processes** → `HAL_FDCAN_IRQHandler()` → `HAL_FDCAN_RxFifo0Callback()`
5. **Callback routes** message to RoboMaster controller
6. **Motor state updated** with position, velocity, current, temperature

## Testing Your GM6020

### Current Motor Configuration

According to `App/config/motor_config.h`:
- **Motor ID**: 20 (MAVLink motor ID)
- **CAN ID**: 0x205 (GM6020 motor 1 feedback ID)
- **Control Mode**: Voltage control (-30000 to +30000)

### Method 1: Send Test Command via MAVLink

```python
from pymavlink import mavutil

# Connect to H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()
print("Connected!")

# Send duty cycle command (0.2 = 20% power)
master.mav.motor_command_send(
    motor_id=20,          # RoboMaster motor ID
    control_mode=3,       # 3 = DUTY_CYCLE mode
    target_value=0.2,     # 20% power (maps to 6000 voltage)
    enable=1
)
print("Command sent to GM6020!")

# Monitor motor parameters
import time
for i in range(20):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        b'RM_20_SPD_KP',
        -1
    )
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        print(f"RM_20_SPD_KP = {msg.param_value}")
    time.sleep(0.5)
```

### Method 2: Using QGroundControl

1. **Connect**: UDP connection to `192.168.11.4:14550`
2. **Verify heartbeat**: Should see "Connected" status
3. **Navigate**: Vehicle Setup → Parameters
4. **Check GM6020 params**:
   - `RM_20_SPD_KP` (should be 50.0)
   - `RM_20_SPD_KI` (should be 0.1)
   - `RM_20_SPD_KD` (should be 0.0)
   - `RM_20_ANG_KP` (should be 0.1)
   - `RM_20_ANG_KI` (should be 0.0)
   - `RM_20_ANG_KD` (should be 0.0)

### Method 3: Send Direct CAN Messages

The GM6020 motor 1 expects:
- **Receive on**: 0x1FF (control commands, bytes 0-1 for motor 1)
- **Transmit on**: 0x205 (feedback: angle, speed, current, temp)

Firmware will now receive these 0x205 messages and update motor state!

## Checking Motor Status

### Via Motor State (Future Enhancement)

Currently, motor feedback is processed but not sent back via MAVLink. To check if the motor is responding, you can:

1. **Add debug output** in `robomaster_process_can_message()` in `App/motors/robomaster_controller.c:396`
2. **Monitor LED patterns** - GREEN LED toggles at 1Hz when heartbeat is sent
3. **Use a logic analyzer** on CAN bus to verify 0x205 messages from GM6020

### Expected CAN Messages

**From STM32 to GM6020** (0x1FF):
```
ID: 0x1FF
Data: [motor1_H, motor1_L, motor2_H, motor2_L, ...]
```

**From GM6020 to STM32** (0x205):
```
ID: 0x205
Data: [angle_H, angle_L, speed_H, speed_L, current_H, current_L, temp, 0]
```

## Build and Flash

```bash
cd /home/imanoob/ilias2026_ws/ilias2026_hal_ws/h753/build
cmake --build .
STM32_Programmer_CLI --connect port=swd --download H753UDP.elf --hardRst --rst --start
```

## Verification Steps

1. **Flash new firmware** to H753
2. **Check LED1 (GREEN)** - should blink at 1Hz (heartbeat)
3. **Connect via MAVLink** - verify UDP connection works
4. **Send motor command** - use pymavlink script above
5. **Monitor CAN bus** - verify 0x1FF commands sent, 0x205 feedback received
6. **Check motor movement** - GM6020 should respond to commands

## Troubleshooting

### Motor still not responding

1. **Check CAN wiring**:
   - STM32 PA11 → CAN transceiver RX
   - STM32 PA12 → CAN transceiver TX
   - CAN_H and CAN_L to GM6020
   - 120Ω termination resistors on both ends

2. **Check CAN baud rate**:
   - FDCAN1 configured for 1 Mbps (standard DJI RoboMaster rate)
   - Check `Core/Src/fdcan.c` line 47-50

3. **Verify GM6020 power**:
   - GM6020 requires 24V power supply
   - Check motor LED status

4. **Check motor ID**:
   - GM6020 default ID is 1 (feedback 0x205)
   - If changed, update `App/config/motor_config.h` line 321

### CAN messages not received

1. **Enable CAN interrupt in STM32CubeMX**:
   - Open `H753UDP.ioc`
   - FDCAN1 → NVIC Settings
   - Enable "FDCAN1 interrupt 0" and "FDCAN1 interrupt 1"
   - Regenerate code

2. **Check interrupt priority**:
   - Should be lower than FreeRTOS max syscall priority
   - Recommended: Priority 5-7

## Next Steps

To get full motor status feedback via MAVLink, consider implementing:

1. **Custom status message** (ROBOMASTER_MOTOR_STATUS, ID 12001)
2. **Periodic status broadcast** in main loop
3. **MAVLink logging** for motor diagnostics

See `MAVLINK_STANDARDIZATION.md` for custom message definitions.

## References

- Motor configuration: `App/config/motor_config.h:317-331`
- RoboMaster controller: `App/motors/robomaster_controller.c`
- CAN hardware: `Core/Src/fdcan.c`
- FreeRTOS task: `Core/Src/freertos.c:349-560`

---

**Build Status**: ✅ Successful (Flash: 152KB / 2MB, RAM: 326KB / 992KB)

**Last Updated**: 2025-11-09
