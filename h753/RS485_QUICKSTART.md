# RS485 Motor Control Quick Start Guide

## Overview

This guide provides a complete walkthrough for setting up and testing RS485 motor control (Ikeya MD motors) via MAVLink on the STM32H753 platform.

## 🎯 What's Been Implemented

✅ **Complete RS485 motor controller** supporting Ikeya MD protocol
✅ **Custom MAVLink MOTOR_COMMAND message** (ID 12004)
✅ **Unified motor control interface** via vtables
✅ **UART hardware abstraction** for RS485 communication
✅ **Velocity and position control modes**
✅ **CRC-16 Modbus** packet validation
✅ **MAVLink UDP communication** at 500 kbps
✅ **Python test program** for easy motor control

## 📋 Prerequisites

### Hardware
- STM32H753 development board
- RS485 transceiver IC (MAX485, SN65HVD75, or similar) × 2
- Ikeya MD RS485 motors × 2
- Ethernet cable
- Power supply for motors

### Software
- STM32CubeMX
- ARM GCC toolchain
- CMake 3.22+
- Python 3 with pymavlink (`pip install pymavlink`)

## 🚀 Quick Setup (5 Steps)

### Step 1: Configure STM32CubeMX

1. Open `H753UDP.ioc` in STM32CubeMX
2. Configure **USART1**:
   - Mode: `Asynchronous`
   - Baud Rate: `500000 Bits/s` ⚠️ **Critical**
   - Word Length: `8 Bits`
   - Parity: `None`
   - Stop Bits: `1`
3. Configure **USART2** (same settings as USART1)
4. **Generate Code** → Select "Keep user sections"

**Detailed instructions**: See `RS485_CUBEMX_CONFIGURATION.md`

### Step 2: Build Firmware

```bash
cd /path/to/h753
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```

Expected output: `build/H753UDP.elf`

### Step 3: Flash Firmware

```bash
STM32_Programmer_CLI --connect port=swd \
    --download H753UDP.elf \
    --hardRst --rst --start
```

Or use STM32CubeProgrammer GUI.

### Step 4: Connect Hardware

**RS485 Transceiver Connections (×2)**

```
STM32H753          MAX485 #1        Ikeya MD Motor #1
-----------        ----------       ------------------
PA9 (USART1_TX) →  DI
PA10(USART1_RX) ← RO
GND             →  GND
3.3V            →  VCC
                   A/B           →  RS485 A/B

STM32H753          MAX485 #2        Ikeya MD Motor #2
-----------        ----------       ------------------
PA2 (USART2_TX) →  DI
PA3 (USART2_RX) ← RO
GND             →  GND
3.3V            →  VCC
                   A/B           →  RS485 A/B
```

**Motor DIP Switch Configuration**

Motor #1 (Device ID 1, Velocity mode):
```
DIP: [OFF] [OFF] [OFF] [ON]  (Binary: 0010)
```

Motor #2 (Device ID 2, Position mode):
```
DIP: [OFF] [OFF] [ON] [ON]  (Binary: 0101)
```

**Network Connection**
- Connect STM32H753 Ethernet to PC
- Configure PC Ethernet:
  - IP: `192.168.11.2`
  - Subnet: `255.255.255.0`

### Step 5: Test with Python Script

```bash
# Test connection
ping 192.168.11.4

# Run automated tests
python3 test_rs485_mavlink.py

# Or interactive mode
python3 test_rs485_mavlink.py --mode interactive
```

## 🎮 Using the Test Program

### Automated Test Mode (Default)

```bash
python3 test_rs485_mavlink.py
```

Runs three automated tests:
1. **Velocity Control Test** (Motor 30)
   - 20 RPS forward → 50 RPS → -30 RPS reverse → Stop
2. **Position Control Test** (Motor 31)
   - 0° → 90° → 180° → -90° → 0°
3. **Simultaneous Control** (Both motors)
   - Motor 30 at 25 RPS + Motor 31 at 45°

### Interactive Mode

```bash
python3 test_rs485_mavlink.py --mode interactive
```

**Commands:**
```
v <motor_id> <rps>      # Set velocity (e.g., 'v 30 50.0')
p <motor_id> <rad>      # Set position (e.g., 'p 31 1.57')
s <motor_id>            # Stop motor (e.g., 's 30')
e                       # Emergency stop all
q                       # Quit
```

**Examples:**
```bash
>>> v 30 25.0         # Motor 30 at 25 RPS
>>> p 31 3.14159      # Motor 31 to 180 degrees
>>> v 30 -10.0        # Motor 30 reverse at 10 RPS
>>> s 30              # Stop motor 30
>>> e                 # Emergency stop all motors
```

## 🔧 Manual Control via pymavlink

### Basic Example

```python
from pymavlink import mavutil
import time

# Connect to STM32H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()
print("Connected!")

# Control RS485 motor #1 (velocity mode, 50 RPS)
master.mav.motor_command_send(
    motor_id=30,           # RS485 motor #1
    control_mode=1,        # 1 = velocity
    target_value=50.0,     # 50 RPS
    enable=1               # Enable motor
)
time.sleep(3)

# Stop motor
master.mav.motor_command_send(
    motor_id=30,
    control_mode=1,
    target_value=0.0,
    enable=1
)

master.close()
```

### Control Modes Reference

| Mode Value | Mode Name | target_value Units | Example Value |
|------------|-----------|-------------------|---------------|
| 0 | Position | Radians | 3.14159 (180°) |
| 1 | Velocity | RPS (Revolutions/sec) | 50.0 RPS |
| 2 | Current | Amps | 2.5 A |
| 3 | Duty Cycle | -1.0 to +1.0 | 0.75 (75%) |

## 📊 Motor Configuration Reference

Current configuration in `App/config/motor_config.h`:

### RS485 Motors

| Motor ID | RS485 Device ID | UART | Control Mode | Max Velocity | Encoder Resolution |
|----------|-----------------|------|--------------|--------------|-------------------|
| 30 | 1 | USART1 | Velocity | 100.0 RPS | 8192 counts/rev |
| 31 | 2 | USART2 | Position | 50.0 RPS | 8192 counts/rev |

### Modifying Motor Configuration

Edit `App/config/motor_config.h`:

```c
// Change max velocity for motor 30
static const rs485_config_t RS485_CONFIGS[MAX_RS485] = {
    {
        .id = 1,
        .control_mode = RS485_CONTROL_VELOCITY,
        .uart_id = 1,
        .watchdog_timeout_ms = 1000,
        .max_velocity_rps = 150.0f,  // ← Change this
        // ...
    },
    // ...
};
```

After changes, rebuild:
```bash
cd build
cmake --build .
```

## 🔍 Troubleshooting

### Problem: Motor doesn't respond

**Check:**
1. ✓ Motor power supply connected
2. ✓ RS485 A/B wires connected correctly
3. ✓ Motor DIP switch ID matches config (Device ID 1 or 2)
4. ✓ UART baudrate is 500000 bps in CubeMX
5. ✓ MAVLink heartbeat received (`ping 192.168.11.4`)

**Debug:**
```bash
# Check network connection
ping 192.168.11.4

# Monitor MAVLink traffic
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:192.168.11.4:14550')
while True:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg)
"
```

### Problem: Build errors

**Common fixes:**
```bash
# Clean rebuild
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```

### Problem: CRC errors in RS485 communication

**Solutions:**
- Use shielded twisted pair cable for RS485
- Add 120Ω termination resistors at both ends
- Check motor DIP switch configuration
- Verify UART baudrate is exactly 500000 bps

### Problem: STM32 not responding after flash

**Solutions:**
- Check LED on GPIO PB0:
  - **3 blinks at startup**: Normal initialization
  - **Rapid blinking**: Initialization failure
  - **1Hz toggle**: Normal operation
- Press reset button
- Re-flash firmware
- Check UART/FDCAN configuration in CubeMX

## 📚 Architecture Overview

```
User (pymavlink)
    ↓ UDP Port 14550
MAVLink MOTOR_COMMAND (ID 12004)
    ↓
mavlink_udp.c → mavlink_message_handler()
    ↓
motor_registry_send_command()
    ↓
motor_set_command() [vtable dispatch]
    ↓
rs485_controller.c
    ↓
hw_uart_transmit() [hardware_manager.c]
    ↓
USART1/2 → RS485 Transceiver → Ikeya MD Motor
```

## 🎯 Next Steps

### Extend Motor Control

1. **Add more motors**: Configure additional RS485 devices (up to 8 per UART)
2. **Implement feedback**: Add motor status reporting via MAVLINK_MSG_ID_MOTOR_STATUS
3. **PID tuning**: Adjust velocity/position control gains in motor configs
4. **Trajectory planning**: Implement smooth motion profiles

### Advanced Features

1. **Half-duplex RS485**: Add GPIO control for DE/RE pins
2. **DMA mode**: Use DMA for UART to reduce CPU load
3. **Error recovery**: Implement automatic retry on CRC errors
4. **Multi-drop bus**: Control multiple motors on single UART

## 📖 Additional Documentation

- **`RS485_IMPLEMENTATION_SUMMARY.md`**: Implementation checklist and details
- **`RS485_CUBEMX_CONFIGURATION.md`**: Complete CubeMX configuration guide
- **`MAVLINK_STANDARDIZATION.md`**: MAVLink dialect documentation
- **`rs485.md`**: Complete Ikeya MD protocol specification
- **`CLAUDE.md`**: Full project documentation

## ⚠️ Safety Notes

- Always test with motor disconnected from load first
- Use emergency stop before approaching moving motors
- Verify motor direction before connecting to mechanical systems
- Monitor motor temperature during extended operation
- Use appropriate power supply ratings for your motors

## 📝 Summary

You now have a complete RS485 motor control system with:

✅ Two-way MAVLink communication over Ethernet
✅ Custom MOTOR_COMMAND message for unified control
✅ Support for velocity and position control modes
✅ CRC-validated RS485 protocol (Ikeya MD)
✅ Python test scripts for easy control
✅ Full hardware abstraction layer

**Ready to control your motors!** 🎉

For questions or issues, check:
1. LED blink patterns on STM32
2. MAVLink traffic with pymavlink
3. UART signals with logic analyzer
4. RS485 bus with oscilloscope

---

**Last Updated**: 2025-11-03
**Firmware**: H753UDP.elf
**MAVLink**: v2.0, robomaster_motor dialect
**RS485 Protocol**: Ikeya MD, 500 kbps, 8N1
