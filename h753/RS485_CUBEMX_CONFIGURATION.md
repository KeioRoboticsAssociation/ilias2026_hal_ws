# RS485 Motor Control - STM32CubeMX Configuration Guide

## Overview

This guide provides step-by-step instructions for configuring the STM32H753 in STM32CubeMX to support RS485 motor control via USART1 and USART2.

## Prerequisites

- STM32CubeMX installed
- STM32H753 board/chip
- RS485 transceiver IC (MAX485, SN65HVD75, or similar)

## Hardware Setup

### RS485 Transceiver Connections

For each USART port used (USART1, USART2):

```
STM32H753          RS485 Transceiver (e.g., MAX485)
-----------        ---------------------------------
USART_TX    →      DI (Driver Input)
USART_RX    ←      RO (Receiver Output)
GPIO (DE)   →      DE (Driver Enable) - Optional for half-duplex
GPIO (RE)   →      RE (Receiver Enable) - Optional for half-duplex
GND         →      GND
VCC         →      VCC (3.3V or 5V depending on transceiver)

RS485 Transceiver → Ikeya MD Motor
-----------------   ----------------
A               →   A (RS485+)
B               →   B (RS485-)
```

**Note**: Current implementation uses full-duplex (4-wire) RS485. For half-duplex (2-wire), you'll need to add GPIO control for DE/RE pins in the rs485_controller.c.

## STM32CubeMX Configuration Steps

### 1. Open Project

1. Open `H753UDP.ioc` in STM32CubeMX
2. Wait for project to load

### 2. Configure USART1 (for RS485 Motor #1)

#### Pinout Configuration

1. Navigate to **Pinout & Configuration** tab
2. In the left panel, expand **Connectivity** → **USART1**
3. Set **Mode** to: `Asynchronous`

#### Parameter Settings

1. Click on **USART1** in the configuration view
2. Set the following parameters:

**Basic Parameters:**
- **Baud Rate**: `500000 Bits/s` (500 kbps) ⚠️ **Critical for Ikeya MD**
- **Word Length**: `8 Bits`
- **Parity**: `None`
- **Stop Bits**: `1`
- **Data Direction**: `Receive and Transmit`
- **Over Sampling**: `16 Samples` (default)

**Advanced Parameters (if visible):**
- **Prescaler**: `UART_PRESCALER_DIV1` (default)
- **FIFO Mode**: `Disable` (default)

#### Pin Assignment

Default USART1 pins (may vary based on your board):
- **TX**: PA9 or PB6 (check datasheet)
- **RX**: PA10 or PB7 (check datasheet)

Verify pins are correctly assigned in the Pinout view.

### 3. Configure USART2 (for RS485 Motor #2)

Repeat the same steps as USART1:

#### Pinout Configuration

1. In **Connectivity**, select **USART2**
2. Set **Mode** to: `Asynchronous`

#### Parameter Settings

Same as USART1:
- **Baud Rate**: `500000 Bits/s`
- **Word Length**: `8 Bits`
- **Parity**: `None`
- **Stop Bits**: `1`

#### Pin Assignment

Default USART2 pins:
- **TX**: PA2 or PD5 (check datasheet)
- **RX**: PA3 or PD6 (check datasheet)

### 4. Optional: Configure GPIO for DE/RE Control (Half-Duplex)

If using half-duplex (2-wire) RS485, configure GPIO pins for direction control:

1. Select two unused GPIO pins (e.g., PA4, PA5)
2. Set to **GPIO_Output**
3. Label them:
   - `USART1_DE` (Driver Enable)
   - `USART1_RE` (Receiver Enable, can be tied to DE)
4. In GPIO settings:
   - **GPIO output level**: `Low`
   - **GPIO mode**: `Output Push Pull`
   - **GPIO Pull-up/Pull-down**: `No pull-up and no pull-down`
   - **Maximum output speed**: `Low`

### 5. Clock Configuration

1. Navigate to **Clock Configuration** tab
2. Verify that the USART clock source provides adequate frequency for 500 kbps
3. For STM32H753:
   - USART1/2 typically run from PCLK1 or PCLK2
   - Ensure PCLK frequency is sufficient (typically 64 MHz or higher)

### 6. NVIC Settings (Optional)

For interrupt-based communication (if needed in future):

1. Go to **NVIC Settings** in USART configuration
2. Enable **USART1 global interrupt** (if using interrupts)
3. Set appropriate priority (default is usually fine)

**Note**: Current implementation uses blocking transmit/receive, so interrupts are not required.

### 7. DMA Settings (Optional, Not Currently Used)

Current implementation does not use DMA. If you want to add DMA support:

1. In USART settings, go to **DMA Settings**
2. Add DMA Request for:
   - `USART1_RX` (Stream selection varies)
   - `USART1_TX` (Stream selection varies)

### 8. Generate Code

1. **Project Manager** tab → **Project** section
2. Verify project name: `H753UDP`
3. **Toolchain / IDE**: `Makefile` or `CMake`
4. Click **GENERATE CODE** button
5. If prompted about overwriting files, select:
   - ✅ Keep user code in USER CODE sections
   - ✅ Backup project before overwrite (recommended)

### 9. Verify Generated Code

After generation, verify the following files contain the USART configuration:

**Core/Inc/usart.h:**
```c
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
```

**Core/Src/usart.c:**
```c
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;  // ⚠️ Verify this is 500000
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  // ...
}
```

## Motor Configuration in Code

After CubeMX configuration, the motors are already configured in `App/config/motor_config.h`:

```c
// RS485 Motor Configurations
static const rs485_config_t RS485_CONFIGS[MAX_RS485] = {
    {
        .id = 1,                          // Device ID on RS485 bus
        .control_mode = RS485_CONTROL_VELOCITY,  // Velocity control (RPS)
        .uart_id = 1,                     // USART1
        .watchdog_timeout_ms = 1000,
        .max_velocity_rps = 100.0f,
        .max_position_count = 32767,
        .encoder_resolution = 8192,       // 2048 PPR × 4
    },
    {
        .id = 2,                          // Device ID on RS485 bus
        .control_mode = RS485_CONTROL_POSITION,  // Position control
        .uart_id = 2,                     // USART2
        .watchdog_timeout_ms = 1000,
        .max_velocity_rps = 50.0f,
        .max_position_count = 32767,
        .encoder_resolution = 8192,
    },
};

// Motor Instance Mapping
static const motor_instance_t MOTOR_INSTANCES[MAX_MOTORS] = {
    // ... other motors ...

    // RS485 motors (use motor IDs 30-31 to avoid conflict with RC channels 1-8)
    {.id = 30, .type = MOTOR_TYPE_RS485, .uart_id = 1, .enabled = true},  // RS485 device 1
    {.id = 31, .type = MOTOR_TYPE_RS485, .uart_id = 2, .enabled = true},  // RS485 device 2
};
```

## Ikeya MD Motor DIP Switch Configuration

Each Ikeya MD motor has a 4-bit DIP switch for configuration:

```
DIP Switch Layout (motor side):
┌─────────────────┐
│ [4] [3] [2] [1] │  ← Switch numbers
└─────────────────┘

Bit 4-2: Motor ID (binary)
Bit 1:   Control Mode (0 = velocity, 1 = position)
```

### Examples:

**Motor ID 1, Velocity Mode:**
```
Switch: [OFF] [OFF] [OFF] [ON]
Binary:   0     0     1    0
Result: ID=1, Velocity Mode
```

**Motor ID 2, Position Mode:**
```
Switch: [OFF] [OFF] [ON] [ON]
Binary:   0     1     0    1
Result: ID=2, Position Mode
```

**Motor ID 8, Velocity Mode:**
```
Switch: [ON] [ON] [ON] [OFF]
Binary:   1    0    0    0
Result: ID=8, Velocity Mode
```

### DIP Switch Truth Table:

| Motor ID | Bit 4 | Bit 3 | Bit 2 | Velocity Mode (Bit 1) | Position Mode (Bit 1) |
|----------|-------|-------|-------|-----------------------|-----------------------|
| 1        | OFF   | OFF   | OFF   | ON                    | OFF                   |
| 2        | OFF   | OFF   | ON    | ON                    | OFF                   |
| 3        | OFF   | OFF   | ON    | ON                    | ON                    |
| 4        | OFF   | ON    | OFF   | ON                    | OFF                   |
| 5        | OFF   | ON    | OFF   | ON                    | ON                    |
| 6        | OFF   | ON    | ON    | ON                    | OFF                   |
| 7        | OFF   | ON    | ON    | ON                    | ON                    |
| 8        | ON    | OFF   | OFF   | ON                    | OFF                   |

**Note**: Motor ID in DIP switch should match the `id` field in `RS485_CONFIGS[]`.

## Verification Steps

### 1. Check UART Initialization

In `Core/Src/freertos.c`, verify UART handles are declared:

```c
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
```

### 2. Check UART Registration

Verify UART registration in `StartDefaultTask()`:

```c
// Register UART handles for RS485 motors
hw_err = hw_uart_register(1, &huart1);  // USART1
// ... error handling ...

hw_err = hw_uart_register(2, &huart2);  // USART2
// ... error handling ...
```

### 3. Build Project

```bash
cd build
cmake --build .
```

Check for compilation errors related to USART.

## Troubleshooting

### Issue: Build errors about undefined `huart1` or `huart2`

**Solution**:
- Ensure CubeMX generated code correctly
- Check that `usart.c` and `usart.h` contain UART handle definitions
- Verify `#include "usart.h"` is present in freertos.c

### Issue: Motor not responding

**Solution**:
1. Check physical RS485 connections (A to A, B to B)
2. Verify motor DIP switch ID matches `RS485_CONFIGS[].id`
3. Check UART baudrate is exactly 500000 bps
4. Use logic analyzer to verify UART TX/RX signals
5. Verify RS485 transceiver power supply (3.3V or 5V)

### Issue: CRC errors in communication

**Solution**:
- Check for noise on RS485 lines (use twisted pair cable)
- Verify baudrate accuracy (500000 bps exactly)
- Add termination resistors on RS485 bus (120Ω typical)
- Check ground connection between STM32 and RS485 transceiver

### Issue: UART initialization fails

**Solution**:
- Check clock configuration (PCLK must support 500 kbps)
- Verify GPIO pins are correctly assigned
- Check for pin conflicts with other peripherals

## Additional Notes

### RS485 Bus Topology

For best results:
- Use twisted pair cable for A/B lines
- Add 120Ω termination resistors at each end of the bus
- Keep cable length < 1200m for 500 kbps
- Maximum 32 devices on one RS485 bus (typical)

### Power Considerations

- RS485 transceivers may require 3.3V or 5V (check datasheet)
- Ensure adequate current supply for multiple motors
- Consider isolated DC-DC converter for motor power

### Future Enhancements

Possible improvements to implement:
1. **Half-Duplex Mode**: Add GPIO control for DE/RE pins
2. **DMA Support**: Use DMA for TX/RX to reduce CPU load
3. **Interrupt Mode**: Use UART interrupts instead of blocking calls
4. **Multi-drop Bus**: Support multiple motors on same UART (already possible with device IDs)

## Summary Checklist

Before testing:
- ✅ USART1/2 configured for 500000 bps, 8N1
- ✅ RS485 transceiver connected to USART TX/RX pins
- ✅ Motor DIP switch set to correct ID and mode
- ✅ RS485 A/B lines connected to motor
- ✅ Code compiled without errors
- ✅ Firmware flashed to STM32H753
- ✅ MAVLink connection established
- ✅ MOTOR_COMMAND messages sent via pymavlink

---

**Last Updated**: 2025-11-03
**Supported Motors**: Ikeya MD RS485 Motors
**Tested Baudrates**: 500000 bps
