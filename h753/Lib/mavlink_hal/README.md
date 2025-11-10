# MAVLink Hardware Abstraction Layer (HAL)

A portable, header-only Hardware Abstraction Layer for MAVLink communication across multiple microcontroller platforms.

## Overview

This portable HAL provides a unified interface for hardware operations required by MAVLink communication systems. It supports STM32, Arduino, ESP32, and other platforms through a function pointer-based abstraction.

**Epic1 Task1 Implementation** - Part of the MAVLink device abstraction framework.

## Features

- **Pure C99 implementation** - No C++ dependencies
- **Header-only design** - Easy integration with inline functions
- **Zero dynamic allocation** - Suitable for embedded systems
- **Platform-agnostic** - Works across STM32, Arduino, ESP32, Linux
- **Comprehensive coverage** - UART, GPIO, Timer, PWM, ADC, CAN (optional)
- **Thread-safe** - Optional mutex support for RTOS environments
- **Feature flags** - Compile only what you need

## Directory Structure

```
Lib/mavlink_hal/
├── include/
│   ├── mavlink_hal_types.h           # Common types and error codes
│   ├── mavlink_hal_interface.h       # Main HAL interface
│   └── mavlink_hal_config.h.template # Configuration template
├── platforms/
│   └── stm32/
│       └── mavlink_hal_stm32.c       # STM32 implementation
├── examples/
│   └── stm32_example.c               # Usage example
└── README.md                          # This file
```

## Quick Start

### 1. Copy Configuration Template

```bash
cp include/mavlink_hal_config.h.template include/mavlink_hal_config.h
```

### 2. Edit Configuration

Uncomment and configure options in `mavlink_hal_config.h`:

```c
/* Select your platform */
#define MAVLINK_HAL_PLATFORM_STM32

/* Enable features you need */
#define MAVLINK_HAL_ENABLE_CAN
#define MAVLINK_HAL_ENABLE_MUTEX

/* Adjust buffer sizes */
#define MAVLINK_HAL_UART_RX_BUFFER_SIZE 512
```

### 3. Include in Your Project

```c
#include "mavlink_hal_interface.h"

/* For STM32, include platform implementation */
extern UART_HandleTypeDef huart2;

int main(void) {
    /* Initialize HAL */
    HAL_Init();
    SystemClock_Config();

    /* Configure MAVLink UART */
    mavlink_hal_uart_config_t uart_config = {
        .baudrate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .flow_control = false,
        .rx_buffer_size = 512,
        .tx_buffer_size = 512
    };

    /* Initialize UART through HAL */
    g_mavlink_hal.uart_init(&huart2, &uart_config);

    /* Send data */
    const uint8_t data[] = "Hello MAVLink!";
    size_t bytes_sent;
    g_mavlink_hal.uart_send(&huart2, data, sizeof(data)-1, 1000, &bytes_sent);

    while (1) {
        /* Your application code */
    }
}
```

## Platform Support

### STM32

**Status:** Partial implementation (UART and Timer functions)

**Supported series:** F4, F7, H7, L4

**Integration:** Uses STM32 HAL driver and CubeMX generated code

**Features:**
- ✅ UART (TX/RX with timeout)
- ✅ Timer (milliseconds/microseconds)
- ⚠️  GPIO (stub - needs implementation)
- ⚠️  PWM (stub - needs implementation)
- ⚠️  ADC (stub - needs implementation)

**Files:**
- `platforms/stm32/mavlink_hal_stm32.c`

### Arduino

**Status:** Not implemented (planned for Epic1 Task2)

**Target boards:** Mega2560, Due, Nano Every

### ESP32

**Status:** Not implemented (planned for Epic1 Task2)

**Features:** FreeRTOS integration, WiFi coexistence

### Linux (Mock/Test)

**Status:** Not implemented (planned for Epic1 Task2)

**Purpose:** Unit testing and simulation

## API Reference

### Error Codes

```c
typedef enum {
    MAVLINK_HAL_OK                  = 0,
    MAVLINK_HAL_ERR_INVALID_PARAM   = -2,
    MAVLINK_HAL_ERR_NOT_INITIALIZED = -3,
    MAVLINK_HAL_ERR_TIMEOUT         = -5,
    MAVLINK_HAL_ERR_HW_FAULT        = -9,
    /* ... see mavlink_hal_types.h for complete list */
} mavlink_hal_error_t;
```

### UART Operations

```c
/* Initialize UART */
mavlink_hal_error_t uart_init(
    mavlink_hal_uart_handle_t handle,
    const mavlink_hal_uart_config_t* config
);

/* Send data (blocking) */
mavlink_hal_error_t uart_send(
    mavlink_hal_uart_handle_t handle,
    const uint8_t* data,
    size_t length,
    uint32_t timeout_ms,
    size_t* bytes_sent
);

/* Receive data (blocking) */
mavlink_hal_error_t uart_receive(
    mavlink_hal_uart_handle_t handle,
    uint8_t* buffer,
    size_t length,
    uint32_t timeout_ms,
    size_t* bytes_received
);
```

### Timer Operations

```c
/* Get milliseconds since boot */
uint32_t time_millis(void);

/* Get microseconds since boot */
uint64_t time_micros(void);

/* Delay (blocking) */
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
```

### GPIO Operations

```c
/* Initialize GPIO pin */
mavlink_hal_error_t gpio_init(
    mavlink_hal_gpio_handle_t handle,
    const mavlink_hal_gpio_config_t* config
);

/* Write pin state */
mavlink_hal_error_t gpio_write(
    mavlink_hal_gpio_handle_t handle,
    bool state
);

/* Read pin state */
mavlink_hal_error_t gpio_read(
    mavlink_hal_gpio_handle_t handle,
    bool* state
);
```

See `include/mavlink_hal_interface.h` for complete API documentation.

## Integration with Existing Code

This HAL is designed to work alongside the existing h753 hardware_manager:

```c
/* Existing h753 hardware_manager */
#include "App/hal/hardware_manager.h"

/* New portable HAL */
#include "Lib/mavlink_hal/include/mavlink_hal_interface.h"

/* Use portable HAL for new cross-platform code */
g_mavlink_hal.uart_send(...);

/* Use existing hardware_manager for platform-specific code */
hw_uart_transmit(...);
```

## Configuration Options

See `include/mavlink_hal_config.h.template` for all options:

- Buffer sizes (UART, CAN)
- Feature enables (CAN, SPI, I2C, Mutex)
- Platform-specific settings
- Debug logging configuration
- Performance tuning

## Development

### Adding a New Platform

1. Create directory: `platforms/<platform_name>/`
2. Implement file: `mavlink_hal_<platform>.c`
3. Implement all required function pointers
4. Use `MAVLINK_HAL_REGISTER()` macro
5. Test with validation: `mavlink_hal_validate()`

### Required Functions

Minimum implementation must provide:
- `uart_init`, `uart_send`, `uart_receive`
- `time_millis`, `delay_ms`

All other functions can return `MAVLINK_HAL_ERR_NOT_SUPPORTED`.

### Testing

```c
/* Validate HAL implementation */
mavlink_hal_error_t err = mavlink_hal_validate();
if (err != MAVLINK_HAL_OK) {
    /* Handle error - missing required functions */
}

/* Check platform */
if (mavlink_hal_has_feature(MAVLINK_HAL_FEATURE_CAN)) {
    /* CAN operations available */
}
```

## Roadmap

- [x] Core types and interface definition
- [x] STM32 UART and Timer implementation
- [ ] STM32 GPIO, PWM, ADC implementation
- [ ] Arduino platform implementation (Epic1 Task2)
- [ ] ESP32 platform implementation (Epic1 Task2)
- [ ] Mock platform for unit testing (Epic1 Task2)
- [ ] DMA support for STM32 UART
- [ ] CAN bus support (FDCAN for H7)
- [ ] SPI/I2C support

## Contributing

This HAL is part of the Epic1 task series. Future implementations will add:
- Epic1 Task2: Complete platform implementations
- Epic2: Configuration schema and code generation
- Epic3: Device interface abstraction
- Epic4: ROS2 integration

## License

Part of the ilias2026_hal_ws project.

## See Also

- `include/mavlink_hal_interface.h` - Complete API documentation
- `include/mavlink_hal_types.h` - Type definitions
- `platforms/stm32/mavlink_hal_stm32.c` - STM32 implementation example
- `../../App/hal/hardware_manager.h` - Existing h753 HAL
