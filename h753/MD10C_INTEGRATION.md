# MD10C Motor Driver Integration

## Summary

The H753 configuration has been updated to support **MD10C motor drivers** which use:
- **1 PWM signal** for speed control (0-100% duty cycle)
- **1 DIR GPIO** for direction control (HIGH=forward, LOW=reverse)

This replaces the previous 2-PWM configuration (forward + reverse channels).

## Changes Made

### 1. YAML Configuration Updated

**File:** `Lib/mavlink_hal/config/examples/h753_full.yaml`

```yaml
- id: 10
  type: dc_motor
  name: dc_motor_left
  hardware:
    timer: "3"
    channel: 1
    pins:
      pwm: PA6          # PWM signal (TIM3_CH1)
      dir: PF3          # Direction GPIO
      encoder_a: PE9    # Encoder channel A
      encoder_b: PE11   # Encoder channel B
  config:
    dc_motor:
      encoder:
        enabled: true
        ppr: 8192
      direction_inverted: false
```

### 2. Template Files Updated

**Modified Templates:**
- `config.h.j2` - Added `dir_pin` and `direction_inverted` fields to `mavlink_gen_dc_motor_config_t`
- `devices.c.j2` - Extracts DIR pin from YAML and passes to controller

**Generated Config Structure:**
```c
typedef struct {
    uint8_t id;
    const char* name;
    uint8_t timer_id;
    uint32_t channel;
    const char* dir_pin;        /* Direction GPIO pin (e.g., "PF3") for MD10C */
    float max_rpm;
    bool has_encoder;
    uint16_t encoder_ppr;
    bool direction_inverted;
} mavlink_gen_dc_motor_config_t;
```

### 3. Generated Files

✅ **Generated Configuration (h753_full):**
```c
const mavlink_gen_dc_motor_config_t mavlink_gen_dc_motor_configs[2] = {
    {
        .id = 10,
        .name = "dc_motor_left",
        .timer_id = 3,
        .channel = TIM_CHANNEL_1,
        .dir_pin = "PF3",           // Direction GPIO
        .max_rpm = 15.0f,
        .has_encoder = true,
        .encoder_ppr = 8192,
        .direction_inverted = false,
    },
    {
        .id = 11,
        .name = "dc_motor_right",
        .timer_id = 3,
        .channel = TIM_CHANNEL_2,
        .dir_pin = "PF4",           // Direction GPIO
        .max_rpm = 15.0f,
        .has_encoder = true,
        .encoder_ppr = 8192,
        .direction_inverted = false,
    }
};
```

## Required Implementation Changes

### Step 1: Update `dc_motor_config_t` Structure

**File:** `App/config/motor_config.h`

Add the `dir_pin` field to the DC motor configuration:

```c
typedef struct {
    uint8_t id;

    // Hardware
    const char* dir_pin;            // Direction GPIO pin (e.g., "PF3") - NEW!

    // Speed PID
    float speed_kp;
    float speed_ki;
    // ... rest of fields ...

    // Control
    uint32_t watchdog_timeout_ms;
    uint32_t control_period_ms;
    bool direction_inverted;
} dc_motor_config_t;
```

### Step 2: Update DC Motor Controller

**File:** `App/motors/dc_controller.h` and `dc_controller.c`

#### A. Add DIR pin to private data:

```c
typedef struct {
    dc_motor_config_t config;
    uint8_t timer_id;
    uint32_t channel;
    const char* dir_pin;             // NEW: Direction GPIO pin name
    GPIO_TypeDef* dir_port;          // NEW: Direction GPIO port
    uint16_t dir_pin_number;         // NEW: Direction GPIO pin number
    uint32_t last_watchdog_reset;
    bool watchdog_expired;
    // ... rest of private data ...
} dc_motor_private_t;
```

#### B. Initialize DIR GPIO in `dc_motor_initialize()`:

```c
static error_code_t dc_motor_initialize(motor_controller_t* controller) {
    // ... existing code ...

    // Parse and initialize DIR GPIO pin
    if (priv->config.dir_pin != NULL) {
        // Parse pin name (e.g., "PF3" -> GPIOF, GPIO_PIN_3)
        priv->dir_port = parse_gpio_port(priv->config.dir_pin);
        priv->dir_pin_number = parse_gpio_pin(priv->config.dir_pin);

        // Set to forward direction initially
        HAL_GPIO_WritePin(priv->dir_port, priv->dir_pin_number, GPIO_PIN_SET);
    }

    // ... rest of initialization ...
}
```

#### C. Control DIR pin in `dc_motor_set_command()`:

```c
static error_code_t dc_motor_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    // ... existing command processing ...

    // Apply direction inversion
    if (config->direction_inverted) {
        duty_cycle = -duty_cycle;
    }

    priv->current_duty_cycle = duty_cycle;

    // Set direction GPIO for MD10C
    if (priv->dir_port != NULL) {
        GPIO_PinState dir_state = (duty_cycle >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(priv->dir_port, priv->dir_pin_number, dir_state);
    }

    // Convert duty cycle to PWM value (use absolute value)
    uint32_t period = hw_timer_get_period(priv->timer_id);
    uint32_t pulse = (uint32_t)((fabsf(duty_cycle) * (float)period));

    // Set PWM
    error_code_t err = hw_timer_set_pwm(priv->timer_id, priv->channel, pulse);
    // ... error handling ...
}
```

### Step 3: Add GPIO Parsing Helpers

**File:** `App/hal/hardware_manager.h` and `hardware_manager.c`

Add helper functions to parse pin names:

```c
/**
 * @brief Parse GPIO port from pin name (e.g., "PF3" -> GPIOF)
 */
GPIO_TypeDef* hw_parse_gpio_port(const char* pin_name);

/**
 * @brief Parse GPIO pin number from pin name (e.g., "PF3" -> GPIO_PIN_3)
 */
uint16_t hw_parse_gpio_pin(const char* pin_name);
```

Implementation example:
```c
GPIO_TypeDef* hw_parse_gpio_port(const char* pin_name) {
    if (!pin_name || strlen(pin_name) < 2) return NULL;

    switch (pin_name[1]) {  // Second character is port letter
        case 'A': return GPIOA;
        case 'B': return GPIOB;
        case 'C': return GPIOC;
        case 'D': return GPIOD;
        case 'E': return GPIOE;
        case 'F': return GPIOF;
        case 'G': return GPIOG;
        case 'H': return GPIOH;
        default: return NULL;
    }
}

uint16_t hw_parse_gpio_pin(const char* pin_name) {
    if (!pin_name || strlen(pin_name) < 2) return 0;

    int pin_num = atoi(&pin_name[2]);  // Parse number after 'P' and port letter
    return (1 << pin_num);  // Convert to GPIO_PIN_x format
}
```

### Step 4: STM32CubeMX Configuration

**IMPORTANT:** Configure DIR pins as GPIO_Output in STM32CubeMX:

1. Open `H753UDP.ioc` in STM32CubeMX
2. Configure pins:
   - **PF3** → GPIO_Output (for DC Motor 10 DIR)
   - **PF4** → GPIO_Output (for DC Motor 11 DIR)
3. Set GPIO parameters:
   - GPIO output level: Low
   - GPIO mode: Output Push Pull
   - GPIO Pull-up/Pull-down: No pull-up and no pull-down
   - Maximum output speed: Low
4. Generate code

## Motor Hardware Connections

### DC Motor 10 (Left)
```
MD10C Driver          STM32H753
-------------         ---------
PWM    ←──────────    PA6  (TIM3_CH1)
DIR    ←──────────    PF3  (GPIO_Output)
ENCA   ─────────→     PE9  (TIM1_CH1 or GPIO)
ENCB   ─────────→     PE11 (TIM1_CH2 or GPIO)
GND    ────────────   GND
VCC    ────────────   5V
```

### DC Motor 11 (Right)
```
MD10C Driver          STM32H753
-------------         ---------
PWM    ←──────────    PB5  (TIM3_CH2)
DIR    ←──────────    PF4  (GPIO_Output)
ENCA   ─────────→     PA0  (TIM2/5_CH1 or GPIO)
ENCB   ─────────→     PB3  (TIM2_CH2 or GPIO)
GND    ────────────   GND
VCC    ────────────   5V
```

## Testing

### MAVLink Test Commands

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# Forward at 50% speed
master.mav.motor_command_send(
    motor_id=10,
    control_mode=3,        # 3 = duty_cycle
    target_value=0.5,      # 50% forward
    enable=1
)

# Reverse at 30% speed
master.mav.motor_command_send(
    motor_id=10,
    control_mode=3,
    target_value=-0.3,     # 30% reverse (negative)
    enable=1
)

# Stop
master.mav.motor_command_send(
    motor_id=10,
    control_mode=3,
    target_value=0.0,
    enable=1
)
```

## Summary Checklist

- [x] Update YAML configuration with MD10C hardware (1 PWM + 1 DIR)
- [x] Update code generator templates (config.h.j2, devices.c.j2)
- [x] Regenerate code from h753_full.yaml
- [ ] Update `dc_motor_config_t` struct to include `dir_pin`
- [ ] Update `dc_motor_private_t` to store DIR GPIO info
- [ ] Implement GPIO parsing helpers in hardware_manager
- [ ] Update `dc_motor_initialize()` to configure DIR GPIO
- [ ] Update `dc_motor_set_command()` to control DIR pin
- [ ] Configure DIR pins in STM32CubeMX (PF3, PF4 as GPIO_Output)
- [ ] Build and test with MAVLink commands

## Date

2025-11-14