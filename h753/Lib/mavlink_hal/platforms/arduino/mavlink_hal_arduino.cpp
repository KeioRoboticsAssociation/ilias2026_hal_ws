/**
 * @file mavlink_hal_arduino.cpp
 * @brief Arduino platform implementation for MAVLink HAL
 *
 * Supports AVR (Mega2560) and ARM (Due, Zero) Arduino boards.
 * Uses Arduino core functions for hardware abstraction.
 *
 * @author Auto-generated for Epic1 Task2
 * @date 2025-11-10
 */

#include "../../include/mavlink_hal_interface.h"

#ifdef MAVLINK_HAL_ON_ARDUINO

/* Arduino includes */
#include <Arduino.h>

/* Software Serial support (optional) */
#ifdef MAVLINK_HAL_ARDUINO_SW_SERIAL
#include <SoftwareSerial.h>
#endif

/* ============================================================================
 * Platform-Specific Includes and Definitions
 * ============================================================================ */

/* Maximum number of instances */
#ifndef MAVLINK_HAL_MAX_UART_INSTANCES
#define MAVLINK_HAL_MAX_UART_INSTANCES 4
#endif

#ifndef MAVLINK_HAL_MAX_GPIO_IRQ_PINS
#define MAVLINK_HAL_MAX_GPIO_IRQ_PINS 16
#endif

/* ============================================================================
 * Private Types and Structures
 * ============================================================================ */

/**
 * @brief UART instance data
 */
typedef struct {
    HardwareSerial*                 serial;
    mavlink_hal_uart_rx_callback_t  rx_callback;
    void*                           rx_user_data;
    bool                            active;
} arduino_uart_instance_t;

/**
 * @brief GPIO IRQ instance data
 */
typedef struct {
    uint8_t                         pin;
    mavlink_hal_gpio_irq_callback_t callback;
    void*                           user_data;
    bool                            active;
} arduino_gpio_irq_instance_t;

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static arduino_uart_instance_t uart_instances[MAVLINK_HAL_MAX_UART_INSTANCES] = {0};
static arduino_gpio_irq_instance_t gpio_irq_instances[MAVLINK_HAL_MAX_GPIO_IRQ_PINS] = {0};

/* PWM resolution for different boards */
#if defined(__AVR__)
#define ARDUINO_PWM_RESOLUTION 8   /* 8-bit PWM on AVR */
#else
#define ARDUINO_PWM_RESOLUTION 12  /* 12-bit PWM on ARM (Due/Zero) */
#endif

/* ============================================================================
 * UART Operations - Arduino Implementation
 * ============================================================================ */

static mavlink_hal_error_t arduino_uart_init(
    mavlink_hal_uart_handle_t handle,
    const mavlink_hal_uart_config_t* config)
{
    if (handle == NULL || config == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;

    /* Configure serial port */
    uint32_t serial_config = SERIAL_8N1;  /* Default: 8 data bits, no parity, 1 stop bit */

    /* Adjust for parity */
    if (config->parity == 1) {
        serial_config = SERIAL_8O1;  /* Odd parity */
    } else if (config->parity == 2) {
        serial_config = SERIAL_8E1;  /* Even parity */
    }

    /* Adjust for stop bits (if supported) */
    if (config->stop_bits == 2) {
        serial_config = SERIAL_8N2;
    }

    serial->begin(config->baudrate, serial_config);

    /* Wait for serial port to be ready */
    while (!(*serial)) {
        delay(10);
    }

    /* Register instance */
    for (int i = 0; i < MAVLINK_HAL_MAX_UART_INSTANCES; i++) {
        if (!uart_instances[i].active) {
            uart_instances[i].serial = serial;
            uart_instances[i].active = true;
            return MAVLINK_HAL_OK;
        }
    }

    return MAVLINK_HAL_ERR_RESOURCE_EXHAUSTED;
}

static mavlink_hal_error_t arduino_uart_deinit(mavlink_hal_uart_handle_t handle)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;
    serial->end();

    /* Clear instance */
    for (int i = 0; i < MAVLINK_HAL_MAX_UART_INSTANCES; i++) {
        if (uart_instances[i].serial == serial) {
            uart_instances[i].active = false;
            uart_instances[i].serial = NULL;
            return MAVLINK_HAL_OK;
        }
    }

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_uart_send(
    mavlink_hal_uart_handle_t handle,
    const uint8_t* data,
    size_t length,
    mavlink_hal_timeout_ms_t timeout_ms,
    size_t* bytes_sent)
{
    if (handle == NULL || data == NULL || bytes_sent == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;

    unsigned long start_time = millis();
    size_t written = 0;

    while (written < length && (millis() - start_time) < timeout_ms) {
        size_t space = serial->availableForWrite();
        if (space > 0) {
            size_t to_write = min(space, length - written);
            written += serial->write(data + written, to_write);
        }
    }

    *bytes_sent = written;

    return (written == length) ? MAVLINK_HAL_OK : MAVLINK_HAL_ERR_TIMEOUT;
}

static mavlink_hal_error_t arduino_uart_receive(
    mavlink_hal_uart_handle_t handle,
    uint8_t* buffer,
    size_t length,
    mavlink_hal_timeout_ms_t timeout_ms,
    size_t* bytes_received)
{
    if (handle == NULL || buffer == NULL || bytes_received == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;

    unsigned long start_time = millis();
    size_t received = 0;

    while (received < length && (millis() - start_time) < timeout_ms) {
        if (serial->available() > 0) {
            buffer[received++] = serial->read();
        }
    }

    *bytes_received = received;

    return (received > 0) ? MAVLINK_HAL_OK : MAVLINK_HAL_ERR_TIMEOUT;
}

static mavlink_hal_error_t arduino_uart_register_rx_callback(
    mavlink_hal_uart_handle_t handle,
    mavlink_hal_uart_rx_callback_t callback,
    void* user_data)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;

    /* Find instance and register callback */
    for (int i = 0; i < MAVLINK_HAL_MAX_UART_INSTANCES; i++) {
        if (uart_instances[i].serial == serial) {
            uart_instances[i].rx_callback = callback;
            uart_instances[i].rx_user_data = user_data;
            return MAVLINK_HAL_OK;
        }
    }

    return MAVLINK_HAL_ERR_NOT_INITIALIZED;
}

static mavlink_hal_error_t arduino_uart_available(
    mavlink_hal_uart_handle_t handle,
    size_t* available)
{
    if (handle == NULL || available == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    HardwareSerial* serial = (HardwareSerial*)handle;
    *available = serial->available();

    return MAVLINK_HAL_OK;
}

/* ============================================================================
 * GPIO Operations - Arduino Implementation
 * ============================================================================ */

/**
 * @brief Arduino GPIO handle - just the pin number
 */
typedef struct {
    uint8_t pin;
} arduino_gpio_t;

static mavlink_hal_error_t arduino_gpio_init(
    mavlink_hal_gpio_handle_t handle,
    const mavlink_hal_gpio_config_t* config)
{
    if (handle == NULL || config == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_gpio_t* gpio = (arduino_gpio_t*)handle;

    switch (config->mode) {
        case MAVLINK_HAL_GPIO_MODE_INPUT:
            pinMode(gpio->pin, INPUT);
            break;

        case MAVLINK_HAL_GPIO_MODE_OUTPUT:
        case MAVLINK_HAL_GPIO_MODE_OUTPUT_OD:
            pinMode(gpio->pin, OUTPUT);
            digitalWrite(gpio->pin, config->initial_state ? HIGH : LOW);
            break;

        case MAVLINK_HAL_GPIO_MODE_INPUT_PULLUP:
            pinMode(gpio->pin, INPUT_PULLUP);
            break;

        case MAVLINK_HAL_GPIO_MODE_INPUT_PULLDOWN:
            /* Not all Arduino boards support INPUT_PULLDOWN */
#ifdef INPUT_PULLDOWN
            pinMode(gpio->pin, INPUT_PULLDOWN);
#else
            pinMode(gpio->pin, INPUT);
#endif
            break;

        default:
            return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_gpio_write(
    mavlink_hal_gpio_handle_t handle,
    bool state)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_gpio_t* gpio = (arduino_gpio_t*)handle;
    digitalWrite(gpio->pin, state ? HIGH : LOW);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_gpio_read(
    mavlink_hal_gpio_handle_t handle,
    bool* state)
{
    if (handle == NULL || state == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_gpio_t* gpio = (arduino_gpio_t*)handle;
    *state = (digitalRead(gpio->pin) == HIGH);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_gpio_toggle(mavlink_hal_gpio_handle_t handle)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_gpio_t* gpio = (arduino_gpio_t*)handle;
    digitalWrite(gpio->pin, !digitalRead(gpio->pin));

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_gpio_register_irq(
    mavlink_hal_gpio_handle_t handle,
    mavlink_hal_gpio_irq_callback_t callback,
    void* user_data)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_gpio_t* gpio = (arduino_gpio_t*)handle;

    /* Find available slot */
    for (int i = 0; i < MAVLINK_HAL_MAX_GPIO_IRQ_PINS; i++) {
        if (!gpio_irq_instances[i].active) {
            gpio_irq_instances[i].pin = gpio->pin;
            gpio_irq_instances[i].callback = callback;
            gpio_irq_instances[i].user_data = user_data;
            gpio_irq_instances[i].active = true;

            /* Note: Actual attachInterrupt() should be called by user */
            return MAVLINK_HAL_OK;
        }
    }

    return MAVLINK_HAL_ERR_RESOURCE_EXHAUSTED;
}

/* ============================================================================
 * Timer Operations - Arduino Implementation
 * ============================================================================ */

static uint32_t arduino_time_millis(void)
{
    return millis();
}

static mavlink_hal_timestamp_us_t arduino_time_micros(void)
{
    return (mavlink_hal_timestamp_us_t)micros();
}

static void arduino_delay_ms(uint32_t ms)
{
    delay(ms);
}

static void arduino_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

static mavlink_hal_error_t arduino_timer_start(
    mavlink_hal_timer_handle_t handle,
    uint32_t period_ms,
    mavlink_hal_timer_callback_t callback,
    void* user_data)
{
    /* Arduino doesn't have built-in timer support for callbacks */
    /* User should use external timer libraries like TimerOne, FlexiTimer2 */
    return MAVLINK_HAL_ERR_NOT_SUPPORTED;
}

static mavlink_hal_error_t arduino_timer_stop(mavlink_hal_timer_handle_t handle)
{
    return MAVLINK_HAL_ERR_NOT_SUPPORTED;
}

/* ============================================================================
 * PWM Operations - Arduino Implementation
 * ============================================================================ */

/**
 * @brief Arduino PWM handle - just the pin number
 */
typedef struct {
    uint8_t pin;
} arduino_pwm_t;

static mavlink_hal_error_t arduino_pwm_init(
    mavlink_hal_pwm_handle_t handle,
    const mavlink_hal_pwm_config_t* config)
{
    if (handle == NULL || config == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_pwm_t* pwm = (arduino_pwm_t*)handle;

    pinMode(pwm->pin, OUTPUT);

#if defined(__SAM3X8E__)  /* Arduino Due */
    /* Due has 12-bit PWM */
    analogWriteResolution(config->resolution_bits);
#endif

    /* Set initial duty cycle */
    analogWrite(pwm->pin, config->initial_duty);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_pwm_set_duty(
    mavlink_hal_pwm_handle_t handle,
    uint16_t duty_cycle)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_pwm_t* pwm = (arduino_pwm_t*)handle;
    analogWrite(pwm->pin, duty_cycle);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_pwm_set_duty_percent(
    mavlink_hal_pwm_handle_t handle,
    float percent)
{
    if (handle == NULL || percent < 0.0f || percent > 100.0f) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_pwm_t* pwm = (arduino_pwm_t*)handle;
    uint16_t max_val = (1 << ARDUINO_PWM_RESOLUTION) - 1;
    uint16_t duty = (uint16_t)((percent * max_val) / 100.0f);

    analogWrite(pwm->pin, duty);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_pwm_start(mavlink_hal_pwm_handle_t handle)
{
    /* PWM automatically starts when analogWrite is called */
    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_pwm_stop(mavlink_hal_pwm_handle_t handle)
{
    if (handle == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_pwm_t* pwm = (arduino_pwm_t*)handle;
    analogWrite(pwm->pin, 0);  /* Set duty cycle to 0 */

    return MAVLINK_HAL_OK;
}

/* ============================================================================
 * ADC Operations - Arduino Implementation
 * ============================================================================ */

/**
 * @brief Arduino ADC handle
 */
typedef struct {
    uint8_t pin;
    float vref;
} arduino_adc_t;

static mavlink_hal_error_t arduino_adc_init(
    mavlink_hal_adc_handle_t handle,
    const mavlink_hal_adc_config_t* config)
{
    if (handle == NULL || config == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_adc_t* adc = (arduino_adc_t*)handle;
    adc->vref = config->reference_voltage;

    pinMode(adc->pin, INPUT);

#if defined(__SAM3X8E__)  /* Arduino Due */
    analogReadResolution(config->resolution_bits);
#endif

#if defined(__AVR__)
    /* Set ADC reference on AVR */
    if (config->reference_voltage == 5.0f) {
        analogReference(DEFAULT);
    } else if (config->reference_voltage == 1.1f) {
        analogReference(INTERNAL);
    } else if (config->reference_voltage == 3.3f) {
        analogReference(EXTERNAL);
    }
#endif

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_adc_read(
    mavlink_hal_adc_handle_t handle,
    uint16_t* value)
{
    if (handle == NULL || value == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_adc_t* adc = (arduino_adc_t*)handle;
    *value = analogRead(adc->pin);

    return MAVLINK_HAL_OK;
}

static mavlink_hal_error_t arduino_adc_read_voltage(
    mavlink_hal_adc_handle_t handle,
    float* voltage)
{
    if (handle == NULL || voltage == NULL) {
        return MAVLINK_HAL_ERR_INVALID_PARAM;
    }

    arduino_adc_t* adc = (arduino_adc_t*)handle;
    uint16_t raw_value = analogRead(adc->pin);

#if defined(__AVR__)
    uint16_t max_val = 1023;  /* 10-bit ADC on AVR */
#elif defined(__SAM3X8E__)
    uint16_t max_val = 4095;  /* 12-bit ADC on Due */
#else
    uint16_t max_val = 1023;  /* Default to 10-bit */
#endif

    *voltage = ((float)raw_value * adc->vref) / (float)max_val;

    return MAVLINK_HAL_OK;
}

/* ============================================================================
 * HAL Registration
 * ============================================================================ */

MAVLINK_HAL_REGISTER(
    .platform = MAVLINK_HAL_PLATFORM_ARDUINO,
    .features = MAVLINK_HAL_FEATURE_UART |
                MAVLINK_HAL_FEATURE_GPIO |
                MAVLINK_HAL_FEATURE_TIMER |
                MAVLINK_HAL_FEATURE_PWM |
                MAVLINK_HAL_FEATURE_ADC,
    .version_major = MAVLINK_HAL_VERSION_MAJOR,
    .version_minor = MAVLINK_HAL_VERSION_MINOR,
    .version_patch = MAVLINK_HAL_VERSION_PATCH,
    .platform_name = "Arduino",

    /* UART operations */
    .uart_init = arduino_uart_init,
    .uart_deinit = arduino_uart_deinit,
    .uart_send = arduino_uart_send,
    .uart_receive = arduino_uart_receive,
    .uart_register_rx_callback = arduino_uart_register_rx_callback,
    .uart_available = arduino_uart_available,

    /* GPIO operations */
    .gpio_init = arduino_gpio_init,
    .gpio_write = arduino_gpio_write,
    .gpio_read = arduino_gpio_read,
    .gpio_toggle = arduino_gpio_toggle,
    .gpio_register_irq = arduino_gpio_register_irq,

    /* Timer operations */
    .time_millis = arduino_time_millis,
    .time_micros = arduino_time_micros,
    .delay_ms = arduino_delay_ms,
    .delay_us = arduino_delay_us,
    .timer_start = arduino_timer_start,
    .timer_stop = arduino_timer_stop,

    /* PWM operations */
    .pwm_init = arduino_pwm_init,
    .pwm_set_duty = arduino_pwm_set_duty,
    .pwm_set_duty_percent = arduino_pwm_set_duty_percent,
    .pwm_start = arduino_pwm_start,
    .pwm_stop = arduino_pwm_stop,

    /* ADC operations */
    .adc_init = arduino_adc_init,
    .adc_read = arduino_adc_read,
    .adc_read_voltage = arduino_adc_read_voltage
);

#endif /* MAVLINK_HAL_ON_ARDUINO */
