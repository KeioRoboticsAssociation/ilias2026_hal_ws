/**
 * @file servo_device.c
 * @brief Servo device implementation using unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "servo_device.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Platform-specific includes (STM32 HAL) */
#ifdef STM32H7
#include "stm32h7xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif

/* ========================================================================== */
/*  FORWARD DECLARATIONS                                                      */
/* ========================================================================== */

static mavlink_device_error_t servo_init(mavlink_device_t* device);
static mavlink_device_error_t servo_update(mavlink_device_t* device, uint32_t dt_ms);
static mavlink_device_error_t servo_shutdown(mavlink_device_t* device);
static mavlink_device_error_t servo_enable(mavlink_device_t* device, bool enable);
static mavlink_device_error_t servo_command(mavlink_device_t* device, const mavlink_device_command_t* command);
static mavlink_device_error_t servo_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback);
static mavlink_device_error_t servo_get_status(mavlink_device_t* device, mavlink_device_status_t* status);
static mavlink_device_error_t servo_set_param(mavlink_device_t* device, const char* name, float value);
static mavlink_device_error_t servo_get_param(mavlink_device_t* device, const char* name, float* value);

/* ========================================================================== */
/*  SERVO VTABLE                                                              */
/* ========================================================================== */

static const mavlink_device_vtable_t servo_vtable = {
    .init = servo_init,
    .update = servo_update,
    .shutdown = servo_shutdown,
    .enable = servo_enable,
    .command = servo_command,
    .get_feedback = servo_get_feedback,
    .get_status = servo_get_status,
    .set_param = servo_set_param,
    .get_param = servo_get_param,
    .self_test = NULL,      /* Not implemented */
    .calibrate = NULL,      /* Not implemented */
};

/* ========================================================================== */
/*  UTILITY FUNCTIONS                                                         */
/* ========================================================================== */

uint16_t servo_angle_to_pulse(
    float angle,
    float min_angle,
    float max_angle,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us)
{
    /* Clamp angle to limits */
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;

    /* Linear mapping: angle -> pulse */
    float normalized = (angle - min_angle) / (max_angle - min_angle);
    uint16_t pulse_us = (uint16_t)(min_pulse_us + normalized * (max_pulse_us - min_pulse_us));

    return pulse_us;
}

float servo_pulse_to_angle(
    uint16_t pulse_us,
    float min_angle,
    float max_angle,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us)
{
    /* Clamp pulse to limits */
    if (pulse_us < min_pulse_us) pulse_us = min_pulse_us;
    if (pulse_us > max_pulse_us) pulse_us = max_pulse_us;

    /* Linear mapping: pulse -> angle */
    float normalized = (float)(pulse_us - min_pulse_us) / (float)(max_pulse_us - min_pulse_us);
    float angle = min_angle + normalized * (max_angle - min_angle);

    return angle;
}

/**
 * @brief Set PWM pulse width (platform-specific)
 */
static mavlink_device_error_t servo_set_pwm(servo_private_data_t* priv, uint16_t pulse_us)
{
    if (!priv || !priv->timer_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;

    /* Calculate compare value from pulse width */
    uint32_t period = htim->Init.Period;
    uint32_t prescaler = htim->Init.Prescaler;
    uint32_t timer_clock = priv->timer_frequency_hz;

    /* CCR = (pulse_us * timer_clock) / (prescaler + 1) / 1000000 */
    uint32_t compare = (uint32_t)((uint64_t)pulse_us * timer_clock / (prescaler + 1) / 1000000ULL);

    /* Set compare value */
    __HAL_TIM_SET_COMPARE(htim, priv->channel, compare);

    priv->current_pulse_us = pulse_us;
    return MAVLINK_DEVICE_ERROR_NONE;
#else
    /* Platform not supported */
    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
#endif
}

/* ========================================================================== */
/*  VTABLE IMPLEMENTATIONS                                                    */
/* ========================================================================== */

static mavlink_device_error_t servo_init(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

    /* Cache configuration */
    priv->min_angle = device->config.limits.min_position;
    priv->max_angle = device->config.limits.max_position;
    priv->min_pulse_us = device->config.config.servo.min_pulse_us;
    priv->max_pulse_us = device->config.config.servo.max_pulse_us;
    priv->neutral_angle = device->config.config.servo.neutral_angle;

    /* Set to neutral position */
    uint16_t neutral_pulse = servo_angle_to_pulse(
        priv->neutral_angle,
        priv->min_angle,
        priv->max_angle,
        priv->min_pulse_us,
        priv->max_pulse_us
    );

    priv->current_angle = priv->neutral_angle;
    priv->current_pulse_us = neutral_pulse;

#if defined(STM32H7) || defined(STM32F4)
    /* Start PWM */
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    if (HAL_TIM_PWM_Start(htim, priv->channel) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
    }
#endif

    /* Set initial PWM */
    return servo_set_pwm(priv, neutral_pulse);
}

static mavlink_device_error_t servo_update(mavlink_device_t* device, uint32_t dt_ms)
{
    /* Servo has no periodic update needed */
    (void)device;
    (void)dt_ms;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t servo_shutdown(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

#if defined(STM32H7) || defined(STM32F4)
    /* Stop PWM */
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    HAL_TIM_PWM_Stop(htim, priv->channel);
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t servo_enable(mavlink_device_t* device, bool enable)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;

    if (enable) {
        /* Start PWM */
        if (HAL_TIM_PWM_Start(htim, priv->channel) != HAL_OK) {
            return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
        }
    } else {
        /* Stop PWM */
        HAL_TIM_PWM_Stop(htim, priv->channel);
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t servo_command(mavlink_device_t* device, const mavlink_device_command_t* command)
{
    if (!device || !device->private_data || !command) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

    /* Only position control supported */
    if (command->mode != MAVLINK_CONTROL_MODE_POSITION) {
        return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }

    float target_angle = command->data.position.target;

    /* Clamp to limits */
    if (target_angle < priv->min_angle) target_angle = priv->min_angle;
    if (target_angle > priv->max_angle) target_angle = priv->max_angle;

    /* Convert angle to pulse width */
    uint16_t pulse_us = servo_angle_to_pulse(
        target_angle,
        priv->min_angle,
        priv->max_angle,
        priv->min_pulse_us,
        priv->max_pulse_us
    );

    /* Set PWM */
    mavlink_device_error_t err = servo_set_pwm(priv, pulse_us);
    if (err == MAVLINK_DEVICE_ERROR_NONE) {
        priv->current_angle = target_angle;
    }

    return err;
}

static mavlink_device_error_t servo_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback)
{
    if (!device || !device->private_data || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

    /* Populate feedback */
    feedback->type = MAVLINK_DEVICE_TYPE_SERVO;
    feedback->data.motor.position = priv->current_angle;
    feedback->data.motor.velocity = 0.0f;  /* No velocity feedback */
    feedback->data.motor.current = 0.0f;   /* No current feedback */
    feedback->data.motor.torque = 0.0f;    /* No torque feedback */
    feedback->data.motor.temperature = 0.0f; /* No temperature feedback */

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t servo_get_status(mavlink_device_t* device, mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Copy status from device */
    *status = device->status;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t servo_set_param(mavlink_device_t* device, const char* name, float value)
{
    if (!device || !device->private_data || !name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

    /* Supported parameters */
    if (strcmp(name, "min_pulse_us") == 0) {
        priv->min_pulse_us = (uint16_t)value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_pulse_us") == 0) {
        priv->max_pulse_us = (uint16_t)value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "neutral_angle") == 0) {
        priv->neutral_angle = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "min_angle") == 0) {
        priv->min_angle = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_angle") == 0) {
        priv->max_angle = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t servo_get_param(mavlink_device_t* device, const char* name, float* value)
{
    if (!device || !device->private_data || !name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    servo_private_data_t* priv = (servo_private_data_t*)device->private_data;

    /* Supported parameters */
    if (strcmp(name, "min_pulse_us") == 0) {
        *value = (float)priv->min_pulse_us;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_pulse_us") == 0) {
        *value = (float)priv->max_pulse_us;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "neutral_angle") == 0) {
        *value = priv->neutral_angle;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "min_angle") == 0) {
        *value = priv->min_angle;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_angle") == 0) {
        *value = priv->max_angle;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

/* ========================================================================== */
/*  PUBLIC API                                                                */
/* ========================================================================== */

mavlink_device_t* servo_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t channel,
    uint32_t timer_frequency_hz)
{
    if (!config || !timer_handle) {
        return NULL;
    }

    /* Allocate device */
    mavlink_device_t* device = (mavlink_device_t*)malloc(sizeof(mavlink_device_t));
    if (!device) {
        return NULL;
    }

    /* Allocate private data */
    servo_private_data_t* priv = (servo_private_data_t*)malloc(sizeof(servo_private_data_t));
    if (!priv) {
        free(device);
        return NULL;
    }

    /* Initialize private data */
    memset(priv, 0, sizeof(servo_private_data_t));
    priv->timer_handle = timer_handle;
    priv->channel = channel;
    priv->timer_frequency_hz = timer_frequency_hz;

    /* Initialize device */
    mavlink_device_error_t err = mavlink_device_init(
        device,
        MAVLINK_DEVICE_TYPE_SERVO,
        id,
        name,
        config,
        &servo_vtable,
        priv
    );

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        free(priv);
        free(device);
        return NULL;
    }

    return device;
}

const mavlink_device_vtable_t* servo_device_get_vtable(void)
{
    return &servo_vtable;
}
