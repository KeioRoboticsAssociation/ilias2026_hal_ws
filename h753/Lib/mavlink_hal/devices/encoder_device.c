/**
 * @file encoder_device.c
 * @brief Encoder sensor device implementation using unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "encoder_device.h"
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

static mavlink_device_error_t encoder_init(mavlink_device_t* device);
static mavlink_device_error_t encoder_update(mavlink_device_t* device, uint32_t dt_ms);
static mavlink_device_error_t encoder_shutdown(mavlink_device_t* device);
static mavlink_device_error_t encoder_enable(mavlink_device_t* device, bool enable);
static mavlink_device_error_t encoder_command(mavlink_device_t* device, const mavlink_device_command_t* command);
static mavlink_device_error_t encoder_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback);
static mavlink_device_error_t encoder_get_status(mavlink_device_t* device, mavlink_device_status_t* status);
static mavlink_device_error_t encoder_set_param(mavlink_device_t* device, const char* name, float value);
static mavlink_device_error_t encoder_get_param(mavlink_device_t* device, const char* name, float* value);

/* ========================================================================== */
/*  ENCODER VTABLE                                                            */
/* ========================================================================== */

static const mavlink_device_vtable_t encoder_vtable = {
    .init = encoder_init,
    .update = encoder_update,
    .shutdown = encoder_shutdown,
    .enable = encoder_enable,
    .command = encoder_command,
    .get_feedback = encoder_get_feedback,
    .get_status = encoder_get_status,
    .set_param = encoder_set_param,
    .get_param = encoder_get_param,
    .self_test = NULL,
    .calibrate = NULL,
};

/* ========================================================================== */
/*  UTILITY FUNCTIONS                                                         */
/* ========================================================================== */

/**
 * @brief Update encoder reading
 */
static void encoder_update_reading(encoder_private_data_t* priv)
{
    if (!priv || !priv->timer_handle) return;

#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    priv->prev_encoder_count = priv->encoder_count;
    priv->encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(htim);
#endif
}

/**
 * @brief Calculate position from encoder
 */
static float encoder_calculate_position(encoder_private_data_t* priv)
{
    /* Position in radians = (encoder_count / counts_per_rev) * 2π / gear_ratio */
    float revolutions = (float)priv->encoder_count / priv->encoder_counts_per_rev;
    float position = (revolutions * 2.0f * 3.14159265359f) / priv->gear_ratio;
    return position;
}

/**
 * @brief Calculate velocity from encoder
 */
static float encoder_calculate_velocity(encoder_private_data_t* priv, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* Velocity in rad/s = Δposition / dt */
    int32_t delta_count = priv->encoder_count - priv->prev_encoder_count;
    float delta_revolutions = (float)delta_count / priv->encoder_counts_per_rev;
    float delta_position = (delta_revolutions * 2.0f * 3.14159265359f) / priv->gear_ratio;
    float velocity = delta_position / dt;

    return velocity;
}

/* ========================================================================== */
/*  VTABLE IMPLEMENTATIONS                                                    */
/* ========================================================================== */

static mavlink_device_error_t encoder_init(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;

    /* Start encoder */
#if defined(STM32H7) || defined(STM32F4)
    if (priv->timer_handle) {
        TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
        if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
            return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
        }
    }
#endif

    /* Initialize counts */
    encoder_update_reading(priv);
    priv->prev_encoder_count = priv->encoder_count;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_update(mavlink_device_t* device, uint32_t dt_ms)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;
    float dt = dt_ms / 1000.0f;

    /* Update encoder reading */
    encoder_update_reading(priv);

    /* Calculate position and velocity */
    priv->position = encoder_calculate_position(priv);
    priv->velocity = encoder_calculate_velocity(priv, dt);

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_shutdown(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;

#if defined(STM32H7) || defined(STM32F4)
    /* Stop encoder */
    if (priv->timer_handle) {
        TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
        HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_enable(mavlink_device_t* device, bool enable)
{
    /* Encoder is always enabled, nothing to do */
    (void)device;
    (void)enable;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_command(mavlink_device_t* device, const mavlink_device_command_t* command)
{
    /* Encoder is a sensor, does not accept commands */
    (void)device;
    (void)command;
    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t encoder_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback)
{
    if (!device || !device->private_data || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;

    feedback->type = MAVLINK_DEVICE_TYPE_ENCODER;
    feedback->data.sensor.value = priv->position;     /* Primary value: position */
    feedback->data.sensor.values[0] = priv->position; /* Position */
    feedback->data.sensor.values[1] = priv->velocity; /* Velocity */
    feedback->data.sensor.value_count = 2;
    feedback->data.sensor.valid = true;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_get_status(mavlink_device_t* device, mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    *status = device->status;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t encoder_set_param(mavlink_device_t* device, const char* name, float value)
{
    if (!device || !device->private_data || !name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;

    if (strcmp(name, "counts_per_rev") == 0) {
        priv->encoder_counts_per_rev = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "gear_ratio") == 0) {
        priv->gear_ratio = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t encoder_get_param(mavlink_device_t* device, const char* name, float* value)
{
    if (!device || !device->private_data || !name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    encoder_private_data_t* priv = (encoder_private_data_t*)device->private_data;

    if (strcmp(name, "counts_per_rev") == 0) {
        *value = priv->encoder_counts_per_rev;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "gear_ratio") == 0) {
        *value = priv->gear_ratio;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

/* ========================================================================== */
/*  PUBLIC API                                                                */
/* ========================================================================== */

mavlink_device_t* encoder_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t timer_frequency_hz,
    float encoder_counts_per_rev,
    float gear_ratio)
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
    encoder_private_data_t* priv = (encoder_private_data_t*)malloc(sizeof(encoder_private_data_t));
    if (!priv) {
        free(device);
        return NULL;
    }

    /* Initialize private data */
    memset(priv, 0, sizeof(encoder_private_data_t));
    priv->timer_handle = timer_handle;
    priv->timer_frequency_hz = timer_frequency_hz;
    priv->encoder_counts_per_rev = encoder_counts_per_rev;
    priv->gear_ratio = gear_ratio;

    /* Initialize device */
    mavlink_device_error_t err = mavlink_device_init(
        device,
        MAVLINK_DEVICE_TYPE_ENCODER,
        id,
        name,
        config,
        &encoder_vtable,
        priv
    );

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        free(priv);
        free(device);
        return NULL;
    }

    return device;
}

const mavlink_device_vtable_t* encoder_device_get_vtable(void)
{
    return &encoder_vtable;
}
