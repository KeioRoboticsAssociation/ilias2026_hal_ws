/**
 * @file dc_motor_device.c
 * @brief DC motor device implementation using unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "dc_motor_device.h"
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
/*  PID CONTROLLER IMPLEMENTATION                                             */
/* ========================================================================== */

void pid_init(pid_controller_t* pid, float kp, float ki, float kd,
              float output_min, float output_max, float integral_max)
{
    if (!pid) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_max = integral_max;
}

float pid_update(pid_controller_t* pid, float setpoint, float measurement, float dt)
{
    if (!pid || dt <= 0.0f) return 0.0f;

    /* Calculate error */
    float error = setpoint - measurement;

    /* Proportional term */
    float p_term = pid->kp * error;

    /* Integral term with anti-windup */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* Derivative term */
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;

    /* Calculate output */
    float output = p_term + i_term + d_term;

    /* Clamp output */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(pid_controller_t* pid)
{
    if (!pid) return;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* ========================================================================== */
/*  FORWARD DECLARATIONS                                                      */
/* ========================================================================== */

static mavlink_device_error_t dc_motor_init(mavlink_device_t* device);
static mavlink_device_error_t dc_motor_update(mavlink_device_t* device, uint32_t dt_ms);
static mavlink_device_error_t dc_motor_shutdown(mavlink_device_t* device);
static mavlink_device_error_t dc_motor_enable(mavlink_device_t* device, bool enable);
static mavlink_device_error_t dc_motor_command(mavlink_device_t* device, const mavlink_device_command_t* command);
static mavlink_device_error_t dc_motor_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback);
static mavlink_device_error_t dc_motor_get_status(mavlink_device_t* device, mavlink_device_status_t* status);
static mavlink_device_error_t dc_motor_set_param(mavlink_device_t* device, const char* name, float value);
static mavlink_device_error_t dc_motor_get_param(mavlink_device_t* device, const char* name, float* value);

/* ========================================================================== */
/*  DC MOTOR VTABLE                                                           */
/* ========================================================================== */

static const mavlink_device_vtable_t dc_motor_vtable = {
    .init = dc_motor_init,
    .update = dc_motor_update,
    .shutdown = dc_motor_shutdown,
    .enable = dc_motor_enable,
    .command = dc_motor_command,
    .get_feedback = dc_motor_get_feedback,
    .get_status = dc_motor_get_status,
    .set_param = dc_motor_set_param,
    .get_param = dc_motor_get_param,
    .self_test = NULL,
    .calibrate = NULL,
};

/* ========================================================================== */
/*  UTILITY FUNCTIONS                                                         */
/* ========================================================================== */

/**
 * @brief Set motor duty cycle (platform-specific)
 *
 * @param priv Private data
 * @param duty_cycle Duty cycle (-1.0 to 1.0)
 * @return Error code
 */
static mavlink_device_error_t dc_motor_set_duty_cycle(dc_motor_private_data_t* priv, float duty_cycle)
{
    if (!priv || !priv->timer_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Clamp duty cycle */
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    if (duty_cycle < -1.0f) duty_cycle = -1.0f;

    priv->duty_cycle = duty_cycle;

#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    uint32_t period = htim->Init.Period;

    if (duty_cycle >= 0.0f) {
        /* Forward direction */
        uint32_t compare_fwd = (uint32_t)(duty_cycle * period);
        __HAL_TIM_SET_COMPARE(htim, priv->channel_fwd, compare_fwd);
        __HAL_TIM_SET_COMPARE(htim, priv->channel_rev, 0);
    } else {
        /* Reverse direction */
        uint32_t compare_rev = (uint32_t)(fabsf(duty_cycle) * period);
        __HAL_TIM_SET_COMPARE(htim, priv->channel_fwd, 0);
        __HAL_TIM_SET_COMPARE(htim, priv->channel_rev, compare_rev);
    }

    return MAVLINK_DEVICE_ERROR_NONE;
#else
    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
#endif
}

/**
 * @brief Update encoder reading (platform-specific)
 */
static void dc_motor_update_encoder(dc_motor_private_data_t* priv)
{
    if (!priv || !priv->encoder_handle) return;

#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim_enc = (TIM_HandleTypeDef*)priv->encoder_handle;
    priv->prev_encoder_count = priv->encoder_count;
    priv->encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(htim_enc);
#endif
}

/**
 * @brief Calculate position from encoder
 */
static float dc_motor_calculate_position(dc_motor_private_data_t* priv)
{
    /* Position in radians = (encoder_count / counts_per_rev) * 2π / gear_ratio */
    float revolutions = (float)priv->encoder_count / priv->encoder_counts_per_rev;
    float position = (revolutions * 2.0f * 3.14159265359f) / priv->gear_ratio;
    return position;
}

/**
 * @brief Calculate velocity from encoder
 */
static float dc_motor_calculate_velocity(dc_motor_private_data_t* priv, float dt)
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

static mavlink_device_error_t dc_motor_init(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    /* Initialize PID controllers */
    pid_init(&priv->position_pid,
             device->config.pid_position.kp,
             device->config.pid_position.ki,
             device->config.pid_position.kd,
             -1.0f, 1.0f, 100.0f);

    pid_init(&priv->velocity_pid,
             device->config.pid_velocity.kp,
             device->config.pid_velocity.ki,
             device->config.pid_velocity.kd,
             -1.0f, 1.0f, 100.0f);

    /* Start PWM */
#if defined(STM32H7) || defined(STM32F4)
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    if (HAL_TIM_PWM_Start(htim, priv->channel_fwd) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
    }
    if (HAL_TIM_PWM_Start(htim, priv->channel_rev) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
    }

    /* Start encoder */
    if (priv->encoder_handle) {
        TIM_HandleTypeDef* htim_enc = (TIM_HandleTypeDef*)priv->encoder_handle;
        if (HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL) != HAL_OK) {
            return MAVLINK_DEVICE_ERROR_HARDWARE_ERROR;
        }
    }
#endif

    /* Initialize to zero duty cycle */
    return dc_motor_set_duty_cycle(priv, 0.0f);
}

static mavlink_device_error_t dc_motor_update(mavlink_device_t* device, uint32_t dt_ms)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;
    float dt = dt_ms / 1000.0f;

    /* Update encoder */
    dc_motor_update_encoder(priv);

    /* Calculate position and velocity */
    priv->position = dc_motor_calculate_position(priv);
    priv->velocity = dc_motor_calculate_velocity(priv, dt);

    /* Run closed-loop control if enabled */
    if (!device->status.enabled) {
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    float duty_cycle = 0.0f;

    switch (priv->control_mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            /* Position control: PID(position) -> velocity -> PID(velocity) -> duty */
            {
                float target_vel = pid_update(&priv->position_pid, priv->target_position, priv->position, dt);
                duty_cycle = pid_update(&priv->velocity_pid, target_vel, priv->velocity, dt);
            }
            break;

        case MAVLINK_CONTROL_MODE_VELOCITY:
            /* Velocity control: PID(velocity) -> duty */
            duty_cycle = pid_update(&priv->velocity_pid, priv->target_velocity, priv->velocity, dt);
            break;

        case MAVLINK_CONTROL_MODE_DUTY_CYCLE:
            /* Open-loop duty cycle control */
            duty_cycle = priv->target_duty_cycle;
            break;

        default:
            return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }

    /* Set duty cycle */
    return dc_motor_set_duty_cycle(priv, duty_cycle);
}

static mavlink_device_error_t dc_motor_shutdown(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    /* Stop motor */
    dc_motor_set_duty_cycle(priv, 0.0f);

#if defined(STM32H7) || defined(STM32F4)
    /* Stop PWM */
    TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)priv->timer_handle;
    HAL_TIM_PWM_Stop(htim, priv->channel_fwd);
    HAL_TIM_PWM_Stop(htim, priv->channel_rev);

    /* Stop encoder */
    if (priv->encoder_handle) {
        TIM_HandleTypeDef* htim_enc = (TIM_HandleTypeDef*)priv->encoder_handle;
        HAL_TIM_Encoder_Stop(htim_enc, TIM_CHANNEL_ALL);
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t dc_motor_enable(mavlink_device_t* device, bool enable)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    if (!enable) {
        /* Stop motor and reset PIDs */
        dc_motor_set_duty_cycle(priv, 0.0f);
        pid_reset(&priv->position_pid);
        pid_reset(&priv->velocity_pid);
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t dc_motor_command(mavlink_device_t* device, const mavlink_device_command_t* command)
{
    if (!device || !device->private_data || !command) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    priv->control_mode = command->mode;

    switch (command->mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            priv->target_position = command->data.position.target;
            break;

        case MAVLINK_CONTROL_MODE_VELOCITY:
            priv->target_velocity = command->data.velocity.target;
            break;

        case MAVLINK_CONTROL_MODE_DUTY_CYCLE:
            priv->target_duty_cycle = command->data.duty_cycle.duty;
            return dc_motor_set_duty_cycle(priv, priv->target_duty_cycle);

        default:
            return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t dc_motor_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback)
{
    if (!device || !device->private_data || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    feedback->type = MAVLINK_DEVICE_TYPE_DC_MOTOR;
    feedback->data.motor.position = priv->position;
    feedback->data.motor.velocity = priv->velocity;
    feedback->data.motor.current = priv->current;
    feedback->data.motor.torque = 0.0f;
    feedback->data.motor.temperature = 0.0f;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t dc_motor_get_status(mavlink_device_t* device, mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    *status = device->status;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t dc_motor_set_param(mavlink_device_t* device, const char* name, float value)
{
    if (!device || !device->private_data || !name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    /* Position PID parameters */
    if (strcmp(name, "pos_kp") == 0) {
        priv->position_pid.kp = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "pos_ki") == 0) {
        priv->position_pid.ki = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "pos_kd") == 0) {
        priv->position_pid.kd = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }
    /* Velocity PID parameters */
    else if (strcmp(name, "vel_kp") == 0) {
        priv->velocity_pid.kp = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "vel_ki") == 0) {
        priv->velocity_pid.ki = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "vel_kd") == 0) {
        priv->velocity_pid.kd = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t dc_motor_get_param(mavlink_device_t* device, const char* name, float* value)
{
    if (!device || !device->private_data || !name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)device->private_data;

    /* Position PID parameters */
    if (strcmp(name, "pos_kp") == 0) {
        *value = priv->position_pid.kp;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "pos_ki") == 0) {
        *value = priv->position_pid.ki;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "pos_kd") == 0) {
        *value = priv->position_pid.kd;
        return MAVLINK_DEVICE_ERROR_NONE;
    }
    /* Velocity PID parameters */
    else if (strcmp(name, "vel_kp") == 0) {
        *value = priv->velocity_pid.kp;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "vel_ki") == 0) {
        *value = priv->velocity_pid.ki;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "vel_kd") == 0) {
        *value = priv->velocity_pid.kd;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

/* ========================================================================== */
/*  PUBLIC API                                                                */
/* ========================================================================== */

mavlink_device_t* dc_motor_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t channel_fwd,
    uint32_t channel_rev,
    void* encoder_handle,
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
    dc_motor_private_data_t* priv = (dc_motor_private_data_t*)malloc(sizeof(dc_motor_private_data_t));
    if (!priv) {
        free(device);
        return NULL;
    }

    /* Initialize private data */
    memset(priv, 0, sizeof(dc_motor_private_data_t));
    priv->timer_handle = timer_handle;
    priv->channel_fwd = channel_fwd;
    priv->channel_rev = channel_rev;
    priv->encoder_handle = encoder_handle;
    priv->timer_frequency_hz = timer_frequency_hz;
    priv->encoder_counts_per_rev = encoder_counts_per_rev;
    priv->gear_ratio = gear_ratio;
    priv->control_mode = MAVLINK_CONTROL_MODE_DUTY_CYCLE;

    /* Initialize device */
    mavlink_device_error_t err = mavlink_device_init(
        device,
        MAVLINK_DEVICE_TYPE_DC_MOTOR,
        id,
        name,
        config,
        &dc_motor_vtable,
        priv
    );

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        free(priv);
        free(device);
        return NULL;
    }

    return device;
}

const mavlink_device_vtable_t* dc_motor_device_get_vtable(void)
{
    return &dc_motor_vtable;
}
