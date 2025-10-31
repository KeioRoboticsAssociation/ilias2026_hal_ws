/**
 * @file dc_controller.c
 * @brief DC motor controller implementation (simplified for H753)
 */

#include "dc_controller.h"
#include <string.h>

/* ============================================================================
 * Motor Interface Implementation
 * ============================================================================ */

static error_code_t dc_motor_initialize(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;

    // Initialize state
    controller->state.status = ERROR_NOT_INITIALIZED;
    controller->state.enabled = false;
    controller->state.last_update_time = hw_get_tick();

    // Initialize private data
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;
    priv->speed_integral = 0.0f;
    priv->speed_last_error = 0.0f;
    priv->position_integral = 0.0f;
    priv->position_last_error = 0.0f;
    priv->current_duty_cycle = 0.0f;

    // Start PWM
    error_code_t err = hw_timer_start_pwm(priv->timer_id, priv->channel);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    // Set to zero initially
    uint32_t period = hw_timer_get_period(priv->timer_id);
    hw_timer_set_pwm(priv->timer_id, priv->channel, 0);

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t dc_motor_update(motor_controller_t* controller, float delta_time) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;

    // Check watchdog
    uint32_t now = hw_get_tick();
    uint32_t elapsed = now - priv->last_watchdog_reset;

    if (elapsed > priv->config.watchdog_timeout_ms) {
        if (!priv->watchdog_expired) {
            priv->watchdog_expired = true;
            controller->state.status = ERROR_TIMEOUT;
            controller->state.timeout_count++;
            hw_timer_set_pwm(priv->timer_id, priv->channel, 0);  // Stop motor
        }
    }

    controller->state.last_update_time = now;
    return controller->state.status;
}

static error_code_t dc_motor_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (!controller || !controller->private_data || !cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    if (cmd->motor_id != controller->id) {
        return ERROR_INVALID_PARAMETER;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;
    const dc_motor_config_t* config = &priv->config;

    // Reset watchdog
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    // Handle enable/disable
    if (!cmd->enable) {
        return motor_set_enabled(controller, false);
    }

    if (!controller->state.enabled) {
        motor_set_enabled(controller, true);
    }

    // Process command based on mode
    float duty_cycle = 0.0f;

    switch (cmd->mode) {
        case CONTROL_MODE_DUTY_CYCLE:
            // Direct duty cycle control (-1.0 to 1.0)
            duty_cycle = constrain_float(cmd->target_value, -1.0f, 1.0f);
            break;

        case CONTROL_MODE_VELOCITY:
            // Velocity control with PID (simplified - no encoder feedback)
            controller->state.target_velocity = constrain_float(
                cmd->target_value,
                -config->max_speed_rad_s,
                config->max_speed_rad_s);

            // Simple open-loop velocity mapping
            duty_cycle = controller->state.target_velocity / config->max_speed_rad_s;
            break;

        default:
            return ERROR_INVALID_PARAMETER;
    }

    // Apply direction inversion
    if (config->direction_inverted) {
        duty_cycle = -duty_cycle;
    }

    priv->current_duty_cycle = duty_cycle;

    // Convert duty cycle to PWM value
    uint32_t period = hw_timer_get_period(priv->timer_id);
    uint32_t pulse = (uint32_t)((fabsf(duty_cycle) * (float)period));

    error_code_t err = hw_timer_set_pwm(priv->timer_id, priv->channel, pulse);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t dc_motor_set_enabled(motor_controller_t* controller, bool enabled) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;

    if (enabled == controller->state.enabled) {
        return ERROR_OK;
    }

    error_code_t err;
    if (enabled) {
        err = hw_timer_start_pwm(priv->timer_id, priv->channel);
    } else {
        hw_timer_set_pwm(priv->timer_id, priv->channel, 0);
        err = hw_timer_stop_pwm(priv->timer_id, priv->channel);
    }

    if (err != ERROR_OK) {
        return err;
    }

    controller->state.enabled = enabled;
    return ERROR_OK;
}

static void dc_motor_emergency_stop(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;

    // Stop motor immediately
    hw_timer_set_pwm(priv->timer_id, priv->channel, 0);

    controller->state.enabled = false;
    controller->state.status = ERROR_SAFETY_VIOLATION;
}

static void dc_motor_reset_watchdog(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (controller->state.status == ERROR_TIMEOUT) {
        controller->state.status = ERROR_OK;
    }
}

static error_code_t dc_motor_self_test(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Simple self-test: try to set zero PWM
    dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;
    error_code_t err = hw_timer_set_pwm(priv->timer_id, priv->channel, 0);
    return err;
}

static motor_state_t dc_motor_get_state(const motor_controller_t* controller) {
    if (!controller) {
        motor_state_t empty = {0};
        empty.status = ERROR_INVALID_PARAMETER;
        return empty;
    }
    return controller->state;
}

static uint8_t dc_motor_get_id(const motor_controller_t* controller) {
    if (!controller) {
        return 0;
    }
    return controller->id;
}

/* ============================================================================
 * Virtual Function Table
 * ============================================================================ */

static const motor_vtable_t dc_motor_vtable = {
    .initialize = dc_motor_initialize,
    .update = dc_motor_update,
    .set_command = dc_motor_set_command,
    .set_enabled = dc_motor_set_enabled,
    .emergency_stop = dc_motor_emergency_stop,
    .reset_watchdog = dc_motor_reset_watchdog,
    .self_test = dc_motor_self_test,
    .get_state = dc_motor_get_state,
    .get_id = dc_motor_get_id
};

const motor_vtable_t* dc_motor_controller_get_vtable(void) {
    return &dc_motor_vtable;
}

/* ============================================================================
 * DC Motor Controller Creation
 * ============================================================================ */

error_code_t dc_motor_controller_create(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const dc_motor_config_t* config,
    motor_controller_t* controller,
    dc_motor_private_t* private_data)
{
    if (!config || !controller || !private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Initialize controller base
    memset(controller, 0, sizeof(motor_controller_t));
    controller->vtable = &dc_motor_vtable;
    controller->id = id;
    controller->type = MOTOR_TYPE_DC;
    controller->private_data = private_data;

    // Initialize private data
    memset(private_data, 0, sizeof(dc_motor_private_t));
    private_data->config = *config;
    private_data->timer_id = timer_id;
    private_data->channel = channel;

    return ERROR_OK;
}
