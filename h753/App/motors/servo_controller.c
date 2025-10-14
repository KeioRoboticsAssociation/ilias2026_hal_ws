/**
 * @file servo_controller.c
 * @brief Servo motor controller implementation for H753
 */

#include "servo_controller.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * Private Helper Functions
 * ============================================================================ */

/**
 * @brief Convert degrees to pulse width in microseconds
 */
static float degrees_to_pulse_us(const servo_config_t* config, float degrees) {
    // Apply direction inversion
    if (config->direction_inverted) {
        degrees = -degrees;
    }

    // Apply offset
    degrees += config->initial_offset;

    // Constrain to angle limits
    degrees = constrain_float(degrees, config->min_angle, config->max_angle);

    // Map angle to pulse width
    // degrees range: [min_angle, max_angle]
    // pulse range: [pulse_min_us, pulse_max_us]
    float angle_range = config->max_angle - config->min_angle;
    float pulse_range = (float)(config->pulse_max_us - config->pulse_min_us);

    if (angle_range < 0.001f) {
        return (float)config->pulse_neutral_us;
    }

    float normalized = (degrees - config->min_angle) / angle_range;
    float pulse_us = config->pulse_min_us + (normalized * pulse_range);

    return constrain_float(pulse_us, config->pulse_min_us, config->pulse_max_us);
}

/**
 * @brief Convert pulse width to degrees
 */
static float pulse_us_to_degrees(const servo_config_t* config, float pulse_us) {
    // Map pulse width to angle
    float angle_range = config->max_angle - config->min_angle;
    float pulse_range = (float)(config->pulse_max_us - config->pulse_min_us);

    if (pulse_range < 0.001f) {
        return 0.0f;
    }

    float normalized = (pulse_us - config->pulse_min_us) / pulse_range;
    float degrees = config->min_angle + (normalized * angle_range);

    // Remove offset
    degrees -= config->initial_offset;

    // Apply direction inversion
    if (config->direction_inverted) {
        degrees = -degrees;
    }

    return degrees;
}

/**
 * @brief Check watchdog timer
 */
static void check_watchdog(motor_controller_t* controller) {
    servo_private_t* priv = (servo_private_t*)controller->private_data;
    const servo_config_t* config = &priv->config;

    uint32_t now = hw_get_tick();
    uint32_t elapsed = now - priv->last_watchdog_reset;

    if (elapsed > config->watchdog_timeout_ms) {
        if (!priv->watchdog_expired) {
            priv->watchdog_expired = true;
            controller->state.status = ERROR_TIMEOUT;
            controller->state.timeout_count++;

            // Apply failsafe behavior
            switch (config->failsafe_behavior) {
                case SERVO_FAILSAFE_NEUTRAL:
                    hw_timer_set_pwm_us(priv->timer_id, priv->channel, config->pulse_neutral_us);
                    break;
                case SERVO_FAILSAFE_HOLD:
                    // Keep current position
                    break;
                case SERVO_FAILSAFE_DISABLE:
                    hw_timer_stop_pwm(priv->timer_id, priv->channel);
                    controller->state.enabled = false;
                    break;
            }
        }
    }
}

/* ============================================================================
 * Motor Interface Implementation
 * ============================================================================ */

static error_code_t servo_initialize(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;
    const servo_config_t* config = &priv->config;

    // Initialize state
    controller->state.status = ERROR_NOT_INITIALIZED;
    controller->state.enabled = !config->start_disabled;
    controller->state.current_position = config->startup_angle_deg;
    controller->state.target_position = config->startup_angle_deg;
    controller->state.last_update_time = hw_get_tick();

    // Initialize private data
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    // Calculate initial pulse width
    priv->current_pulse_us = degrees_to_pulse_us(config, config->startup_angle_deg);

    // Start PWM
    error_code_t err = hw_timer_start_pwm(priv->timer_id, priv->channel);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    // Set initial position
    err = hw_timer_set_pwm_us(priv->timer_id, priv->channel, (uint32_t)priv->current_pulse_us);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t servo_update(motor_controller_t* controller, float delta_time) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;

    // Check watchdog
    check_watchdog(controller);

    // Update timestamp
    controller->state.last_update_time = hw_get_tick();

    // If watchdog expired, don't update position
    if (priv->watchdog_expired) {
        return ERROR_TIMEOUT;
    }

    // Update current position (for servos, target = current in steady state)
    controller->state.current_position = controller->state.target_position;

    return controller->state.status;
}

static error_code_t servo_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (!controller || !controller->private_data || !cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    if (cmd->motor_id != controller->id) {
        return ERROR_INVALID_PARAMETER;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;
    const servo_config_t* config = &priv->config;

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
    float target_angle = 0.0f;

    switch (cmd->mode) {
        case CONTROL_MODE_POSITION:
            // Target value is angle in degrees
            target_angle = constrain_float(cmd->target_value, config->min_angle, config->max_angle);
            break;

        case CONTROL_MODE_DUTY_CYCLE:
            // Target value is normalized PWM duty cycle (0.0 to 1.0)
            // Map to pulse width
            {
                float pulse_range = config->pulse_max_us - config->pulse_min_us;
                float pulse_us = config->pulse_min_us + (cmd->target_value * pulse_range);
                target_angle = pulse_us_to_degrees(config, pulse_us);
            }
            break;

        default:
            return ERROR_INVALID_PARAMETER;
    }

    // Update target position
    controller->state.target_position = target_angle;

    // Convert to pulse width
    priv->current_pulse_us = degrees_to_pulse_us(config, target_angle);

    // Set PWM
    error_code_t err = hw_timer_set_pwm_us(priv->timer_id, priv->channel, (uint32_t)priv->current_pulse_us);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t servo_set_enabled(motor_controller_t* controller, bool enabled) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;

    if (enabled == controller->state.enabled) {
        return ERROR_OK;  // Already in desired state
    }

    error_code_t err;
    if (enabled) {
        err = hw_timer_start_pwm(priv->timer_id, priv->channel);
        if (err != ERROR_OK) {
            return err;
        }
        // Set to current position
        hw_timer_set_pwm_us(priv->timer_id, priv->channel, (uint32_t)priv->current_pulse_us);
    } else {
        err = hw_timer_stop_pwm(priv->timer_id, priv->channel);
        if (err != ERROR_OK) {
            return err;
        }
    }

    controller->state.enabled = enabled;
    return ERROR_OK;
}

static void servo_emergency_stop(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;
    const servo_config_t* config = &priv->config;

    // Move to neutral position
    hw_timer_set_pwm_us(priv->timer_id, priv->channel, config->pulse_neutral_us);

    controller->state.enabled = false;
    controller->state.status = ERROR_SAFETY_VIOLATION;
}

static void servo_reset_watchdog(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    servo_private_t* priv = (servo_private_t*)controller->private_data;
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (controller->state.status == ERROR_TIMEOUT) {
        controller->state.status = ERROR_OK;
    }
}

static error_code_t servo_self_test(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Simple self-test: try to set neutral position
    servo_private_t* priv = (servo_private_t*)controller->private_data;
    const servo_config_t* config = &priv->config;

    error_code_t err = hw_timer_set_pwm_us(priv->timer_id, priv->channel, config->pulse_neutral_us);
    return err;
}

static motor_state_t servo_get_state(const motor_controller_t* controller) {
    if (!controller) {
        motor_state_t empty = {0};
        empty.status = ERROR_INVALID_PARAMETER;
        return empty;
    }
    return controller->state;
}

static uint8_t servo_get_id(const motor_controller_t* controller) {
    if (!controller) {
        return 0;
    }
    return controller->id;
}

/* ============================================================================
 * Virtual Function Table
 * ============================================================================ */

static const motor_vtable_t servo_vtable = {
    .initialize = servo_initialize,
    .update = servo_update,
    .set_command = servo_set_command,
    .set_enabled = servo_set_enabled,
    .emergency_stop = servo_emergency_stop,
    .reset_watchdog = servo_reset_watchdog,
    .self_test = servo_self_test,
    .get_state = servo_get_state,
    .get_id = servo_get_id
};

const motor_vtable_t* servo_controller_get_vtable(void) {
    return &servo_vtable;
}

/* ============================================================================
 * Servo Controller Creation
 * ============================================================================ */

error_code_t servo_controller_create(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const servo_config_t* config,
    motor_controller_t* controller,
    servo_private_t* private_data)
{
    if (!config || !controller || !private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Initialize controller base
    memset(controller, 0, sizeof(motor_controller_t));
    controller->vtable = &servo_vtable;
    controller->id = id;
    controller->type = MOTOR_TYPE_SERVO;
    controller->private_data = private_data;

    // Initialize private data
    memset(private_data, 0, sizeof(servo_private_t));
    private_data->config = *config;
    private_data->timer_id = timer_id;
    private_data->channel = channel;

    return ERROR_OK;
}
