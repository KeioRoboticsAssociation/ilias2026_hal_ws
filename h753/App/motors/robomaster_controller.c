/**
 * @file robomaster_controller.c
 * @brief RoboMaster motor controller implementation (simplified for H753)
 */

#include "robomaster_controller.h"
#include <string.h>

/* ============================================================================
 * Motor Interface Implementation
 * ============================================================================ */

static error_code_t rm_initialize(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Initialize state
    controller->state.status = ERROR_NOT_INITIALIZED;
    controller->state.enabled = false;
    controller->state.last_update_time = hw_get_tick();

    // Initialize private data
    priv->last_watchdog_reset = hw_get_tick();
    priv->last_can_rx = hw_get_tick();
    priv->watchdog_expired = false;

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t rm_update(motor_controller_t* controller, float delta_time) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;
    uint32_t now = hw_get_tick();

    // Check watchdog
    uint32_t elapsed = now - priv->last_watchdog_reset;
    if (elapsed > priv->config.watchdog_timeout_ms) {
        if (!priv->watchdog_expired) {
            priv->watchdog_expired = true;
            controller->state.status = ERROR_TIMEOUT;
            controller->state.timeout_count++;
            // Send zero current command
            uint8_t data[8] = {0};
            hw_can_transmit(priv->config.can_id, data, 8);
        }
    }

    controller->state.last_update_time = now;
    return controller->state.status;
}

static error_code_t rm_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (!controller || !controller->private_data || !cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    if (cmd->motor_id != controller->id) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Reset watchdog
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (!cmd->enable) {
        return motor_set_enabled(controller, false);
    }

    if (!controller->state.enabled) {
        motor_set_enabled(controller, true);
    }

    // For simplicity, send raw current command
    // In a full implementation, you would use PID control here
    int16_t current = 0;

    switch (cmd->mode) {
        case CONTROL_MODE_CURRENT:
            current = (int16_t)cmd->target_value;
            break;

        case CONTROL_MODE_VELOCITY:
            // Simple proportional control (would normally use full PID)
            current = (int16_t)(cmd->target_value * priv->config.speed_kp * 1000.0f);
            break;

        default:
            return ERROR_INVALID_PARAMETER;
    }

    // Constrain current
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;

    // Send CAN command (DJI RoboMaster format)
    // Note: This is simplified - real implementation would group motors
    uint8_t data[8] = {0};
    data[0] = (current >> 8) & 0xFF;
    data[1] = current & 0xFF;

    error_code_t err = hw_can_transmit(priv->config.can_id, data, 8);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_COMM_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t rm_set_enabled(motor_controller_t* controller, bool enabled) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    if (!enabled) {
        // Send zero current
        uint8_t data[8] = {0};
        hw_can_transmit(priv->config.can_id, data, 8);
    }

    controller->state.enabled = enabled;
    return ERROR_OK;
}

static void rm_emergency_stop(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Send zero current immediately
    uint8_t data[8] = {0};
    hw_can_transmit(priv->config.can_id, data, 8);

    controller->state.enabled = false;
    controller->state.status = ERROR_SAFETY_VIOLATION;
}

static void rm_reset_watchdog(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (controller->state.status == ERROR_TIMEOUT) {
        controller->state.status = ERROR_OK;
    }
}

static error_code_t rm_self_test(motor_controller_t* controller) {
    // RoboMaster motors don't have a simple self-test
    // Would normally check CAN communication
    return ERROR_OK;
}

static motor_state_t rm_get_state(const motor_controller_t* controller) {
    if (!controller) {
        motor_state_t empty = {0};
        empty.status = ERROR_INVALID_PARAMETER;
        return empty;
    }
    return controller->state;
}

static uint8_t rm_get_id(const motor_controller_t* controller) {
    if (!controller) {
        return 0;
    }
    return controller->id;
}

/* ============================================================================
 * Virtual Function Table
 * ============================================================================ */

static const motor_vtable_t rm_vtable = {
    .initialize = rm_initialize,
    .update = rm_update,
    .set_command = rm_set_command,
    .set_enabled = rm_set_enabled,
    .emergency_stop = rm_emergency_stop,
    .reset_watchdog = rm_reset_watchdog,
    .self_test = rm_self_test,
    .get_state = rm_get_state,
    .get_id = rm_get_id
};

const motor_vtable_t* robomaster_controller_get_vtable(void) {
    return &rm_vtable;
}

/* ============================================================================
 * RoboMaster Controller Creation
 * ============================================================================ */

error_code_t robomaster_controller_create(
    uint8_t id,
    const robomaster_config_t* config,
    motor_controller_t* controller,
    robomaster_private_t* private_data)
{
    if (!config || !controller || !private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Initialize controller base
    memset(controller, 0, sizeof(motor_controller_t));
    controller->vtable = &rm_vtable;
    controller->id = id;
    controller->type = MOTOR_TYPE_ROBOMASTER;
    controller->private_data = private_data;

    // Initialize private data
    memset(private_data, 0, sizeof(robomaster_private_t));
    private_data->config = *config;

    return ERROR_OK;
}

/* ============================================================================
 * CAN Message Processing
 * ============================================================================ */

error_code_t robomaster_process_can_message(
    motor_controller_t* controller,
    uint32_t can_id,
    const uint8_t* data,
    uint8_t length)
{
    if (!controller || !controller->private_data || !data || length < 8) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Check if this message is for this motor
    if (can_id != (priv->config.can_id + 0x200)) {  // Standard RoboMaster CAN ID offset
        return ERROR_INVALID_PARAMETER;
    }

    // Parse DJI RoboMaster feedback message
    // Format: [angle_H, angle_L, speed_H, speed_L, current_H, current_L, temp, reserved]
    priv->encoder_angle = (int16_t)((data[0] << 8) | data[1]);
    priv->encoder_speed = (int16_t)((data[2] << 8) | data[3]);
    priv->motor_current = (int16_t)((data[4] << 8) | data[5]);

    // Update state
    controller->state.current_position = (float)priv->encoder_angle * 0.043633f;  // Convert to radians
    controller->state.current_velocity = (float)priv->encoder_speed * 0.104720f;  // Convert to rad/s
    controller->state.current_current = (float)priv->motor_current / 1000.0f;      // Convert to A
    controller->state.temperature = (float)data[6];

    priv->last_can_rx = hw_get_tick();

    return ERROR_OK;
}
