/**
 * @file robomaster_controller.c
 * @brief RoboMaster motor controller implementation (hybrid M3508/GM6020 support)
 */

#include "robomaster_controller.h"
#include <string.h>

/* ============================================================================
 * Motor Type Detection Helper
 * ============================================================================ */

/**
 * @brief Detect motor type based on CAN ID and configuration
 * @note GM6020: feedback IDs 0x205-0x20B, voltage control (-30000 to +30000)
 *       M3508/M2006: feedback IDs 0x201-0x208, current control (-16384 to +16384)
 */
static rm_motor_type_t detect_motor_type(uint16_t can_id, const robomaster_config_t* config) {
    // GM6020 motors typically use higher gains for voltage control
    // Use speed_kp > 10.0 as heuristic for GM6020 (voltage-based control needs higher gains)
    if (config->speed_kp > 10.0f) {
        return RM_TYPE_GM6020;
    }

    // Default to M3508/M2006 (current control)
    return RM_TYPE_M3508;
}

/**
 * @brief Get control CAN ID based on motor type and feedback ID
 */
static uint32_t get_control_can_id(uint16_t feedback_id, rm_motor_type_t motor_type) {
    if (motor_type == RM_TYPE_GM6020) {
        // GM6020: 0x1FF for motors 1-4 (0x205-0x208), 0x2FF for motors 5-7 (0x209-0x20B)
        if (feedback_id >= 0x209 && feedback_id <= 0x20B) {
            return 0x2FF;
        } else {
            return 0x1FF;
        }
    } else {
        // M3508/M2006: 0x200 for motors 1-4 (0x201-0x204), 0x1FF for motors 5-8 (0x205-0x208)
        if (feedback_id >= 0x201 && feedback_id <= 0x204) {
            return 0x200;
        } else {
            return 0x1FF;
        }
    }
}

/**
 * @brief Get motor index in CAN message (0-3)
 */
static uint8_t get_motor_index(uint16_t feedback_id, rm_motor_type_t motor_type) {
    if (motor_type == RM_TYPE_GM6020) {
        // GM6020: 0x205-0x208 → index 0-3, 0x209-0x20B → index 0-2
        if (feedback_id >= 0x209) {
            return (feedback_id - 0x209);
        } else {
            return (feedback_id - 0x205);
        }
    } else {
        // M3508/M2006: 0x201-0x204 → index 0-3, 0x205-0x208 → index 0-3
        if (feedback_id >= 0x205) {
            return (feedback_id - 0x205);
        } else {
            return (feedback_id - 0x201);
        }
    }
}

/* ============================================================================
 * Motor Interface Implementation
 * ============================================================================ */

static error_code_t rm_initialize(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Detect motor type based on CAN ID and configuration
    priv->motor_type = detect_motor_type(priv->config.can_id, &priv->config);

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

            // Send zero command (voltage or current depending on motor type)
            uint32_t control_id = get_control_can_id(priv->config.can_id, priv->motor_type);
            uint8_t motor_idx = get_motor_index(priv->config.can_id, priv->motor_type);
            uint8_t data[8] = {0};
            // Zero values in the correct position for this motor
            data[motor_idx * 2] = 0;
            data[motor_idx * 2 + 1] = 0;
            hw_can_transmit(control_id, data, 8);
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

    // Calculate control value based on motor type
    int16_t control_value = 0;

    if (priv->motor_type == RM_TYPE_GM6020) {
        // GM6020: Voltage control (-30000 to +30000)
        switch (cmd->mode) {
            case CONTROL_MODE_CURRENT:
                // Interpret as voltage directly
                control_value = (int16_t)cmd->target_value;
                break;

            case CONTROL_MODE_VELOCITY:
                // Simple proportional control (voltage output)
                control_value = (int16_t)(cmd->target_value * priv->config.speed_kp);
                break;

            case CONTROL_MODE_DUTY_CYCLE:
                // Map -1.0 to 1.0 → -30000 to +30000
                control_value = (int16_t)(cmd->target_value * 30000.0f);
                break;

            default:
                return ERROR_INVALID_PARAMETER;
        }

        // Constrain voltage for GM6020
        if (control_value > 30000) control_value = 30000;
        if (control_value < -30000) control_value = -30000;

    } else {
        // M3508/M2006: Current control (-16384 to +16384)
        switch (cmd->mode) {
            case CONTROL_MODE_CURRENT:
                control_value = (int16_t)cmd->target_value;
                break;

            case CONTROL_MODE_VELOCITY:
                // Simple proportional control (current output)
                control_value = (int16_t)(cmd->target_value * priv->config.speed_kp * 1000.0f);
                break;

            case CONTROL_MODE_DUTY_CYCLE:
                // Map -1.0 to 1.0 → -10000 to +10000 (safe current limit)
                control_value = (int16_t)(cmd->target_value * 10000.0f);
                break;

            default:
                return ERROR_INVALID_PARAMETER;
        }

        // Constrain current for M3508/M2006
        if (control_value > 16384) control_value = 16384;
        if (control_value < -16384) control_value = -16384;
    }

    // Get control CAN ID and motor index
    uint32_t control_id = get_control_can_id(priv->config.can_id, priv->motor_type);
    uint8_t motor_idx = get_motor_index(priv->config.can_id, priv->motor_type);

    // Build CAN message (8 bytes for up to 4 motors)
    // Format: [motor1_H, motor1_L, motor2_H, motor2_L, motor3_H, motor3_L, motor4_H, motor4_L]
    uint8_t data[8] = {0};
    data[motor_idx * 2] = (control_value >> 8) & 0xFF;      // High byte
    data[motor_idx * 2 + 1] = control_value & 0xFF;         // Low byte

    error_code_t err = hw_can_transmit(control_id, data, 8);
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
        // Send zero command (voltage or current)
        uint32_t control_id = get_control_can_id(priv->config.can_id, priv->motor_type);
        uint8_t motor_idx = get_motor_index(priv->config.can_id, priv->motor_type);
        uint8_t data[8] = {0};
        data[motor_idx * 2] = 0;
        data[motor_idx * 2 + 1] = 0;
        hw_can_transmit(control_id, data, 8);
    }

    controller->state.enabled = enabled;
    return ERROR_OK;
}

static void rm_emergency_stop(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;

    // Send zero command immediately
    uint32_t control_id = get_control_can_id(priv->config.can_id, priv->motor_type);
    uint8_t motor_idx = get_motor_index(priv->config.can_id, priv->motor_type);
    uint8_t data[8] = {0};
    data[motor_idx * 2] = 0;
    data[motor_idx * 2 + 1] = 0;
    hw_can_transmit(control_id, data, 8);

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

    // Check if this message is for this motor (feedback ID should match configured CAN ID)
    if (can_id != priv->config.can_id) {
        return ERROR_INVALID_PARAMETER;
    }

    // Parse DJI RoboMaster feedback message
    // Format: [angle_H, angle_L, speed_H, speed_L, current_H, current_L, temp, reserved]
    priv->encoder_angle = (int16_t)((data[0] << 8) | data[1]);
    priv->encoder_speed = (int16_t)((data[2] << 8) | data[3]);
    priv->motor_current = (int16_t)((data[4] << 8) | data[5]);

    // Update state with appropriate conversion factors
    if (priv->motor_type == RM_TYPE_GM6020) {
        // GM6020: 8192 counts/revolution, speed in RPM
        controller->state.current_position = (float)priv->encoder_angle * (2.0f * 3.14159f / 8192.0f);  // radians
        controller->state.current_velocity = (float)priv->encoder_speed * (2.0f * 3.14159f / 60.0f);    // rad/s
        controller->state.current_current = (float)priv->motor_current / 1000.0f;  // mA to A
    } else {
        // M3508/M2006: 8192 counts/revolution, speed in RPM
        controller->state.current_position = (float)priv->encoder_angle * (2.0f * 3.14159f / 8192.0f);  // radians
        controller->state.current_velocity = (float)priv->encoder_speed * (2.0f * 3.14159f / 60.0f);    // rad/s
        controller->state.current_current = (float)priv->motor_current / 1000.0f;  // mA to A
    }
    controller->state.temperature = (float)data[6];

    priv->last_can_rx = hw_get_tick();

    return ERROR_OK;
}
