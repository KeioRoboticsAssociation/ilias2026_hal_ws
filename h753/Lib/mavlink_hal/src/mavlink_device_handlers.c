/**
 * @file mavlink_device_handlers.c
 * @brief MAVLink message handlers for unified device interface
 *
 * Provides automatic MAVLink message routing to devices based on ID.
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "mavlink_device_interface.h"
#include <string.h>

/* Include MAVLink library - adjust path as needed */
/* #include "mavlink/c_library_v2/common/mavlink.h" */

/* Forward declarations for registry functions */
extern mavlink_device_t* mavlink_device_registry_find(uint8_t device_id);
extern uint32_t mavlink_device_registry_get_all(mavlink_device_t** devices, uint32_t max_devices);
extern uint32_t mavlink_device_registry_count(void);

/* ========================================================================== */
/*  MESSAGE HANDLER: RC_CHANNELS_OVERRIDE (ID 70)                             */
/* ========================================================================== */

/**
 * @brief Handle RC_CHANNELS_OVERRIDE message
 *
 * Maps RC channels 1-8 to device IDs 1-8 (typically servos).
 * PWM values 1000-2000 are converted to appropriate device commands.
 *
 * @param channels RC channel values (8 channels)
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_handle_rc_channels(const uint16_t channels[8])
{
    if (!channels) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    mavlink_device_error_t last_error = MAVLINK_DEVICE_ERROR_NONE;

    /* RC channels 1-8 map to device IDs 1-8 */
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t pwm = channels[i];

        /* Check if channel is active (PWM 1000-2000) */
        if (pwm < 1000 || pwm > 2000) {
            continue;
        }

        /* Find device by ID (ID = channel + 1) */
        uint8_t device_id = i + 1;
        mavlink_device_t* device = mavlink_device_registry_find(device_id);

        if (!device || !device->status.enabled) {
            continue;
        }

        /* Create command based on device type */
        mavlink_device_command_t command = {0};
        command.type = device->id.type;

        if (device->id.type == MAVLINK_DEVICE_TYPE_SERVO) {
            /* Servo: PWM to angle conversion */
            command.mode = MAVLINK_CONTROL_MODE_POSITION;

            float min_angle = device->config.limits.min_position;
            float max_angle = device->config.limits.max_position;

            /* Map PWM 1000-2000 to min-max angle */
            command.data.position.target = min_angle +
                ((float)(pwm - 1000) / 1000.0f) * (max_angle - min_angle);

        } else if (mavlink_device_is_motor(device->id.type)) {
            /* Motor: PWM to duty cycle conversion */
            command.mode = MAVLINK_CONTROL_MODE_DUTY_CYCLE;

            /* Map PWM 1000-2000 to duty -1.0 to +1.0 */
            command.data.duty_cycle.duty = ((float)(pwm - 1500) / 500.0f);
        }

        /* Send command to device */
        mavlink_device_error_t err = mavlink_device_send_command(device, &command);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            last_error = err;
        }
    }

    return last_error;
}

/* ========================================================================== */
/*  MESSAGE HANDLER: MANUAL_CONTROL (ID 69)                                   */
/* ========================================================================== */

/**
 * @brief Handle MANUAL_CONTROL message
 *
 * Joystick control with axes (x, y, z, r) mapped to devices.
 * Implementation can be customized based on robot type.
 *
 * @param x Forward/backward axis (-1000 to +1000)
 * @param y Left/right axis (-1000 to +1000)
 * @param z Throttle axis (-1000 to +1000)
 * @param r Rotation axis (-1000 to +1000)
 * @param buttons Button bitfield
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_handle_manual_control(
    int16_t x,
    int16_t y,
    int16_t z,
    int16_t r,
    uint16_t buttons)
{
    /* Normalize axes to -1.0 to +1.0 */
    float x_norm = x / 1000.0f;
    float y_norm = y / 1000.0f;
    float z_norm = z / 1000.0f;
    float r_norm = r / 1000.0f;

    /* Example: Differential drive for motors ID 10-11 */
    mavlink_device_t* left_motor = mavlink_device_registry_find(10);
    mavlink_device_t* right_motor = mavlink_device_registry_find(11);

    if (left_motor && right_motor) {
        /* Arcade drive: z = throttle, r = rotation */
        float left_speed = z_norm + r_norm;
        float right_speed = z_norm - r_norm;

        /* Clamp to -1.0 to +1.0 */
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (left_speed < -1.0f) left_speed = -1.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;
        if (right_speed < -1.0f) right_speed = -1.0f;

        /* Send velocity commands */
        mavlink_device_command_t left_cmd = {0};
        left_cmd.type = left_motor->id.type;
        left_cmd.mode = MAVLINK_CONTROL_MODE_VELOCITY;
        left_cmd.data.velocity.target = left_speed * left_motor->config.limits.max_velocity;

        mavlink_device_command_t right_cmd = {0};
        right_cmd.type = right_motor->id.type;
        right_cmd.mode = MAVLINK_CONTROL_MODE_VELOCITY;
        right_cmd.data.velocity.target = right_speed * right_motor->config.limits.max_velocity;

        mavlink_device_send_command(left_motor, &left_cmd);
        mavlink_device_send_command(right_motor, &right_cmd);
    }

    /* Handle buttons (emergency stop, mode changes, etc.) */
    if (buttons & 0x01) {  /* Button 0: Emergency stop */
        extern mavlink_device_error_t mavlink_device_registry_emergency_stop(void);
        mavlink_device_registry_emergency_stop();
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  MESSAGE HANDLER: MOTOR_COMMAND (Custom ID 12004)                          */
/* ========================================================================== */

/**
 * @brief Handle generic MOTOR_COMMAND message
 *
 * Universal motor control command for any device type.
 * Supports position, velocity, current, and duty cycle modes.
 *
 * @param motor_id Target device ID (1-255)
 * @param control_mode Control mode (0=position, 1=velocity, 2=current, 3=duty)
 * @param target_value Target value (units depend on mode)
 * @param enable Enable flag
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_handle_motor_command(
    uint8_t motor_id,
    uint8_t control_mode,
    float target_value,
    uint8_t enable)
{
    /* Find device by ID */
    mavlink_device_t* device = mavlink_device_registry_find(motor_id);

    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Handle enable/disable */
    if (!enable) {
        return mavlink_device_enable(device, false);
    }

    /* Ensure device is enabled */
    if (!device->status.enabled) {
        mavlink_device_enable(device, true);
    }

    /* Create command */
    mavlink_device_command_t command = {0};
    command.type = device->id.type;
    command.mode = (mavlink_control_mode_t)control_mode;

    switch (command.mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            command.data.position.target = target_value;
            break;

        case MAVLINK_CONTROL_MODE_VELOCITY:
            command.data.velocity.target = target_value;
            break;

        case MAVLINK_CONTROL_MODE_CURRENT:
        case MAVLINK_CONTROL_MODE_TORQUE:
            command.data.current.target = target_value;
            break;

        case MAVLINK_CONTROL_MODE_DUTY_CYCLE:
            command.data.duty_cycle.duty = target_value;
            break;

        default:
            return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Send command to device */
    return mavlink_device_send_command(device, &command);
}

/* ========================================================================== */
/*  MESSAGE HANDLER: PARAM_REQUEST_LIST (ID 21)                               */
/* ========================================================================== */

/**
 * @brief Handle PARAM_REQUEST_LIST message
 *
 * Send all device parameters via MAVLink.
 * This is a callback - actual MAVLink transmission must be implemented.
 *
 * @param send_param_fn Callback to send parameter value
 * @param context User context for callback
 * @return Number of parameters sent
 */
uint32_t mavlink_device_handle_param_request_list(
    void (*send_param_fn)(const char* name, float value, uint16_t index, uint16_t total, void* ctx),
    void* context)
{
    if (!send_param_fn) {
        return 0;
    }

    /* Get all devices */
    mavlink_device_t* devices[64];
    uint32_t device_count = mavlink_device_registry_get_all(devices, 64);

    uint32_t param_count = 0;
    uint32_t param_index = 0;

    /* Count total parameters first */
    for (uint32_t i = 0; i < device_count; i++) {
        if (devices[i] && mavlink_device_has_capability(
            devices[i]->id.capabilities, MAVLINK_DEVICE_CAP_PARAMETERS)) {
            /* Each motor has multiple PID parameters */
            if (mavlink_device_is_motor(devices[i]->id.type)) {
                param_count += 9;  /* 3 PID controllers × 3 gains */
            }
        }
    }

    /* Send parameters */
    for (uint32_t i = 0; i < device_count; i++) {
        if (!devices[i]) continue;

        uint8_t id = devices[i]->id.id;

        /* Example: RoboMaster PID parameters */
        if (devices[i]->id.type == MAVLINK_DEVICE_TYPE_ROBOMASTER) {
            char param_name[16];
            float value;

            /* Speed PID */
            snprintf(param_name, sizeof(param_name), "RM_%d_SPD_KP", id);
            if (mavlink_device_get_param(devices[i], "speed_kp", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            snprintf(param_name, sizeof(param_name), "RM_%d_SPD_KI", id);
            if (mavlink_device_get_param(devices[i], "speed_ki", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            snprintf(param_name, sizeof(param_name), "RM_%d_SPD_KD", id);
            if (mavlink_device_get_param(devices[i], "speed_kd", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            /* Angle PID */
            snprintf(param_name, sizeof(param_name), "RM_%d_ANG_KP", id);
            if (mavlink_device_get_param(devices[i], "angle_kp", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            snprintf(param_name, sizeof(param_name), "RM_%d_ANG_KI", id);
            if (mavlink_device_get_param(devices[i], "angle_ki", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            snprintf(param_name, sizeof(param_name), "RM_%d_ANG_KD", id);
            if (mavlink_device_get_param(devices[i], "angle_kd", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }
        }

        /* Similar handling for DC motors */
        else if (devices[i]->id.type == MAVLINK_DEVICE_TYPE_DC_MOTOR) {
            char param_name[16];
            float value;

            snprintf(param_name, sizeof(param_name), "DC_%d_SPD_KP", id);
            if (mavlink_device_get_param(devices[i], "speed_kp", &value) == MAVLINK_DEVICE_ERROR_NONE) {
                send_param_fn(param_name, value, param_index++, param_count, context);
            }

            /* ... additional DC motor parameters */
        }
    }

    return param_index;
}

/* ========================================================================== */
/*  MESSAGE HANDLER: PARAM_SET (ID 23)                                        */
/* ========================================================================== */

/**
 * @brief Handle PARAM_SET message
 *
 * Set device parameter by name.
 *
 * @param param_name Parameter name (e.g., "RM_20_SPD_KP")
 * @param value Parameter value
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_handle_param_set(
    const char* param_name,
    float value)
{
    if (!param_name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Parse parameter name to extract device ID and parameter */
    /* Format: "TYPE_ID_PARAM" e.g., "RM_20_SPD_KP" */

    char type_prefix[8] = {0};
    uint8_t device_id = 0;
    char param_suffix[16] = {0};

    /* Simple parsing (can be improved) */
    if (sscanf(param_name, "%[^_]_%hhu_%s", type_prefix, &device_id, param_suffix) != 3) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Find device */
    mavlink_device_t* device = mavlink_device_registry_find(device_id);
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Convert parameter suffix to internal name */
    char internal_param[32];
    if (strcmp(param_suffix, "SPD_KP") == 0) {
        strcpy(internal_param, "speed_kp");
    } else if (strcmp(param_suffix, "SPD_KI") == 0) {
        strcpy(internal_param, "speed_ki");
    } else if (strcmp(param_suffix, "SPD_KD") == 0) {
        strcpy(internal_param, "speed_kd");
    } else if (strcmp(param_suffix, "ANG_KP") == 0) {
        strcpy(internal_param, "angle_kp");
    } else if (strcmp(param_suffix, "ANG_KI") == 0) {
        strcpy(internal_param, "angle_ki");
    } else if (strcmp(param_suffix, "ANG_KD") == 0) {
        strcpy(internal_param, "angle_kd");
    } else if (strcmp(param_suffix, "POS_KP") == 0) {
        strcpy(internal_param, "position_kp");
    } else if (strcmp(param_suffix, "POS_KI") == 0) {
        strcpy(internal_param, "position_ki");
    } else if (strcmp(param_suffix, "POS_KD") == 0) {
        strcpy(internal_param, "position_kd");
    } else {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Set parameter on device */
    return mavlink_device_set_param(device, internal_param, value);
}

/* ========================================================================== */
/*  TELEMETRY GENERATION                                                      */
/* ========================================================================== */

/**
 * @brief Generate telemetry messages for all devices
 *
 * Call this periodically to send device telemetry via MAVLink.
 * Uses callback function to allow flexible MAVLink transmission.
 *
 * @param current_time_ms Current system time
 * @param send_telemetry_fn Callback to send telemetry
 * @param context User context for callback
 * @return Number of telemetry messages generated
 */
uint32_t mavlink_device_generate_telemetry(
    uint32_t current_time_ms,
    void (*send_telemetry_fn)(const mavlink_device_telemetry_t* telemetry, void* ctx),
    void* context)
{
    if (!send_telemetry_fn) {
        return 0;
    }

    /* Update telemetry for devices that need it */
    extern mavlink_device_error_t mavlink_device_registry_update_telemetry(uint32_t current_time_ms);
    mavlink_device_registry_update_telemetry(current_time_ms);

    /* Get all devices */
    mavlink_device_t* devices[64];
    uint32_t device_count = mavlink_device_registry_get_all(devices, 64);

    uint32_t telemetry_count = 0;

    for (uint32_t i = 0; i < device_count; i++) {
        if (devices[i] && mavlink_device_has_capability(
            devices[i]->id.capabilities, MAVLINK_DEVICE_CAP_TELEMETRY)) {

            send_telemetry_fn(&devices[i]->telemetry, context);
            telemetry_count++;
        }
    }

    return telemetry_count;
}

/* ========================================================================== */
/*  HEARTBEAT GENERATION                                                      */
/* ========================================================================== */

/**
 * @brief Get device registry status for heartbeat
 *
 * @param total_devices Output: total devices
 * @param active_devices Output: active (enabled) devices
 * @param error_devices Output: devices in error state
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_get_heartbeat_status(
    uint32_t* total_devices,
    uint32_t* active_devices,
    uint32_t* error_devices)
{
    extern mavlink_device_error_t mavlink_device_registry_get_stats(
        uint32_t* total, uint32_t* enabled, uint32_t* errors);

    return mavlink_device_registry_get_stats(total_devices, active_devices, error_devices);
}
