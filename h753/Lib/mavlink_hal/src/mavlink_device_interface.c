/**
 * @file mavlink_device_interface.c
 * @brief Implementation of unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "mavlink_device_interface.h"
#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*  UTILITY FUNCTION IMPLEMENTATIONS                                          */
/* ========================================================================== */

const char* mavlink_device_type_name(mavlink_device_type_t type) {
    switch (type) {
        case MAVLINK_DEVICE_TYPE_NONE: return "None";
        case MAVLINK_DEVICE_TYPE_SERVO: return "Servo";
        case MAVLINK_DEVICE_TYPE_DC_MOTOR: return "DC Motor";
        case MAVLINK_DEVICE_TYPE_BLDC_MOTOR: return "BLDC Motor";
        case MAVLINK_DEVICE_TYPE_STEPPER: return "Stepper";
        case MAVLINK_DEVICE_TYPE_ROBOMASTER: return "RoboMaster";
        case MAVLINK_DEVICE_TYPE_RS485_MOTOR: return "RS485 Motor";
        case MAVLINK_DEVICE_TYPE_ENCODER: return "Encoder";
        case MAVLINK_DEVICE_TYPE_IMU: return "IMU";
        case MAVLINK_DEVICE_TYPE_GPS: return "GPS";
        case MAVLINK_DEVICE_TYPE_ANALOG_SENSOR: return "Analog Sensor";
        case MAVLINK_DEVICE_TYPE_DIGITAL_IO: return "Digital I/O";
        case MAVLINK_DEVICE_TYPE_CUSTOM: return "Custom";
        default: return "Unknown";
    }
}

const char* mavlink_device_state_name(mavlink_device_state_t state) {
    switch (state) {
        case MAVLINK_DEVICE_STATE_UNINITIALIZED: return "Uninitialized";
        case MAVLINK_DEVICE_STATE_INITIALIZING: return "Initializing";
        case MAVLINK_DEVICE_STATE_READY: return "Ready";
        case MAVLINK_DEVICE_STATE_ACTIVE: return "Active";
        case MAVLINK_DEVICE_STATE_ERROR: return "Error";
        case MAVLINK_DEVICE_STATE_DISABLED: return "Disabled";
        case MAVLINK_DEVICE_STATE_CALIBRATING: return "Calibrating";
        default: return "Unknown";
    }
}

const char* mavlink_device_control_mode_name(mavlink_control_mode_t mode) {
    switch (mode) {
        case MAVLINK_CONTROL_MODE_POSITION: return "Position";
        case MAVLINK_CONTROL_MODE_VELOCITY: return "Velocity";
        case MAVLINK_CONTROL_MODE_CURRENT: return "Current";
        case MAVLINK_CONTROL_MODE_DUTY_CYCLE: return "Duty Cycle";
        case MAVLINK_CONTROL_MODE_TORQUE: return "Torque";
        case MAVLINK_CONTROL_MODE_DISABLED: return "Disabled";
        default: return "Unknown";
    }
}

/* ========================================================================== */
/*  DEVICE INTERFACE IMPLEMENTATIONS                                          */
/* ========================================================================== */

mavlink_device_error_t mavlink_device_init(
    mavlink_device_t* device,
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    const mavlink_device_vtable_t* vtable,
    void* private_data)
{
    if (!device || !config || !vtable) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Initialize device identification */
    device->id.id = id;
    device->id.type = type;
    strncpy(device->id.name, name ? name : "Unknown", sizeof(device->id.name) - 1);
    device->id.name[sizeof(device->id.name) - 1] = '\0';
    device->id.capabilities = 0;  /* Will be set by device-specific init */
    device->id.hardware_version = 1;
    device->id.firmware_version = 1;

    /* Initialize status */
    device->status.state = MAVLINK_DEVICE_STATE_INITIALIZING;
    device->status.error = MAVLINK_DEVICE_ERROR_NONE;
    device->status.control_mode = MAVLINK_CONTROL_MODE_DISABLED;
    device->status.uptime_ms = 0;
    device->status.command_count = 0;
    device->status.error_count = 0;
    device->status.enabled = false;

    /* Copy configuration */
    memcpy(&device->config, config, sizeof(mavlink_device_config_t));

    /* Set vtable and private data */
    device->vtable = vtable;
    device->private_data = private_data;

    /* Initialize watchdog */
    device->last_command_time_ms = 0;

    /* Initialize telemetry */
    memset(&device->telemetry, 0, sizeof(mavlink_device_telemetry_t));
    device->telemetry_rate_hz = 1;  /* Default 1Hz */
    device->last_telemetry_time_ms = 0;

    /* Call device-specific initialization */
    if (vtable->init) {
        mavlink_device_error_t err = vtable->init(device, config);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            device->status.state = MAVLINK_DEVICE_STATE_ERROR;
            device->status.error = err;
            return err;
        }
    }

    /* Device initialized successfully */
    device->status.state = MAVLINK_DEVICE_STATE_READY;
    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_update(
    mavlink_device_t* device,
    uint32_t dt_ms)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Update uptime */
    device->status.uptime_ms += dt_ms;

    /* Check watchdog timeout */
    if (device->status.enabled && mavlink_device_watchdog_timeout(device, device->status.uptime_ms)) {
        mavlink_device_handle_timeout(device);
    }

    /* Call device-specific update */
    if (device->vtable && device->vtable->update) {
        mavlink_device_error_t err = device->vtable->update(device, dt_ms);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            device->status.error = err;
            device->status.error_count++;
            return err;
        }
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_shutdown(mavlink_device_t* device)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific shutdown */
    if (device->vtable && device->vtable->shutdown) {
        mavlink_device_error_t err = device->vtable->shutdown(device);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            return err;
        }
    }

    device->status.state = MAVLINK_DEVICE_STATE_UNINITIALIZED;
    device->status.enabled = false;

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_enable(
    mavlink_device_t* device,
    bool enable)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific enable */
    if (device->vtable && device->vtable->enable) {
        mavlink_device_error_t err = device->vtable->enable(device, enable);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            return err;
        }
    }

    device->status.enabled = enable;
    device->status.state = enable ? MAVLINK_DEVICE_STATE_ACTIVE : MAVLINK_DEVICE_STATE_READY;

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_send_command(
    mavlink_device_t* device,
    const mavlink_device_command_t* command)
{
    if (!device || !command) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    if (!device->status.enabled) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    /* Update watchdog */
    device->last_command_time_ms = device->status.uptime_ms;

    /* Call device-specific command handler */
    if (device->vtable && device->vtable->command) {
        mavlink_device_error_t err = device->vtable->command(device, command);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            device->status.error = err;
            device->status.error_count++;
            return err;
        }
    }

    device->status.command_count++;
    device->status.control_mode = command->mode;

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_get_feedback(
    const mavlink_device_t* device,
    mavlink_device_feedback_t* feedback)
{
    if (!device || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific feedback getter */
    if (device->vtable && device->vtable->get_feedback) {
        return device->vtable->get_feedback(device, feedback);
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

mavlink_device_error_t mavlink_device_get_status(
    const mavlink_device_t* device,
    mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific status getter */
    if (device->vtable && device->vtable->get_status) {
        return device->vtable->get_status(device, status);
    }

    /* Return cached status */
    memcpy(status, &device->status, sizeof(mavlink_device_status_t));
    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t mavlink_device_set_param(
    mavlink_device_t* device,
    const char* param_name,
    float value)
{
    if (!device || !param_name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific parameter setter */
    if (device->vtable && device->vtable->set_param) {
        return device->vtable->set_param(device, param_name, value);
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

mavlink_device_error_t mavlink_device_get_param(
    const mavlink_device_t* device,
    const char* param_name,
    float* value)
{
    if (!device || !param_name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific parameter getter */
    if (device->vtable && device->vtable->get_param) {
        return device->vtable->get_param(device, param_name, value);
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

mavlink_device_error_t mavlink_device_self_test(
    mavlink_device_t* device,
    mavlink_device_diagnostic_t* diagnostic)
{
    if (!device || !diagnostic) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Call device-specific self-test */
    if (device->vtable && device->vtable->self_test) {
        return device->vtable->self_test(device, diagnostic);
    }

    /* Default: no self-test */
    diagnostic->result = MAVLINK_DIAG_RESULT_SKIPPED;
    strncpy(diagnostic->message, "Self-test not implemented", sizeof(diagnostic->message) - 1);
    diagnostic->execution_time_us = 0;

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

mavlink_device_error_t mavlink_device_calibrate(mavlink_device_t* device)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    device->status.state = MAVLINK_DEVICE_STATE_CALIBRATING;

    /* Call device-specific calibration */
    if (device->vtable && device->vtable->calibrate) {
        mavlink_device_error_t err = device->vtable->calibrate(device);
        if (err != MAVLINK_DEVICE_ERROR_NONE) {
            device->status.state = MAVLINK_DEVICE_STATE_ERROR;
            device->status.error = err;
            return err;
        }
    }

    device->status.state = MAVLINK_DEVICE_STATE_READY;
    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  TELEMETRY AND WATCHDOG                                                    */
/* ========================================================================== */

bool mavlink_device_needs_telemetry(
    const mavlink_device_t* device,
    uint32_t current_time_ms)
{
    if (!device || device->telemetry_rate_hz == 0) {
        return false;
    }

    uint32_t period_ms = 1000 / device->telemetry_rate_hz;
    return (current_time_ms - device->last_telemetry_time_ms) >= period_ms;
}

mavlink_device_error_t mavlink_device_update_telemetry(
    mavlink_device_t* device,
    uint32_t current_time_ms)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Update timestamp */
    device->telemetry.timestamp_ms = current_time_ms;
    device->last_telemetry_time_ms = current_time_ms;

    /* Get current feedback */
    mavlink_device_get_feedback(device, &device->telemetry.feedback);

    /* Get current status */
    mavlink_device_get_status(device, &device->telemetry.status);

    /* Calculate health metric (simple: based on error count) */
    if (device->status.command_count > 0) {
        device->telemetry.health = 1.0f -
            ((float)device->status.error_count / (float)device->status.command_count);
    } else {
        device->telemetry.health = 1.0f;
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

bool mavlink_device_watchdog_timeout(
    const mavlink_device_t* device,
    uint32_t current_time_ms)
{
    if (!device || device->config.failsafe.timeout_ms == 0) {
        return false;
    }

    if (device->last_command_time_ms == 0) {
        return false;  /* No commands received yet */
    }

    return (current_time_ms - device->last_command_time_ms) > device->config.failsafe.timeout_ms;
}

mavlink_device_error_t mavlink_device_handle_timeout(mavlink_device_t* device)
{
    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    mavlink_device_error_t err = MAVLINK_DEVICE_ERROR_NONE;

    switch (device->config.failsafe.action) {
        case MAVLINK_FAILSAFE_HOLD:
            /* Hold current position - do nothing */
            break;

        case MAVLINK_FAILSAFE_NEUTRAL:
            /* Move to neutral position */
            {
                mavlink_device_command_t neutral_cmd = {0};
                neutral_cmd.type = device->id.type;
                neutral_cmd.mode = MAVLINK_CONTROL_MODE_POSITION;

                if (mavlink_device_is_motor(device->id.type)) {
                    /* Servo neutral angle or motor zero position */
                    if (device->id.type == MAVLINK_DEVICE_TYPE_SERVO) {
                        neutral_cmd.data.position.target = device->config.config.servo.neutral_angle;
                    } else {
                        neutral_cmd.data.position.target = 0.0f;
                    }
                    err = mavlink_device_send_command(device, &neutral_cmd);
                }
            }
            break;

        case MAVLINK_FAILSAFE_DISABLE:
            /* Disable device */
            err = mavlink_device_enable(device, false);
            break;

        case MAVLINK_FAILSAFE_CUSTOM:
            /* Custom failsafe value */
            {
                mavlink_device_command_t custom_cmd = {0};
                custom_cmd.type = device->id.type;
                custom_cmd.mode = MAVLINK_CONTROL_MODE_DUTY_CYCLE;
                custom_cmd.data.duty_cycle.duty = device->config.failsafe.custom_value;
                err = mavlink_device_send_command(device, &custom_cmd);
            }
            break;
    }

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        device->status.error = MAVLINK_DEVICE_ERROR_TIMEOUT;
        device->status.error_count++;
    }

    return err;
}

/* ========================================================================== */
/*  DEVICE FACTORY (FORWARD DECLARATIONS)                                     */
/* ========================================================================== */

/* These are implemented in device-specific files */
extern mavlink_device_t* mavlink_device_create_servo_impl(
    uint8_t id, const char* name, const mavlink_device_config_t* config, void* private_data);
extern mavlink_device_t* mavlink_device_create_motor_impl(
    mavlink_device_type_t type, uint8_t id, const char* name,
    const mavlink_device_config_t* config, void* private_data);
extern mavlink_device_t* mavlink_device_create_sensor_impl(
    mavlink_device_type_t type, uint8_t id, const char* name,
    const mavlink_device_config_t* config, void* private_data);

mavlink_device_t* mavlink_device_create_servo(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    return mavlink_device_create_servo_impl(id, name, config, private_data);
}

mavlink_device_t* mavlink_device_create_motor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    return mavlink_device_create_motor_impl(type, id, name, config, private_data);
}

mavlink_device_t* mavlink_device_create_sensor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    return mavlink_device_create_sensor_impl(type, id, name, config, private_data);
}

void mavlink_device_destroy(mavlink_device_t* device)
{
    if (!device) {
        return;
    }

    /* Shutdown device */
    mavlink_device_shutdown(device);

    /* Free private data if allocated */
    if (device->private_data) {
        free(device->private_data);
        device->private_data = NULL;
    }

    /* Free device itself */
    free(device);
}
