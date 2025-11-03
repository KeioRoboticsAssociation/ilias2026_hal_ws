/**
 * @file rs485_controller.c
 * @brief RS485 Ikeya MD motor controller implementation
 */

#include "rs485_controller.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * CRC-16 Calculation (Modbus Method)
 * ============================================================================ */

uint16_t rs485_calculate_crc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/* ============================================================================
 * Protocol Helper Functions
 * ============================================================================ */

static error_code_t rs485_send_read_request(uint8_t uart_id, uint8_t motor_id) {
    rs485_read_request_t packet;
    packet.header = RS485_HEADER_READ_REQUEST;
    packet.id = motor_id;
    packet.crc = rs485_calculate_crc16((uint8_t*)&packet, sizeof(packet) - 2);

    return hw_uart_transmit(uart_id, (uint8_t*)&packet, sizeof(packet));
}

static error_code_t rs485_receive_response(uint8_t uart_id, uint8_t* buffer, uint16_t size, uint32_t timeout_ms) {
    return hw_uart_receive(uart_id, buffer, size, timeout_ms);
}

static bool rs485_verify_crc(const uint8_t* packet, uint16_t packet_size) {
    if (packet_size < 4) return false;

    // Calculate CRC over all bytes except the last 2 (CRC field itself)
    uint16_t calculated_crc = rs485_calculate_crc16(packet, packet_size - 2);

    // Extract received CRC (little-endian)
    uint16_t received_crc = packet[packet_size - 2] | (packet[packet_size - 1] << 8);

    return (calculated_crc == received_crc);
}

/* ============================================================================
 * RS485 Protocol Functions
 * ============================================================================ */

error_code_t rs485_read_status(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    // Send READ request
    error_code_t err = rs485_send_read_request(priv->config.uart_id, priv->config.id);
    if (err != ERROR_OK) {
        priv->consecutive_errors++;
        return err;
    }

    // Receive response (8 bytes for both velocity and position)
    uint8_t response_buffer[8];
    err = rs485_receive_response(priv->config.uart_id, response_buffer, 8, RS485_TIMEOUT_MS);
    if (err != ERROR_OK) {
        priv->timeout_count++;
        priv->consecutive_errors++;
        return ERROR_TIMEOUT;
    }

    // Verify CRC
    if (!rs485_verify_crc(response_buffer, 8)) {
        priv->crc_error_count++;
        priv->consecutive_errors++;
        return ERROR_COMM_ERROR;
    }

    // Parse response based on header
    uint8_t header = response_buffer[0];
    uint8_t id = response_buffer[1];

    // Verify ID matches
    if (id != priv->config.id) {
        return ERROR_COMM_ERROR;
    }

    if (header == RS485_HEADER_VEL_RESPONSE) {
        // Velocity control mode response
        rs485_velocity_response_t* vel_resp = (rs485_velocity_response_t*)response_buffer;
        priv->current_rps = vel_resp->rps;
        priv->detected_mode = RS485_MODE_VELOCITY;

        // Update motor state
        controller->state.current_velocity = vel_resp->rps * 2.0f * 3.14159265f;  // RPS to rad/s

    } else if (header == RS485_HEADER_POS_RESPONSE) {
        // Position control mode response
        rs485_position_response_t* pos_resp = (rs485_position_response_t*)response_buffer;
        priv->current_count = pos_resp->count;
        priv->current_rotation = pos_resp->rotation;
        priv->detected_mode = RS485_MODE_POSITION;

        // Update motor state
        // Total position = (rotation * 8192) + count
        // Encoder resolution is 2048 x 4 = 8192 counts/revolution
        float total_counts = (float)pos_resp->rotation * 8192.0f + (float)pos_resp->count;
        controller->state.current_position = total_counts * (2.0f * 3.14159265f / 8192.0f);  // counts to radians

    } else {
        return ERROR_COMM_ERROR;
    }

    priv->last_response_time = hw_get_tick();
    priv->consecutive_errors = 0;
    return ERROR_OK;
}

error_code_t rs485_write_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (!controller || !controller->private_data || !cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    error_code_t err;

    if (priv->detected_mode == RS485_MODE_VELOCITY) {
        // Velocity control mode - send RPS command
        rs485_velocity_write_t packet;
        packet.header = RS485_HEADER_WRITE;
        packet.id = priv->config.id;

        // Convert command to RPS based on mode
        switch (cmd->mode) {
            case CONTROL_MODE_VELOCITY:
                // rad/s to RPS
                packet.target_rps = cmd->target_value / (2.0f * 3.14159265f);
                break;

            case CONTROL_MODE_DUTY_CYCLE:
                // Map -1.0 to 1.0 → max RPS range
                packet.target_rps = cmd->target_value * priv->config.max_rps;
                break;

            case CONTROL_MODE_CURRENT:
                // Direct RPS value
                packet.target_rps = cmd->target_value;
                break;

            default:
                return ERROR_INVALID_PARAMETER;
        }

        // Constrain RPS
        if (packet.target_rps > priv->config.max_rps) packet.target_rps = priv->config.max_rps;
        if (packet.target_rps < -priv->config.max_rps) packet.target_rps = -priv->config.max_rps;

        // Calculate CRC
        packet.crc = rs485_calculate_crc16((uint8_t*)&packet, sizeof(packet) - 2);

        // Transmit
        err = hw_uart_transmit(priv->config.uart_id, (uint8_t*)&packet, sizeof(packet));

    } else if (priv->detected_mode == RS485_MODE_POSITION) {
        // Position control mode - send count + rotation command
        rs485_position_write_t packet;
        packet.header = RS485_HEADER_WRITE;
        packet.id = priv->config.id;

        // Convert command to count + rotation based on mode
        if (cmd->mode == CONTROL_MODE_POSITION) {
            // radians to counts
            float target_radians = cmd->target_value;
            float total_counts = target_radians * (8192.0f / (2.0f * 3.14159265f));

            // Split into rotation and count
            packet.target_rotation = (int16_t)(total_counts / 8192.0f);
            packet.target_count = (int16_t)((int32_t)total_counts % 8192);

        } else {
            return ERROR_INVALID_PARAMETER;  // Position mode only supports position commands
        }

        // Constrain values
        if (packet.target_rotation > priv->config.max_rotation) {
            packet.target_rotation = priv->config.max_rotation;
        }
        if (packet.target_rotation < -priv->config.max_rotation) {
            packet.target_rotation = -priv->config.max_rotation;
        }

        // Calculate CRC
        packet.crc = rs485_calculate_crc16((uint8_t*)&packet, sizeof(packet) - 2);

        // Transmit
        err = hw_uart_transmit(priv->config.uart_id, (uint8_t*)&packet, sizeof(packet));

    } else {
        return ERROR_NOT_INITIALIZED;  // Mode not detected yet
    }

    if (err != ERROR_OK) {
        priv->consecutive_errors++;
        return err;
    }

    priv->consecutive_errors = 0;
    return ERROR_OK;
}

error_code_t rs485_detect_mode(motor_controller_t* controller) {
    // Send READ request to auto-detect control mode
    return rs485_read_status(controller);
}

/* ============================================================================
 * Motor Interface Implementation
 * ============================================================================ */

static error_code_t rs485_initialize(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    // Initialize state
    controller->state.status = ERROR_NOT_INITIALIZED;
    controller->state.enabled = false;
    controller->state.last_update_time = hw_get_tick();

    // Initialize private data
    priv->last_watchdog_reset = hw_get_tick();
    priv->last_response_time = hw_get_tick();
    priv->watchdog_expired = false;
    priv->consecutive_errors = 0;
    priv->crc_error_count = 0;
    priv->timeout_count = 0;

    // Auto-detect control mode
    error_code_t err = rs485_detect_mode(controller);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_HARDWARE_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    controller->state.enabled = true;
    return ERROR_OK;
}

static error_code_t rs485_update(motor_controller_t* controller, float delta_time) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;
    uint32_t now = hw_get_tick();

    // Check watchdog
    uint32_t elapsed = now - priv->last_watchdog_reset;
    if (elapsed > priv->config.watchdog_timeout_ms) {
        if (!priv->watchdog_expired) {
            priv->watchdog_expired = true;
            controller->state.status = ERROR_TIMEOUT;
            controller->state.timeout_count++;

            // Send zero command
            motor_command_t zero_cmd = {0};
            zero_cmd.motor_id = controller->id;
            zero_cmd.enable = true;
            zero_cmd.mode = (priv->detected_mode == RS485_MODE_VELOCITY) ?
                           CONTROL_MODE_VELOCITY : CONTROL_MODE_POSITION;
            zero_cmd.target_value = 0.0f;
            rs485_write_command(controller, &zero_cmd);
        }
    }

    // Periodic status read (every 100ms)
    static uint32_t last_status_read = 0;
    if (now - last_status_read > 100) {
        rs485_read_status(controller);
        last_status_read = now;
    }

    // Check for excessive errors
    if (priv->consecutive_errors >= RS485_MAX_RETRIES) {
        controller->state.status = ERROR_COMM_ERROR;
    }

    controller->state.last_update_time = now;
    return controller->state.status;
}

static error_code_t rs485_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (!controller || !controller->private_data || !cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    if (cmd->motor_id != controller->id) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    // Reset watchdog
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (!cmd->enable) {
        return motor_set_enabled(controller, false);
    }

    if (!controller->state.enabled) {
        motor_set_enabled(controller, true);
    }

    // Send command via RS485
    error_code_t err = rs485_write_command(controller, cmd);
    if (err != ERROR_OK) {
        controller->state.status = ERROR_COMM_ERROR;
        return err;
    }

    controller->state.status = ERROR_OK;
    return ERROR_OK;
}

static error_code_t rs485_set_enabled(motor_controller_t* controller, bool enabled) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    if (!enabled) {
        // Send zero command
        motor_command_t zero_cmd = {0};
        zero_cmd.motor_id = controller->id;
        zero_cmd.enable = true;
        zero_cmd.mode = (priv->detected_mode == RS485_MODE_VELOCITY) ?
                       CONTROL_MODE_VELOCITY : CONTROL_MODE_POSITION;
        zero_cmd.target_value = 0.0f;
        rs485_write_command(controller, &zero_cmd);
    }

    controller->state.enabled = enabled;
    return ERROR_OK;
}

static void rs485_emergency_stop(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    // Send zero command immediately
    motor_command_t zero_cmd = {0};
    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    zero_cmd.motor_id = controller->id;
    zero_cmd.enable = true;
    zero_cmd.mode = (priv->detected_mode == RS485_MODE_VELOCITY) ?
                   CONTROL_MODE_VELOCITY : CONTROL_MODE_POSITION;
    zero_cmd.target_value = 0.0f;
    rs485_write_command(controller, &zero_cmd);

    controller->state.enabled = false;
    controller->state.status = ERROR_SAFETY_VIOLATION;
}

static void rs485_reset_watchdog(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;
    priv->last_watchdog_reset = hw_get_tick();
    priv->watchdog_expired = false;

    if (controller->state.status == ERROR_TIMEOUT) {
        controller->state.status = ERROR_OK;
    }
}

static error_code_t rs485_self_test(motor_controller_t* controller) {
    // Try to detect mode as self-test
    return rs485_detect_mode(controller);
}

static motor_state_t rs485_get_state(const motor_controller_t* controller) {
    if (!controller) {
        motor_state_t empty = {0};
        empty.status = ERROR_INVALID_PARAMETER;
        return empty;
    }
    return controller->state;
}

static uint8_t rs485_get_id(const motor_controller_t* controller) {
    if (!controller) {
        return 0;
    }
    return controller->id;
}

/* ============================================================================
 * Virtual Function Table
 * ============================================================================ */

static const motor_vtable_t rs485_vtable = {
    .initialize = rs485_initialize,
    .update = rs485_update,
    .set_command = rs485_set_command,
    .set_enabled = rs485_set_enabled,
    .emergency_stop = rs485_emergency_stop,
    .reset_watchdog = rs485_reset_watchdog,
    .self_test = rs485_self_test,
    .get_state = rs485_get_state,
    .get_id = rs485_get_id
};

const motor_vtable_t* rs485_controller_get_vtable(void) {
    return &rs485_vtable;
}

/* ============================================================================
 * RS485 Controller Creation
 * ============================================================================ */

error_code_t rs485_controller_create(
    uint8_t id,
    const rs485_config_t* config,
    motor_controller_t* controller,
    rs485_private_t* private_data)
{
    if (!config || !controller || !private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    // Initialize controller base
    memset(controller, 0, sizeof(motor_controller_t));
    controller->vtable = &rs485_vtable;
    controller->id = id;
    controller->type = MOTOR_TYPE_DC;  // Reuse DC type for now
    controller->private_data = private_data;

    // Initialize private data
    memset(private_data, 0, sizeof(rs485_private_t));
    private_data->config = *config;
    private_data->detected_mode = config->control_mode;

    return ERROR_OK;
}
