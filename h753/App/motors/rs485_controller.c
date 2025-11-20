/**
 * @file rs485_controller.c
 * @brief RS485 Ikeya MD motor controller implementation
 */

#include "rs485_controller.h"
#include <string.h>
#include <math.h>
#include "cmsis_os.h"  // For osDelay()

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

static bool rs485_verify_crc(const uint8_t* packet, uint16_t packet_size) {
    if (packet_size < 4) return false;

    // Calculate CRC over all bytes except the last 2 (CRC field itself)
    uint16_t calculated_crc = rs485_calculate_crc16(packet, packet_size - 2);

    // Extract received CRC (little-endian)
    uint16_t received_crc = packet[packet_size - 2] | (packet[packet_size - 1] << 8);

    return (calculated_crc == received_crc);
}

/**
 * @brief Send packet and receive response with retry logic
 * @param uart_id UART peripheral ID
 * @param tx_packet Transmit packet buffer
 * @param tx_size Transmit packet size
 * @param rx_buffer Receive buffer
 * @param rx_size Expected receive size
 * @param timeout_ms Timeout per attempt
 * @param max_retries Maximum retry attempts (default: 3)
 * @return ERROR_OK on success, error code on failure
 */
static error_code_t rs485_transact_with_retry(
    uint8_t uart_id,
    const uint8_t* tx_packet,
    uint16_t tx_size,
    uint8_t* rx_buffer,
    uint16_t rx_size,
    uint32_t timeout_ms,
    uint8_t max_retries)
{
    error_code_t err;

    for (uint8_t attempt = 0; attempt < max_retries; attempt++) {
        // Send packet
        err = hw_uart_transmit(uart_id, tx_packet, tx_size);
        if (err != ERROR_OK) {
            continue;  // Retry on transmit error
        }

        // Receive response
        err = hw_uart_receive(uart_id, rx_buffer, rx_size, timeout_ms);
        if (err == ERROR_OK) {
            // Verify CRC
            if (rs485_verify_crc(rx_buffer, rx_size)) {
                return ERROR_OK;  // Success!
            }
            // CRC error, retry
            err = ERROR_COMM_ERROR;
        }

        // If not last attempt, continue retrying
        if (attempt < max_retries - 1) {
            // Small delay before retry (1ms)
            osDelay(1);
        }
    }

    // All retries exhausted
    return err;
}

/* ============================================================================
 * RS485 Protocol Functions
 * ============================================================================ */

error_code_t rs485_read_status(motor_controller_t* controller) {
    if (!controller || !controller->private_data) {
        return ERROR_INVALID_PARAMETER;
    }

    rs485_private_t* priv = (rs485_private_t*)controller->private_data;

    // Validate device ID is in valid range (1-9)
    if (priv->config.rs485_device_id == 0 || priv->config.rs485_device_id > 9) {
        return ERROR_CONFIG_ERROR;
    }

    // Prepare READ request packet
    rs485_read_request_t read_packet;
    read_packet.header = RS485_HEADER_READ_REQUEST;
    read_packet.id = priv->config.rs485_device_id;
    read_packet.crc = rs485_calculate_crc16((uint8_t*)&read_packet, sizeof(read_packet) - 2);

    // Send READ request and receive response with retry (up to 3 attempts)
    uint8_t response_buffer[16];
    error_code_t err = rs485_transact_with_retry(
        priv->config.uart_id,
        (uint8_t*)&read_packet,
        sizeof(read_packet),
        response_buffer,
        16,
        RS485_TIMEOUT_MS,
        RS485_MAX_RETRIES
    );

    if (err != ERROR_OK) {
        if (err == ERROR_TIMEOUT) {
            priv->timeout_count++;
        } else if (err == ERROR_COMM_ERROR) {
            priv->crc_error_count++;
        }
        priv->consecutive_errors++;
        return err;
    }

    // Parse response based on header
    uint8_t header = response_buffer[0];
    uint8_t id = response_buffer[1];

    // Verify ID matches
    if (id != priv->config.rs485_device_id) {
        priv->consecutive_errors++;
        return ERROR_COMM_ERROR;
    }

    if (header == RS485_HEADER_VEL_RESPONSE) {
        // Velocity control mode response (3 motors)
        rs485_velocity_response_t* vel_resp = (rs485_velocity_response_t*)response_buffer;

        // Store all 3 motor values
        for (int i = 0; i < 3; i++) {
            priv->current_rps[i] = vel_resp->rps[i];
        }

        priv->detected_mode = RS485_MODE_VELOCITY;

        // Update motor state for THIS motor (based on motor_index)
        uint8_t idx = priv->motor_index;
        controller->state.current_velocity = vel_resp->rps[idx] * 2.0f * 3.14159265f;  // RPS to rad/s

    } else if (header == RS485_HEADER_POS_RESPONSE) {
        // Position control mode response (3 motors)
        rs485_position_response_t* pos_resp = (rs485_position_response_t*)response_buffer;

        // Store all 3 motor values
        for (int i = 0; i < 3; i++) {
            priv->current_count[i] = pos_resp->count[i];
            priv->current_rotation[i] = pos_resp->rotation[i];
        }

        priv->detected_mode = RS485_MODE_POSITION;

        // Update motor state for THIS motor (based on motor_index)
        uint8_t idx = priv->motor_index;
        // Total position = (rotation * 8192) + count
        // Encoder resolution is 2048 x 4 = 8192 counts/revolution
        float total_counts = (float)pos_resp->rotation[idx] * 8192.0f + (float)pos_resp->count[idx];
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
    uint8_t idx = priv->motor_index;  // Motor index on the board (0-2)

    // Validate device ID is in valid range (1-9)
    if (priv->config.rs485_device_id == 0 || priv->config.rs485_device_id > 9) {
        return ERROR_CONFIG_ERROR;
    }

    uint8_t tx_buffer[16];
    uint8_t response_buffer[16];
    error_code_t err;

    if (priv->detected_mode == RS485_MODE_VELOCITY) {
        // Velocity control mode - send RPS command for all 3 motors
        rs485_velocity_write_t* packet = (rs485_velocity_write_t*)tx_buffer;
        packet->header = RS485_HEADER_WRITE;
        packet->id = priv->config.rs485_device_id;

        // Initialize all 3 motor values with their last known/commanded values
        // (Other motors on same board get their last values, this motor gets new command)
        for (int i = 0; i < 3; i++) {
            packet->target_rps[i] = priv->current_rps[i];  // Preserve other motors' values
        }

        // Convert command to RPS based on mode for THIS motor
        float target_rps;
        switch (cmd->mode) {
            case CONTROL_MODE_VELOCITY:
                // rad/s to RPS
                target_rps = cmd->target_value / (2.0f * 3.14159265f);
                break;

            case CONTROL_MODE_DUTY_CYCLE:
                // Map -1.0 to 1.0 → max RPS range
                target_rps = cmd->target_value * priv->config.max_rps;
                break;

            case CONTROL_MODE_CURRENT:
                // Direct RPS value
                target_rps = cmd->target_value;
                break;

            default:
                return ERROR_INVALID_PARAMETER;
        }

        // Constrain RPS
        if (target_rps > priv->config.max_rps) target_rps = priv->config.max_rps;
        if (target_rps < -priv->config.max_rps) target_rps = -priv->config.max_rps;

        // Update only THIS motor's value in the packet
        packet->target_rps[idx] = target_rps;

        // Update cache for next time
        priv->current_rps[idx] = target_rps;

        // Calculate CRC
        packet->crc = rs485_calculate_crc16(tx_buffer, sizeof(rs485_velocity_write_t) - 2);

        // Send WRITE and receive response with retry
        err = rs485_transact_with_retry(
            priv->config.uart_id,
            tx_buffer,
            sizeof(rs485_velocity_write_t),
            response_buffer,
            16,
            RS485_TIMEOUT_MS,
            RS485_MAX_RETRIES
        );

    } else if (priv->detected_mode == RS485_MODE_POSITION) {
        // Position control mode - send count + rotation command for all 3 motors
        rs485_position_write_t* packet = (rs485_position_write_t*)tx_buffer;
        packet->header = RS485_HEADER_WRITE;
        packet->id = priv->config.rs485_device_id;

        // Initialize all 3 motor values with their last known values
        for (int i = 0; i < 3; i++) {
            packet->target_count[i] = priv->current_count[i];
            packet->target_rotation[i] = priv->current_rotation[i];
        }

        // Convert command to count + rotation based on mode for THIS motor
        if (cmd->mode == CONTROL_MODE_POSITION) {
            // radians to counts
            float target_radians = cmd->target_value;
            float total_counts = target_radians * (8192.0f / (2.0f * 3.14159265f));

            // Split into rotation and count
            int16_t target_rotation = (int16_t)(total_counts / 8192.0f);
            int16_t target_count = (int16_t)((int32_t)total_counts % 8192);

            // Constrain values
            if (target_rotation > priv->config.max_rotation) {
                target_rotation = priv->config.max_rotation;
            }
            if (target_rotation < -priv->config.max_rotation) {
                target_rotation = -priv->config.max_rotation;
            }

            // Update only THIS motor's value in the packet
            packet->target_rotation[idx] = target_rotation;
            packet->target_count[idx] = target_count;

            // Update cache for next time
            priv->current_rotation[idx] = target_rotation;
            priv->current_count[idx] = target_count;

        } else {
            return ERROR_INVALID_PARAMETER;  // Position mode only supports position commands
        }

        // Calculate CRC
        packet->crc = rs485_calculate_crc16(tx_buffer, sizeof(rs485_position_write_t) - 2);

        // Send WRITE and receive response with retry
        err = rs485_transact_with_retry(
            priv->config.uart_id,
            tx_buffer,
            sizeof(rs485_position_write_t),
            response_buffer,
            16,
            RS485_TIMEOUT_MS,
            RS485_MAX_RETRIES
        );

    } else {
        return ERROR_NOT_INITIALIZED;  // Mode not detected yet
    }

    if (err != ERROR_OK) {
        if (err == ERROR_TIMEOUT) {
            priv->timeout_count++;
        } else if (err == ERROR_COMM_ERROR) {
            priv->crc_error_count++;
        }
        priv->consecutive_errors++;
        return err;
    }

    // Parse response to update motor state (motor returns current status after WRITE)
    uint8_t header = response_buffer[0];
    uint8_t id = response_buffer[1];

    // Verify ID matches
    if (id != priv->config.rs485_device_id) {
        priv->consecutive_errors++;
        return ERROR_COMM_ERROR;
    }

    if (header == RS485_HEADER_VEL_RESPONSE) {
        // Velocity control mode response (3 motors)
        rs485_velocity_response_t* vel_resp = (rs485_velocity_response_t*)response_buffer;

        // Store all 3 motor values
        for (int i = 0; i < 3; i++) {
            priv->current_rps[i] = vel_resp->rps[i];
        }

        // Update motor state for THIS motor (based on motor_index)
        controller->state.current_velocity = vel_resp->rps[idx] * 2.0f * 3.14159265f;  // RPS to rad/s

    } else if (header == RS485_HEADER_POS_RESPONSE) {
        // Position control mode response (3 motors)
        rs485_position_response_t* pos_resp = (rs485_position_response_t*)response_buffer;

        // Store all 3 motor values
        for (int i = 0; i < 3; i++) {
            priv->current_count[i] = pos_resp->count[i];
            priv->current_rotation[i] = pos_resp->rotation[i];
        }

        // Update motor state for THIS motor (based on motor_index)
        // Total position = (rotation * 8192) + count
        float total_counts = (float)pos_resp->rotation[idx] * 8192.0f + (float)pos_resp->count[idx];
        controller->state.current_position = total_counts * (2.0f * 3.14159265f / 8192.0f);  // counts to radians

    } else {
        priv->consecutive_errors++;
        return ERROR_COMM_ERROR;
    }

    priv->last_response_time = hw_get_tick();
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
    priv->last_status_read_time = hw_get_tick();
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

    // Periodic status read (every RS485_STATUS_READ_INTERVAL_MS)
    // This ensures we always have fresh feedback from the motor
    if (now - priv->last_status_read_time >= RS485_STATUS_READ_INTERVAL_MS) {
        error_code_t read_err = rs485_read_status(controller);
        priv->last_status_read_time = now;

        // If read fails after retries, mark error but continue operation
        if (read_err != ERROR_OK && controller->state.status == ERROR_OK) {
            controller->state.status = read_err;
        }
    }

    // Check for excessive consecutive errors (timeout after 3 failures)
    if (priv->consecutive_errors >= RS485_MAX_RETRIES) {
        controller->state.status = ERROR_COMM_ERROR;
        controller->state.enabled = false;  // Disable motor on persistent comm failure
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

    // Validate RS485 device ID is in valid range (1-9)
    if (config->rs485_device_id == 0 || config->rs485_device_id > 9) {
        return ERROR_CONFIG_ERROR;
    }

    // Validate motor index is in valid range (0-2 for 3 motors per board)
    if (config->motor_index > 2) {
        return ERROR_CONFIG_ERROR;
    }

    // Initialize controller base
    memset(controller, 0, sizeof(motor_controller_t));
    controller->vtable = &rs485_vtable;
    controller->id = id;
    controller->type = MOTOR_TYPE_RS485;  // Correct RS485 motor type
    controller->private_data = private_data;

    // Initialize private data
    memset(private_data, 0, sizeof(rs485_private_t));
    private_data->config = *config;
    private_data->motor_index = config->motor_index;  // Set motor index (0-2)
    private_data->detected_mode = config->control_mode;

    return ERROR_OK;
}
