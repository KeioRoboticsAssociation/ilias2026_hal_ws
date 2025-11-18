/**
 * @file rs485_controller.h
 * @brief RS485 Ikeya MD motor controller for H753 platform
 *
 * Implements RS485 communication protocol for Ikeya MD motors
 * Supports both velocity control (RPS) and position control (count + rotation)
 */

#ifndef RS485_CONTROLLER_H
#define RS485_CONTROLLER_H

#include "motor_interface.h"
#include "../hal/hardware_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * RS485 Protocol Definitions
 * ============================================================================ */

// Protocol headers
#define RS485_HEADER_READ_REQUEST       0xAA
#define RS485_HEADER_VEL_RESPONSE       0x55
#define RS485_HEADER_POS_RESPONSE       0x56
#define RS485_HEADER_WRITE              0xBB

// Communication parameters
#define RS485_BAUDRATE                  500000
#define RS485_TIMEOUT_MS                5      // Response timeout per attempt
#define RS485_MAX_RETRIES               3      // Maximum retry attempts
#define RS485_STATUS_READ_INTERVAL_MS   100    // Periodic status read interval

/* ============================================================================
 * RS485 Protocol Packets
 * ============================================================================ */

// READ request packet (4 bytes) - unchanged
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0xAA
    uint8_t id;         // 1-8
    uint16_t crc;       // CRC-16 Modbus (little-endian)
} rs485_read_request_t;

// Velocity control response packet (16 bytes) - 3 motors
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0x55
    uint8_t id;         // 1-8
    float rps[3];       // Current RPS for 3 motors (3 × 4 bytes = 12 bytes)
    uint16_t crc;       // CRC-16 Modbus (little-endian)
} rs485_velocity_response_t;

// Position control response packet (16 bytes) - 3 motors
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0x56
    uint8_t id;         // 1-8
    int16_t count[3];      // Encoder count values for 3 motors (3 × 2 bytes)
    int16_t rotation[3];   // Number of rotations for 3 motors (3 × 2 bytes)
    uint16_t crc;       // CRC-16 Modbus (little-endian)
} rs485_position_response_t;

// WRITE packet for velocity control (16 bytes) - 3 motors
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xBB
    uint8_t id;           // 1-8
    float target_rps[3];  // Target RPS for 3 motors (3 × 4 bytes = 12 bytes)
    uint16_t crc;         // CRC-16 Modbus (little-endian)
} rs485_velocity_write_t;

// WRITE packet for position control (16 bytes) - 3 motors
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0xBB
    uint8_t id;         // 1-8
    int16_t target_count[3];    // Target count for 3 motors (3 × 2 bytes)
    int16_t target_rotation[3]; // Target rotation for 3 motors (3 × 2 bytes)
    uint16_t crc;       // CRC-16 Modbus (little-endian)
} rs485_position_write_t;

/* ============================================================================
 * RS485 Controller Private Data
 * ============================================================================ */
typedef struct {
    rs485_config_t config;
    uint32_t last_watchdog_reset;
    bool watchdog_expired;
    uint32_t last_response_time;
    uint32_t last_status_read_time;  // Timestamp of last periodic status read

    // Motor index on the board (0-2 for 3 motors per board)
    uint8_t motor_index;

    // Protocol state
    rs485_control_mode_t detected_mode;
    uint8_t consecutive_errors;

    // Feedback data (velocity mode) - all 3 motors on the board
    float current_rps[3];

    // Feedback data (position mode) - all 3 motors on the board
    int16_t current_count[3];
    int16_t current_rotation[3];

    // Statistics
    uint32_t crc_error_count;
    uint32_t timeout_count;
} rs485_private_t;

/* ============================================================================
 * RS485 Controller API
 * ============================================================================ */

/**
 * @brief Create a new RS485 motor controller
 */
error_code_t rs485_controller_create(
    uint8_t id,
    const rs485_config_t* config,
    motor_controller_t* controller,
    rs485_private_t* private_data);

/**
 * @brief Get the RS485 controller virtual function table
 */
const motor_vtable_t* rs485_controller_get_vtable(void);

/**
 * @brief Send READ request and receive response
 * @return ERROR_OK on success, error code otherwise
 */
error_code_t rs485_read_status(motor_controller_t* controller);

/**
 * @brief Send WRITE command (velocity or position)
 * @return ERROR_OK on success, error code otherwise
 */
error_code_t rs485_write_command(motor_controller_t* controller, const motor_command_t* cmd);

/**
 * @brief Auto-detect motor control mode by sending READ request
 * @return ERROR_OK on success, error code otherwise
 */
error_code_t rs485_detect_mode(motor_controller_t* controller);

/**
 * @brief Calculate CRC-16 Modbus
 */
uint16_t rs485_calculate_crc16(const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* RS485_CONTROLLER_H */
