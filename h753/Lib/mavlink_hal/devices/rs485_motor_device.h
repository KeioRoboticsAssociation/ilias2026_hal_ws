/**
 * @file rs485_motor_device.h
 * @brief RS485 motor device implementation using unified device interface
 *
 * Provides RS485 Ikeya MD motor control via UART:
 * - Position control (position mode)
 * - Velocity control (velocity mode)
 * - Built-in encoder feedback
 * - RS485 UART protocol (500 kbps)
 * - CRC-16 Modbus validation
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_RS485_MOTOR_DEVICE_H
#define MAVLINK_RS485_MOTOR_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../include/mavlink_device_interface.h"

/* ========================================================================== */
/*  RS485 MOTOR PRIVATE DATA                                                  */
/* ========================================================================== */

/**
 * @brief Platform-specific RS485 motor data
 */
typedef struct {
    /* Hardware resources */
    void* uart_handle;           /**< UART handle (e.g., UART_HandleTypeDef*) */
    uint8_t rs485_device_id;     /**< RS485 device ID (1-8, set via DIP switch) */

    /* Control mode */
    mavlink_control_mode_t control_mode;

    /* Target values */
    float target_position;       /**< Target position in radians */
    float target_velocity;       /**< Target velocity in RPS */

    /* Feedback */
    float position;              /**< Current position in radians */
    float velocity;              /**< Current velocity in RPS */
    bool feedback_valid;

    /* Configuration */
    float max_velocity_rps;      /**< Maximum velocity in RPS */
    float max_acceleration_rps2; /**< Maximum acceleration in RPS² */

    /* Communication state */
    uint32_t last_command_time_ms;
    uint32_t retry_count;
    uint8_t tx_buffer[16];
    uint8_t rx_buffer[16];

} rs485_motor_private_data_t;

/* ========================================================================== */
/*  RS485 MOTOR DEVICE FACTORY                                                */
/* ========================================================================== */

/**
 * @brief Create RS485 motor device
 *
 * @param id Device ID (MAVLink motor ID)
 * @param name Device name
 * @param config Device configuration
 * @param uart_handle UART handle
 * @param rs485_device_id RS485 device ID (1-8)
 * @param max_velocity_rps Maximum velocity in RPS
 * @param max_acceleration_rps2 Maximum acceleration in RPS²
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* rs485_motor_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* uart_handle,
    uint8_t rs485_device_id,
    float max_velocity_rps,
    float max_acceleration_rps2
);

/**
 * @brief Get RS485 motor vtable
 *
 * @return Pointer to RS485 motor device vtable
 */
const mavlink_device_vtable_t* rs485_motor_device_get_vtable(void);

/* ========================================================================== */
/*  RS485 PROTOCOL FUNCTIONS                                                  */
/* ========================================================================== */

/**
 * @brief Send position command via RS485
 */
mavlink_device_error_t rs485_send_position_command(
    rs485_motor_private_data_t* priv,
    float position_rad
);

/**
 * @brief Send velocity command via RS485
 */
mavlink_device_error_t rs485_send_velocity_command(
    rs485_motor_private_data_t* priv,
    float velocity_rps,
    float acceleration_rps2
);

/**
 * @brief Request feedback from RS485 motor
 */
mavlink_device_error_t rs485_request_feedback(
    rs485_motor_private_data_t* priv
);

/**
 * @brief Calculate CRC-16 Modbus
 */
uint16_t rs485_crc16(const uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_RS485_MOTOR_DEVICE_H */
