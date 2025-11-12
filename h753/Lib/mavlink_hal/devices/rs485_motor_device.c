/**
 * @file rs485_motor_device.c
 * @brief RS485 motor device implementation using unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "rs485_motor_device.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Platform-specific includes (STM32 HAL) */
#ifdef STM32H7
#include "stm32h7xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif

/* ========================================================================== */
/*  RS485 PROTOCOL CONSTANTS                                                  */
/* ========================================================================== */

#define RS485_CMD_POSITION      0x01
#define RS485_CMD_VELOCITY      0x02
#define RS485_CMD_FEEDBACK      0x03
#define RS485_CMD_STOP          0x04

#define RS485_TIMEOUT_MS        100
#define RS485_MAX_RETRIES       3

/* ========================================================================== */
/*  FORWARD DECLARATIONS                                                      */
/* ========================================================================== */

static mavlink_device_error_t rs485_init(mavlink_device_t* device);
static mavlink_device_error_t rs485_update(mavlink_device_t* device, uint32_t dt_ms);
static mavlink_device_error_t rs485_shutdown(mavlink_device_t* device);
static mavlink_device_error_t rs485_enable(mavlink_device_t* device, bool enable);
static mavlink_device_error_t rs485_command(mavlink_device_t* device, const mavlink_device_command_t* command);
static mavlink_device_error_t rs485_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback);
static mavlink_device_error_t rs485_get_status(mavlink_device_t* device, mavlink_device_status_t* status);
static mavlink_device_error_t rs485_set_param(mavlink_device_t* device, const char* name, float value);
static mavlink_device_error_t rs485_get_param(mavlink_device_t* device, const char* name, float* value);

/* ========================================================================== */
/*  RS485 VTABLE                                                              */
/* ========================================================================== */

static const mavlink_device_vtable_t rs485_vtable = {
    .init = rs485_init,
    .update = rs485_update,
    .shutdown = rs485_shutdown,
    .enable = rs485_enable,
    .command = rs485_command,
    .get_feedback = rs485_get_feedback,
    .get_status = rs485_get_status,
    .set_param = rs485_set_param,
    .get_param = rs485_get_param,
    .self_test = NULL,
    .calibrate = NULL,
};

/* ========================================================================== */
/*  RS485 PROTOCOL IMPLEMENTATION                                             */
/* ========================================================================== */

uint16_t rs485_crc16(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
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

mavlink_device_error_t rs485_send_position_command(
    rs485_motor_private_data_t* priv,
    float position_rad)
{
    if (!priv || !priv->uart_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Build command packet */
    uint8_t* buf = priv->tx_buffer;
    buf[0] = priv->rs485_device_id;
    buf[1] = RS485_CMD_POSITION;

    /* Convert position to motor units (example: 1 rad = 1000 units) */
    int32_t position_units = (int32_t)(position_rad * 1000.0f);
    buf[2] = (position_units >> 24) & 0xFF;
    buf[3] = (position_units >> 16) & 0xFF;
    buf[4] = (position_units >> 8) & 0xFF;
    buf[5] = position_units & 0xFF;

    /* Calculate CRC */
    uint16_t crc = rs485_crc16(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = (crc >> 8) & 0xFF;

#if defined(STM32H7) || defined(STM32F4)
    /* Send via UART */
    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)priv->uart_handle;
    if (HAL_UART_Transmit(huart, buf, 8, RS485_TIMEOUT_MS) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t rs485_send_velocity_command(
    rs485_motor_private_data_t* priv,
    float velocity_rps,
    float acceleration_rps2)
{
    if (!priv || !priv->uart_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Clamp velocity */
    if (velocity_rps > priv->max_velocity_rps) velocity_rps = priv->max_velocity_rps;
    if (velocity_rps < -priv->max_velocity_rps) velocity_rps = -priv->max_velocity_rps;

    /* Clamp acceleration */
    if (acceleration_rps2 > priv->max_acceleration_rps2) acceleration_rps2 = priv->max_acceleration_rps2;

    /* Build command packet */
    uint8_t* buf = priv->tx_buffer;
    buf[0] = priv->rs485_device_id;
    buf[1] = RS485_CMD_VELOCITY;

    /* Convert velocity to motor units (example: 1 RPS = 100 units) */
    int16_t velocity_units = (int16_t)(velocity_rps * 100.0f);
    buf[2] = (velocity_units >> 8) & 0xFF;
    buf[3] = velocity_units & 0xFF;

    /* Convert acceleration to motor units */
    int16_t accel_units = (int16_t)(acceleration_rps2 * 10.0f);
    buf[4] = (accel_units >> 8) & 0xFF;
    buf[5] = accel_units & 0xFF;

    /* Calculate CRC */
    uint16_t crc = rs485_crc16(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = (crc >> 8) & 0xFF;

#if defined(STM32H7) || defined(STM32F4)
    /* Send via UART */
    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)priv->uart_handle;
    if (HAL_UART_Transmit(huart, buf, 8, RS485_TIMEOUT_MS) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t rs485_request_feedback(
    rs485_motor_private_data_t* priv)
{
    if (!priv || !priv->uart_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Build request packet */
    uint8_t* buf = priv->tx_buffer;
    buf[0] = priv->rs485_device_id;
    buf[1] = RS485_CMD_FEEDBACK;

    /* Calculate CRC */
    uint16_t crc = rs485_crc16(buf, 2);
    buf[2] = crc & 0xFF;
    buf[3] = (crc >> 8) & 0xFF;

#if defined(STM32H7) || defined(STM32F4)
    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)priv->uart_handle;

    /* Send request */
    if (HAL_UART_Transmit(huart, buf, 4, RS485_TIMEOUT_MS) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }

    /* Receive response (8 bytes: ID, CMD, position(4), CRC(2)) */
    if (HAL_UART_Receive(huart, priv->rx_buffer, 8, RS485_TIMEOUT_MS) != HAL_OK) {
        priv->feedback_valid = false;
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }

    /* Verify CRC */
    uint16_t rx_crc = (priv->rx_buffer[7] << 8) | priv->rx_buffer[6];
    uint16_t calc_crc = rs485_crc16(priv->rx_buffer, 6);
    if (rx_crc != calc_crc) {
        priv->feedback_valid = false;
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }

    /* Parse feedback */
    int32_t position_units = (int32_t)((priv->rx_buffer[2] << 24) |
                                        (priv->rx_buffer[3] << 16) |
                                        (priv->rx_buffer[4] << 8) |
                                        priv->rx_buffer[5]);

    priv->position = (float)position_units / 1000.0f;  /* Convert to radians */
    priv->feedback_valid = true;
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  VTABLE IMPLEMENTATIONS                                                    */
/* ========================================================================== */

static mavlink_device_error_t rs485_init(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;
    priv->control_mode = MAVLINK_CONTROL_MODE_VELOCITY;
    priv->feedback_valid = false;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_update(mavlink_device_t* device, uint32_t dt_ms)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    if (!device->status.enabled) {
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    /* Request feedback periodically */
    rs485_request_feedback(priv);

    /* Calculate velocity from position change */
    static float prev_position = 0.0f;
    if (priv->feedback_valid && dt_ms > 0) {
        float dt = dt_ms / 1000.0f;
        priv->velocity = (priv->position - prev_position) / dt;
        prev_position = priv->position;
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_shutdown(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    /* Send stop command */
    rs485_send_velocity_command(priv, 0.0f, priv->max_acceleration_rps2);

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_enable(mavlink_device_t* device, bool enable)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    if (!enable) {
        /* Stop motor */
        rs485_send_velocity_command(priv, 0.0f, priv->max_acceleration_rps2);
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_command(mavlink_device_t* device, const mavlink_device_command_t* command)
{
    if (!device || !device->private_data || !command) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;
    priv->control_mode = command->mode;

    switch (command->mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            priv->target_position = command->data.position.target;
            return rs485_send_position_command(priv, priv->target_position);

        case MAVLINK_CONTROL_MODE_VELOCITY:
            priv->target_velocity = command->data.velocity.target;
            return rs485_send_velocity_command(priv, priv->target_velocity,
                                                priv->max_acceleration_rps2);

        default:
            return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }
}

static mavlink_device_error_t rs485_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback)
{
    if (!device || !device->private_data || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    if (!priv->feedback_valid) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }

    feedback->type = MAVLINK_DEVICE_TYPE_RS485_MOTOR;
    feedback->data.motor.position = priv->position;
    feedback->data.motor.velocity = priv->velocity;
    feedback->data.motor.current = 0.0f;
    feedback->data.motor.torque = 0.0f;
    feedback->data.motor.temperature = 0.0f;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_get_status(mavlink_device_t* device, mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    *status = device->status;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rs485_set_param(mavlink_device_t* device, const char* name, float value)
{
    if (!device || !device->private_data || !name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    if (strcmp(name, "max_velocity") == 0) {
        priv->max_velocity_rps = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_acceleration") == 0) {
        priv->max_acceleration_rps2 = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t rs485_get_param(mavlink_device_t* device, const char* name, float* value)
{
    if (!device || !device->private_data || !name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)device->private_data;

    if (strcmp(name, "max_velocity") == 0) {
        *value = priv->max_velocity_rps;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "max_acceleration") == 0) {
        *value = priv->max_acceleration_rps2;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

/* ========================================================================== */
/*  PUBLIC API                                                                */
/* ========================================================================== */

mavlink_device_t* rs485_motor_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* uart_handle,
    uint8_t rs485_device_id,
    float max_velocity_rps,
    float max_acceleration_rps2)
{
    if (!config || !uart_handle) {
        return NULL;
    }

    /* Allocate device */
    mavlink_device_t* device = (mavlink_device_t*)malloc(sizeof(mavlink_device_t));
    if (!device) {
        return NULL;
    }

    /* Allocate private data */
    rs485_motor_private_data_t* priv = (rs485_motor_private_data_t*)malloc(sizeof(rs485_motor_private_data_t));
    if (!priv) {
        free(device);
        return NULL;
    }

    /* Initialize private data */
    memset(priv, 0, sizeof(rs485_motor_private_data_t));
    priv->uart_handle = uart_handle;
    priv->rs485_device_id = rs485_device_id;
    priv->max_velocity_rps = max_velocity_rps;
    priv->max_acceleration_rps2 = max_acceleration_rps2;
    priv->control_mode = MAVLINK_CONTROL_MODE_VELOCITY;

    /* Initialize device */
    mavlink_device_error_t err = mavlink_device_init(
        device,
        MAVLINK_DEVICE_TYPE_RS485_MOTOR,
        id,
        name,
        config,
        &rs485_vtable,
        priv
    );

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        free(priv);
        free(device);
        return NULL;
    }

    return device;
}

const mavlink_device_vtable_t* rs485_motor_device_get_vtable(void)
{
    return &rs485_vtable;
}
