/**
 * @file robomaster_device.c
 * @brief RoboMaster motor device implementation using unified device interface
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "robomaster_device.h"
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
/*  ROBOMASTER CONSTANTS                                                      */
/* ========================================================================== */

#define RM_ANGLE_RAW_MAX        8191
#define RM_ANGLE_TO_RAD(raw)    ((float)(raw) / (float)RM_ANGLE_RAW_MAX * 2.0f * 3.14159265359f)
#define RM_RPM_TO_RAD_S(rpm)    ((float)(rpm) / 60.0f * 2.0f * 3.14159265359f)

/* Motor-specific limits */
#define RM_GM6020_MAX_CURRENT   30000  /**< GM6020 max current (raw) */
#define RM_M3508_MAX_CURRENT    16384  /**< M3508 max current (raw) */
#define RM_M2006_MAX_CURRENT    10000  /**< M2006 max current (raw) */

/* CAN message IDs */
#define RM_CAN_ID_CMD_1_4       0x200  /**< Command for motors 1-4 (M3508/M2006) */
#define RM_CAN_ID_CMD_5_8       0x1FF  /**< Command for motors 5-8 (M3508/M2006) */
#define RM_CAN_ID_CMD_GM_1_4    0x1FF  /**< Command for GM6020 motors 1-4 */
#define RM_CAN_ID_CMD_GM_5_7    0x2FF  /**< Command for GM6020 motors 5-7 */

/* ========================================================================== */
/*  FORWARD DECLARATIONS                                                      */
/* ========================================================================== */

static mavlink_device_error_t rm_init(mavlink_device_t* device);
static mavlink_device_error_t rm_update(mavlink_device_t* device, uint32_t dt_ms);
static mavlink_device_error_t rm_shutdown(mavlink_device_t* device);
static mavlink_device_error_t rm_enable(mavlink_device_t* device, bool enable);
static mavlink_device_error_t rm_command(mavlink_device_t* device, const mavlink_device_command_t* command);
static mavlink_device_error_t rm_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback);
static mavlink_device_error_t rm_get_status(mavlink_device_t* device, mavlink_device_status_t* status);
static mavlink_device_error_t rm_set_param(mavlink_device_t* device, const char* name, float value);
static mavlink_device_error_t rm_get_param(mavlink_device_t* device, const char* name, float* value);

/* ========================================================================== */
/*  ROBOMASTER VTABLE                                                         */
/* ========================================================================== */

static const mavlink_device_vtable_t rm_vtable = {
    .init = rm_init,
    .update = rm_update,
    .shutdown = rm_shutdown,
    .enable = rm_enable,
    .command = rm_command,
    .get_feedback = rm_get_feedback,
    .get_status = rm_get_status,
    .set_param = rm_set_param,
    .get_param = rm_get_param,
    .self_test = NULL,
    .calibrate = NULL,
};

/* ========================================================================== */
/*  UTILITY FUNCTIONS                                                         */
/* ========================================================================== */

/**
 * @brief Get max current for motor type
 */
static int16_t rm_get_max_current(robomaster_motor_type_t type)
{
    switch (type) {
        case ROBOMASTER_TYPE_GM6020: return RM_GM6020_MAX_CURRENT;
        case ROBOMASTER_TYPE_M3508:  return RM_M3508_MAX_CURRENT;
        case ROBOMASTER_TYPE_M2006:  return RM_M2006_MAX_CURRENT;
        default: return 0;
    }
}

/**
 * @brief Convert current (Amps) to raw units
 *
 * Current scaling varies by motor type.
 */
static int16_t rm_current_to_raw(float current_amps, robomaster_motor_type_t type)
{
    int16_t raw = 0;

    switch (type) {
        case ROBOMASTER_TYPE_GM6020:
            /* GM6020: -30000 to +30000 corresponds to ~±3A */
            raw = (int16_t)(current_amps * 10000.0f);
            break;

        case ROBOMASTER_TYPE_M3508:
            /* M3508: -16384 to +16384 corresponds to ~±20A */
            raw = (int16_t)(current_amps * 819.2f);
            break;

        case ROBOMASTER_TYPE_M2006:
            /* M2006: -10000 to +10000 corresponds to ~±10A */
            raw = (int16_t)(current_amps * 1000.0f);
            break;
    }

    /* Clamp to motor limits */
    int16_t max_current = rm_get_max_current(type);
    if (raw > max_current) raw = max_current;
    if (raw < -max_current) raw = -max_current;

    return raw;
}

/**
 * @brief Convert raw current to Amps
 */
static float rm_raw_to_current(int16_t raw, robomaster_motor_type_t type)
{
    switch (type) {
        case ROBOMASTER_TYPE_GM6020:
            return (float)raw / 10000.0f;
        case ROBOMASTER_TYPE_M3508:
            return (float)raw / 819.2f;
        case ROBOMASTER_TYPE_M2006:
            return (float)raw / 1000.0f;
        default:
            return 0.0f;
    }
}

/* ========================================================================== */
/*  VTABLE IMPLEMENTATIONS                                                    */
/* ========================================================================== */

static mavlink_device_error_t rm_init(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    /* Initialize PID controllers */
    pid_init(&priv->speed_pid,
             device->config.pid_velocity.kp,
             device->config.pid_velocity.ki,
             device->config.pid_velocity.kd,
             (float)(-priv->max_current_raw),
             (float)priv->max_current_raw,
             1000.0f);

    pid_init(&priv->angle_pid,
             device->config.pid_position.kp,
             device->config.pid_position.ki,
             device->config.pid_position.kd,
             -100.0f, 100.0f, 100.0f);  /* Output is target velocity */

    priv->control_mode = MAVLINK_CONTROL_MODE_CURRENT;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_update(mavlink_device_t* device, uint32_t dt_ms)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;
    float dt = dt_ms / 1000.0f;

    if (!device->status.enabled) {
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    /* Run closed-loop control based on mode */
    int16_t target_current_raw = 0;

    switch (priv->control_mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            /* Angle control: PID(angle) -> velocity -> PID(velocity) -> current */
            {
                float target_vel = pid_update(&priv->angle_pid, priv->target_angle,
                                               priv->feedback.angle_rad, dt);
                float current_amps = pid_update(&priv->speed_pid, target_vel,
                                                 priv->feedback.velocity_rad_s, dt);
                target_current_raw = rm_current_to_raw(current_amps, priv->motor_type);
            }
            break;

        case MAVLINK_CONTROL_MODE_VELOCITY:
            /* Speed control: PID(velocity) -> current */
            {
                float current_amps = pid_update(&priv->speed_pid, priv->target_velocity,
                                                 priv->feedback.velocity_rad_s, dt);
                target_current_raw = rm_current_to_raw(current_amps, priv->motor_type);
            }
            break;

        case MAVLINK_CONTROL_MODE_CURRENT:
            /* Direct current control */
            target_current_raw = rm_current_to_raw(priv->target_current, priv->motor_type);
            break;

        default:
            return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }

    /* Store command for batch send (done externally via robomaster_send_can_commands) */
    priv->feedback.current_raw = target_current_raw;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_shutdown(mavlink_device_t* device)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    /* Set current to zero */
    priv->target_current = 0.0f;
    priv->feedback.current_raw = 0;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_enable(mavlink_device_t* device, bool enable)
{
    if (!device || !device->private_data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    if (!enable) {
        /* Reset PIDs */
        pid_reset(&priv->speed_pid);
        pid_reset(&priv->angle_pid);
        priv->target_current = 0.0f;
        priv->feedback.current_raw = 0;
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_command(mavlink_device_t* device, const mavlink_device_command_t* command)
{
    if (!device || !device->private_data || !command) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    priv->control_mode = command->mode;

    switch (command->mode) {
        case MAVLINK_CONTROL_MODE_POSITION:
            priv->target_angle = command->data.position.target;
            break;

        case MAVLINK_CONTROL_MODE_VELOCITY:
            priv->target_velocity = command->data.velocity.target;
            break;

        case MAVLINK_CONTROL_MODE_CURRENT:
            priv->target_current = command->data.current.target;
            break;

        default:
            return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_get_feedback(mavlink_device_t* device, mavlink_device_feedback_t* feedback)
{
    if (!device || !device->private_data || !feedback) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    feedback->type = MAVLINK_DEVICE_TYPE_ROBOMASTER;
    feedback->data.motor.position = priv->feedback.angle_rad;
    feedback->data.motor.velocity = priv->feedback.velocity_rad_s;
    feedback->data.motor.current = priv->feedback.current_amps;
    feedback->data.motor.torque = 0.0f;
    feedback->data.motor.temperature = (float)priv->feedback.temperature;

    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_get_status(mavlink_device_t* device, mavlink_device_status_t* status)
{
    if (!device || !status) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    *status = device->status;
    return MAVLINK_DEVICE_ERROR_NONE;
}

static mavlink_device_error_t rm_set_param(mavlink_device_t* device, const char* name, float value)
{
    if (!device || !device->private_data || !name) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    /* Speed PID parameters */
    if (strcmp(name, "speed_kp") == 0) {
        priv->speed_pid.kp = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "speed_ki") == 0) {
        priv->speed_pid.ki = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "speed_kd") == 0) {
        priv->speed_pid.kd = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }
    /* Angle PID parameters */
    else if (strcmp(name, "angle_kp") == 0) {
        priv->angle_pid.kp = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "angle_ki") == 0) {
        priv->angle_pid.ki = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "angle_kd") == 0) {
        priv->angle_pid.kd = value;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

static mavlink_device_error_t rm_get_param(mavlink_device_t* device, const char* name, float* value)
{
    if (!device || !device->private_data || !name || !value) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    /* Speed PID parameters */
    if (strcmp(name, "speed_kp") == 0) {
        *value = priv->speed_pid.kp;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "speed_ki") == 0) {
        *value = priv->speed_pid.ki;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "speed_kd") == 0) {
        *value = priv->speed_pid.kd;
        return MAVLINK_DEVICE_ERROR_NONE;
    }
    /* Angle PID parameters */
    else if (strcmp(name, "angle_kp") == 0) {
        *value = priv->angle_pid.kp;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "angle_ki") == 0) {
        *value = priv->angle_pid.ki;
        return MAVLINK_DEVICE_ERROR_NONE;
    } else if (strcmp(name, "angle_kd") == 0) {
        *value = priv->angle_pid.kd;
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    return MAVLINK_DEVICE_ERROR_UNSUPPORTED;
}

/* ========================================================================== */
/*  PUBLIC API                                                                */
/* ========================================================================== */

mavlink_device_t* robomaster_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* can_handle,
    uint8_t can_id,
    robomaster_motor_type_t motor_type)
{
    if (!config || !can_handle) {
        return NULL;
    }

    /* Allocate device */
    mavlink_device_t* device = (mavlink_device_t*)malloc(sizeof(mavlink_device_t));
    if (!device) {
        return NULL;
    }

    /* Allocate private data */
    robomaster_private_data_t* priv = (robomaster_private_data_t*)malloc(sizeof(robomaster_private_data_t));
    if (!priv) {
        free(device);
        return NULL;
    }

    /* Initialize private data */
    memset(priv, 0, sizeof(robomaster_private_data_t));
    priv->can_handle = can_handle;
    priv->can_id = can_id;
    priv->motor_type = motor_type;
    priv->max_current_raw = rm_get_max_current(motor_type);
    priv->control_mode = MAVLINK_CONTROL_MODE_CURRENT;

    /* Initialize device */
    mavlink_device_error_t err = mavlink_device_init(
        device,
        MAVLINK_DEVICE_TYPE_ROBOMASTER,
        id,
        name,
        config,
        &rm_vtable,
        priv
    );

    if (err != MAVLINK_DEVICE_ERROR_NONE) {
        free(priv);
        free(device);
        return NULL;
    }

    return device;
}

const mavlink_device_vtable_t* robomaster_device_get_vtable(void)
{
    return &rm_vtable;
}

/* ========================================================================== */
/*  CAN COMMUNICATION                                                         */
/* ========================================================================== */

mavlink_device_error_t robomaster_process_can_feedback(
    mavlink_device_t* device,
    const uint8_t* data)
{
    if (!device || !device->private_data || !data) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    robomaster_private_data_t* priv = (robomaster_private_data_t*)device->private_data;

    /* Parse CAN feedback (8 bytes) */
    priv->feedback.angle_raw = (uint16_t)((data[0] << 8) | data[1]);
    priv->feedback.velocity_rpm = (int16_t)((data[2] << 8) | data[3]);
    priv->feedback.current_raw = (int16_t)((data[4] << 8) | data[5]);
    priv->feedback.temperature = data[6];

    /* Convert to SI units */
    priv->feedback.angle_rad = RM_ANGLE_TO_RAD(priv->feedback.angle_raw);
    priv->feedback.velocity_rad_s = RM_RPM_TO_RAD_S(priv->feedback.velocity_rpm);
    priv->feedback.current_amps = rm_raw_to_current(priv->feedback.current_raw, priv->motor_type);

    return MAVLINK_DEVICE_ERROR_NONE;
}

mavlink_device_error_t robomaster_send_can_commands(
    void* can_handle,
    mavlink_device_t* devices[4])
{
    if (!can_handle) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    /* Prepare CAN message (8 bytes for 4 motors) */
    uint8_t data[8] = {0};

    for (int i = 0; i < 4; i++) {
        int16_t current = 0;

        if (devices[i] && devices[i]->private_data) {
            robomaster_private_data_t* priv = (robomaster_private_data_t*)devices[i]->private_data;
            current = priv->feedback.current_raw;  /* Get commanded current */
        }

        /* Pack current into CAN message (big-endian) */
        data[i * 2] = (uint8_t)((current >> 8) & 0xFF);
        data[i * 2 + 1] = (uint8_t)(current & 0xFF);
    }

#if defined(STM32H7) && defined(HAL_FDCAN_MODULE_ENABLED)
    /* Send via FDCAN (STM32H7) */
    FDCAN_HandleTypeDef* hfdcan = (FDCAN_HandleTypeDef*)can_handle;

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier = RM_CAN_ID_CMD_1_4;  /* Adjust based on motor group */
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }
#elif defined(STM32F4) && defined(HAL_CAN_MODULE_ENABLED)
    /* Send via CAN (STM32F4) */
    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef*)can_handle;

    CAN_TxHeaderTypeDef tx_header = {0};
    tx_header.StdId = RM_CAN_ID_CMD_1_4;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
        return MAVLINK_DEVICE_ERROR_COMM_ERROR;
    }
#endif

    return MAVLINK_DEVICE_ERROR_NONE;
}
