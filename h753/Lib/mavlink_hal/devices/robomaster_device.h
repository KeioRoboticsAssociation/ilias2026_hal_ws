/**
 * @file robomaster_device.h
 * @brief RoboMaster motor device implementation using unified device interface
 *
 * Provides RoboMaster motor control via CAN bus:
 * - GM6020 (gimbal motor)
 * - M3508 (chassis motor with C620 ESC)
 * - M2006 (small chassis motor with C610 ESC)
 *
 * Features:
 * - Position control (angle mode)
 * - Velocity control (speed mode)
 * - Current control
 * - Built-in encoder feedback
 * - Dual PID control (speed + angle)
 * - CAN communication (up to 8 motors per CAN bus)
 * - Runtime PID parameter tuning
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_ROBOMASTER_DEVICE_H
#define MAVLINK_ROBOMASTER_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../include/mavlink_device_interface.h"
#include "dc_motor_device.h"  /* Reuse PID controller */

/* ========================================================================== */
/*  ROBOMASTER MOTOR TYPES                                                    */
/* ========================================================================== */

typedef enum {
    ROBOMASTER_TYPE_GM6020 = 0,  /**< GM6020 gimbal motor */
    ROBOMASTER_TYPE_M3508,       /**< M3508 chassis motor with C620 ESC */
    ROBOMASTER_TYPE_M2006,       /**< M2006 small chassis motor with C610 ESC */
} robomaster_motor_type_t;

/* ========================================================================== */
/*  ROBOMASTER FEEDBACK DATA                                                  */
/* ========================================================================== */

/**
 * @brief RoboMaster motor feedback from CAN
 *
 * This structure matches the CAN feedback format from RoboMaster motors.
 */
typedef struct {
    uint16_t angle_raw;          /**< Raw angle (0-8191) */
    int16_t velocity_rpm;        /**< Velocity in RPM */
    int16_t current_raw;         /**< Current in raw units */
    uint8_t temperature;         /**< Temperature in degrees C */

    /* Calculated values */
    float angle_rad;             /**< Angle in radians */
    float velocity_rad_s;        /**< Velocity in rad/s */
    float current_amps;          /**< Current in Amps */
} robomaster_feedback_t;

/* ========================================================================== */
/*  ROBOMASTER PRIVATE DATA                                                   */
/* ========================================================================== */

/**
 * @brief Platform-specific RoboMaster motor data
 */
typedef struct {
    /* Hardware resources */
    void* can_handle;            /**< CAN handle (e.g., FDCAN_HandleTypeDef*) */
    uint8_t can_id;              /**< CAN ID (0x201-0x208 or 0x205-0x20B for GM6020) */
    robomaster_motor_type_t motor_type;

    /* Feedback data */
    robomaster_feedback_t feedback;
    uint32_t last_feedback_time_ms;

    /* Control mode */
    mavlink_control_mode_t control_mode;

    /* PID controllers */
    pid_controller_t speed_pid;  /**< Speed loop PID */
    pid_controller_t angle_pid;  /**< Angle loop PID */

    /* Target values */
    float target_angle;          /**< Target angle in radians */
    float target_velocity;       /**< Target velocity in rad/s */
    float target_current;        /**< Target current in Amps */

    /* Motor limits */
    int16_t max_current_raw;     /**< Maximum current in raw units */

} robomaster_private_data_t;

/* ========================================================================== */
/*  ROBOMASTER DEVICE FACTORY                                                 */
/* ========================================================================== */

/**
 * @brief Create RoboMaster motor device
 *
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param can_handle CAN handle (e.g., FDCAN_HandleTypeDef*)
 * @param can_id CAN ID (0x201-0x208 for M3508/M2006, 0x205-0x20B for GM6020)
 * @param motor_type Motor type (GM6020, M3508, M2006)
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* robomaster_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* can_handle,
    uint8_t can_id,
    robomaster_motor_type_t motor_type
);

/**
 * @brief Get RoboMaster motor vtable
 *
 * @return Pointer to RoboMaster device vtable
 */
const mavlink_device_vtable_t* robomaster_device_get_vtable(void);

/* ========================================================================== */
/*  ROBOMASTER CAN COMMUNICATION                                              */
/* ========================================================================== */

/**
 * @brief Process CAN feedback message
 *
 * Call this from CAN RX interrupt/callback.
 *
 * @param device RoboMaster device
 * @param data CAN data (8 bytes)
 * @return Error code
 */
mavlink_device_error_t robomaster_process_can_feedback(
    mavlink_device_t* device,
    const uint8_t* data
);

/**
 * @brief Send CAN command to motors (batch send)
 *
 * Sends commands to up to 4 motors in a single CAN message.
 * Call this function with device IDs 1-4 or 5-8.
 *
 * @param can_handle CAN handle
 * @param devices Array of 4 RoboMaster devices (NULL if not used)
 * @return Error code
 */
mavlink_device_error_t robomaster_send_can_commands(
    void* can_handle,
    mavlink_device_t* devices[4]
);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_ROBOMASTER_DEVICE_H */
