/**
 * @file robomaster_controller.h
 * @brief RoboMaster motor controller for H753 platform
 */

#ifndef ROBOMASTER_CONTROLLER_H
#define ROBOMASTER_CONTROLLER_H

#include "motor_interface.h"
#include "../hal/hardware_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * RoboMaster Motor Type Detection
 * ============================================================================ */
typedef enum {
    RM_TYPE_M3508,   // M3508 or M2006 (current control, -16384 to +16384)
    RM_TYPE_GM6020   // GM6020 (voltage control, -30000 to +30000)
} rm_motor_type_t;

/* ============================================================================
 * RoboMaster Controller Private Data
 * ============================================================================ */
typedef struct {
    robomaster_config_t config;
    uint32_t last_watchdog_reset;
    bool watchdog_expired;
    uint32_t last_can_rx;

    // Motor type (auto-detected based on CAN ID and gains)
    rm_motor_type_t motor_type;

    // PID state
    float angle_integral;
    float angle_last_error;
    float speed_integral;
    float speed_last_error;

    // Feedback from motor
    int16_t encoder_angle;
    int16_t encoder_speed;
    int16_t motor_current;
} robomaster_private_t;

/* ============================================================================
 * RoboMaster Controller API
 * ============================================================================ */

/**
 * @brief Create a new RoboMaster controller
 */
error_code_t robomaster_controller_create(
    uint8_t id,
    const robomaster_config_t* config,
    motor_controller_t* controller,
    robomaster_private_t* private_data);

/**
 * @brief Get the RoboMaster controller virtual function table
 */
const motor_vtable_t* robomaster_controller_get_vtable(void);

/**
 * @brief Process CAN message from RoboMaster motor
 */
error_code_t robomaster_process_can_message(
    motor_controller_t* controller,
    uint32_t can_id,
    const uint8_t* data,
    uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* ROBOMASTER_CONTROLLER_H */
