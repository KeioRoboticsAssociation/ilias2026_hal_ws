/**
 * @file dc_controller.h
 * @brief DC motor controller for H753 platform
 */

#ifndef DC_CONTROLLER_H
#define DC_CONTROLLER_H

#include "motor_interface.h"
#include "../hal/hardware_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * DC Motor Controller Private Data
 * ============================================================================ */
typedef struct {
    dc_motor_config_t config;
    uint8_t timer_id;
    uint32_t channel;
    uint32_t last_watchdog_reset;
    bool watchdog_expired;

    // PID state
    float speed_integral;
    float speed_last_error;
    float position_integral;
    float position_last_error;

    // Current duty cycle
    float current_duty_cycle;
} dc_motor_private_t;

/* ============================================================================
 * DC Motor Controller API
 * ============================================================================ */

/**
 * @brief Create a new DC motor controller
 */
error_code_t dc_motor_controller_create(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const dc_motor_config_t* config,
    motor_controller_t* controller,
    dc_motor_private_t* private_data);

/**
 * @brief Get the DC motor controller virtual function table
 */
const motor_vtable_t* dc_motor_controller_get_vtable(void);

#ifdef __cplusplus
}
#endif

#endif /* DC_CONTROLLER_H */
