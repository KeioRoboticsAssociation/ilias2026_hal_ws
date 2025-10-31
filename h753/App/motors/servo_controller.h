/**
 * @file servo_controller.h
 * @brief Servo motor controller for H753 platform
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "motor_interface.h"
#include "../hal/hardware_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Servo Controller Private Data
 * ============================================================================ */
typedef struct {
    servo_config_t config;
    uint8_t timer_id;
    uint32_t channel;
    uint32_t last_watchdog_reset;
    bool watchdog_expired;
    float current_pulse_us;
} servo_private_t;

/* ============================================================================
 * Servo Controller API
 * ============================================================================ */

/**
 * @brief Create a new servo controller
 * @param id Servo ID
 * @param timer_id Timer ID (1-4)
 * @param channel Timer channel (TIM_CHANNEL_1, etc.)
 * @param config Servo configuration
 * @param controller Output pointer to controller (must be pre-allocated)
 * @param private_data Output pointer to private data (must be pre-allocated)
 * @return Error code
 */
error_code_t servo_controller_create(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const servo_config_t* config,
    motor_controller_t* controller,
    servo_private_t* private_data);

/**
 * @brief Get the servo controller virtual function table
 * @return Pointer to vtable
 */
const motor_vtable_t* servo_controller_get_vtable(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROLLER_H */
