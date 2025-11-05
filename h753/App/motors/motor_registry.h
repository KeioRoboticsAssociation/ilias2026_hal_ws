/**
 * @file motor_registry.h
 * @brief Motor registry and factory system for H753
 *
 * This module manages all motor instances and provides a factory
 * to create motors based on configuration.
 */

#ifndef MOTOR_REGISTRY_H
#define MOTOR_REGISTRY_H

#include "motor_interface.h"
#include "servo_controller.h"
#include "dc_controller.h"
#include "robomaster_controller.h"
#include "rs485_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Motor Registry Structures
 * ============================================================================ */

/**
 * @brief Motor entry in the registry
 */
typedef struct {
    motor_controller_t controller;  // Base controller
    motor_type_t type;               // Motor type
    bool allocated;                  // Is this slot allocated?

    // Private data storage (union to save memory)
    union {
        servo_private_t servo;
        dc_motor_private_t dc_motor;
        robomaster_private_t robomaster;
        rs485_private_t rs485;
    } private_data;
} motor_entry_t;

/**
 * @brief Motor registry main structure
 */
typedef struct {
    motor_entry_t motors[MAX_MOTORS];  // All motor slots
    uint8_t motor_count;                // Number of active motors
    bool initialized;                   // Registry initialized
} motor_registry_t;

/* ============================================================================
 * Global Motor Registry
 * ============================================================================ */
extern motor_registry_t g_motor_registry;

/* ============================================================================
 * Motor Registry API
 * ============================================================================ */

/**
 * @brief Initialize the motor registry
 * @return Error code
 */
error_code_t motor_registry_init(void);

/**
 * @brief Create all motors from configuration
 * @return Error code
 */
error_code_t motor_registry_create_all_motors(void);

/**
 * @brief Register a motor in the registry
 * @param id Motor ID
 * @param type Motor type
 * @param controller Pointer to initialized controller
 * @return Error code
 */
error_code_t motor_registry_register(uint8_t id, motor_type_t type, motor_controller_t* controller);

/**
 * @brief Get motor controller by ID
 * @param id Motor ID
 * @return Pointer to controller or NULL if not found
 */
motor_controller_t* motor_registry_get(uint8_t id);

/**
 * @brief Get motor state by ID
 * @param id Motor ID
 * @return Motor state (status will be ERROR_INVALID_PARAMETER if not found)
 */
motor_state_t motor_registry_get_state(uint8_t id);

/**
 * @brief Send command to motor
 * @param id Motor ID
 * @param cmd Command to send
 * @return Error code
 */
error_code_t motor_registry_send_command(uint8_t id, const motor_command_t* cmd);

/**
 * @brief Update all motors (call periodically)
 * @param delta_time Time since last update in seconds
 * @return Error code
 */
error_code_t motor_registry_update_all(float delta_time);

/**
 * @brief Emergency stop all motors
 */
void motor_registry_emergency_stop_all(void);

/**
 * @brief Get number of registered motors
 * @return Motor count
 */
uint8_t motor_registry_get_count(void);

/**
 * @brief Check if registry is initialized
 * @return true if initialized
 */
bool motor_registry_is_initialized(void);

/* ============================================================================
 * Motor Factory API
 * ============================================================================ */

/**
 * @brief Create a servo motor
 * @param id Motor ID
 * @param timer_id Timer ID
 * @param channel Timer channel
 * @param config Servo configuration
 * @param entry Motor entry to populate
 * @return Error code
 */
error_code_t motor_factory_create_servo(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const servo_config_t* config,
    motor_entry_t* entry);

/**
 * @brief Create a DC motor
 * @param id Motor ID
 * @param timer_id Timer ID
 * @param channel Timer channel
 * @param config DC motor configuration
 * @param entry Motor entry to populate
 * @return Error code
 */
error_code_t motor_factory_create_dc_motor(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const dc_motor_config_t* config,
    motor_entry_t* entry);

/**
 * @brief Create a RoboMaster motor
 * @param id Motor ID
 * @param config RoboMaster configuration
 * @param entry Motor entry to populate
 * @return Error code
 */
error_code_t motor_factory_create_robomaster(
    uint8_t id,
    const robomaster_config_t* config,
    motor_entry_t* entry);

/**
 * @brief Create an RS485 motor
 * @param id Motor ID
 * @param config RS485 configuration
 * @param entry Motor entry to populate
 * @return Error code
 */
error_code_t motor_factory_create_rs485(
    uint8_t id,
    const rs485_config_t* config,
    motor_entry_t* entry);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_REGISTRY_H */
