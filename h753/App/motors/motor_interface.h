/**
 * @file motor_interface.h
 * @brief Unified motor interface for H753 platform (C implementation)
 *
 * This file defines the common interface for all motor types,
 * following the F446RE design pattern adapted for C.
 */

#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Motor State Structure
 * ============================================================================ */
typedef struct {
    float current_position;     // Current position (degrees for servo, radians for DC/RM)
    float target_position;      // Target position
    float current_velocity;     // Current velocity (deg/s or rad/s)
    float target_velocity;      // Target velocity
    float current_current;      // Current draw (A)
    float temperature;          // Temperature (C)
    error_code_t status;        // Motor status
    bool enabled;               // Motor enabled state
    uint32_t last_update_time;  // Last update timestamp (ms)
    uint32_t error_count;       // Error counter
    uint32_t timeout_count;     // Timeout counter
} motor_state_t;

/* ============================================================================
 * Motor Command Structure
 * ============================================================================ */
typedef struct {
    uint8_t motor_id;           // Motor ID
    control_mode_t mode;        // Control mode
    float target_value;         // Target value (depends on mode)
    bool enable;                // Enable/disable motor
    uint32_t timestamp;         // Command timestamp (ms)
} motor_command_t;

/* ============================================================================
 * Motor Controller Interface (function pointers for polymorphism)
 * ============================================================================ */

// Forward declaration
typedef struct motor_controller motor_controller_t;

// Motor controller function pointer types
typedef error_code_t (*motor_init_fn)(motor_controller_t* controller);
typedef error_code_t (*motor_update_fn)(motor_controller_t* controller, float delta_time);
typedef error_code_t (*motor_set_command_fn)(motor_controller_t* controller, const motor_command_t* cmd);
typedef error_code_t (*motor_set_enabled_fn)(motor_controller_t* controller, bool enabled);
typedef void (*motor_emergency_stop_fn)(motor_controller_t* controller);
typedef void (*motor_reset_watchdog_fn)(motor_controller_t* controller);
typedef error_code_t (*motor_self_test_fn)(motor_controller_t* controller);
typedef motor_state_t (*motor_get_state_fn)(const motor_controller_t* controller);
typedef uint8_t (*motor_get_id_fn)(const motor_controller_t* controller);

/**
 * @brief Virtual function table for motor controller
 *
 * This enables polymorphism in C using function pointers.
 */
typedef struct {
    motor_init_fn initialize;
    motor_update_fn update;
    motor_set_command_fn set_command;
    motor_set_enabled_fn set_enabled;
    motor_emergency_stop_fn emergency_stop;
    motor_reset_watchdog_fn reset_watchdog;
    motor_self_test_fn self_test;
    motor_get_state_fn get_state;
    motor_get_id_fn get_id;
} motor_vtable_t;

/**
 * @brief Base motor controller structure
 *
 * All specific motor controllers (servo, DC, RoboMaster) will embed this
 * as their first member to enable polymorphism.
 */
struct motor_controller {
    const motor_vtable_t* vtable;  // Virtual function table
    uint8_t id;                     // Motor ID
    motor_type_t type;              // Motor type
    motor_state_t state;            // Current state
    void* hw_handle;                // Hardware handle (TIM_HandleTypeDef*, etc.)
    void* private_data;             // Private data for specific motor type
};

/* ============================================================================
 * Motor Controller Interface Functions (inline wrappers)
 * ============================================================================ */

/**
 * @brief Initialize motor controller
 */
static inline error_code_t motor_initialize(motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->initialize) {
        return controller->vtable->initialize(controller);
    }
    return ERROR_INVALID_PARAMETER;
}

/**
 * @brief Update motor controller (periodic call)
 */
static inline error_code_t motor_update(motor_controller_t* controller, float delta_time) {
    if (controller && controller->vtable && controller->vtable->update) {
        return controller->vtable->update(controller, delta_time);
    }
    return ERROR_INVALID_PARAMETER;
}

/**
 * @brief Send command to motor
 */
static inline error_code_t motor_set_command(motor_controller_t* controller, const motor_command_t* cmd) {
    if (controller && controller->vtable && controller->vtable->set_command) {
        return controller->vtable->set_command(controller, cmd);
    }
    return ERROR_INVALID_PARAMETER;
}

/**
 * @brief Enable/disable motor
 */
static inline error_code_t motor_set_enabled(motor_controller_t* controller, bool enabled) {
    if (controller && controller->vtable && controller->vtable->set_enabled) {
        return controller->vtable->set_enabled(controller, enabled);
    }
    return ERROR_INVALID_PARAMETER;
}

/**
 * @brief Emergency stop motor
 */
static inline void motor_emergency_stop(motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->emergency_stop) {
        controller->vtable->emergency_stop(controller);
    }
}

/**
 * @brief Reset watchdog timer
 */
static inline void motor_reset_watchdog(motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->reset_watchdog) {
        controller->vtable->reset_watchdog(controller);
    }
}

/**
 * @brief Run self-test
 */
static inline error_code_t motor_self_test(motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->self_test) {
        return controller->vtable->self_test(controller);
    }
    return ERROR_INVALID_PARAMETER;
}

/**
 * @brief Get motor state
 */
static inline motor_state_t motor_get_state(const motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->get_state) {
        return controller->vtable->get_state(controller);
    }
    motor_state_t empty_state = {0};
    empty_state.status = ERROR_INVALID_PARAMETER;
    return empty_state;
}

/**
 * @brief Get motor ID
 */
static inline uint8_t motor_get_id(const motor_controller_t* controller) {
    if (controller && controller->vtable && controller->vtable->get_id) {
        return controller->vtable->get_id(controller);
    }
    return 0;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Constrain value within range
 */
static inline float constrain_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Check if value is within range
 */
static inline bool is_in_range(float value, float min, float max) {
    return (value >= min) && (value <= max);
}

/**
 * @brief Calculate PID output
 */
static inline float calculate_pid(
    float error,
    float* integral,
    float* last_error,
    float kp, float ki, float kd,
    float max_integral, float max_output,
    float delta_time)
{
    // Proportional
    float p = kp * error;

    // Integral with anti-windup
    *integral += ki * error * delta_time;
    *integral = constrain_float(*integral, -max_integral, max_integral);

    // Derivative
    float d = (delta_time > 0.0001f) ? (kd * (error - *last_error) / delta_time) : 0.0f;
    *last_error = error;

    // Output with limiting
    float output = p + *integral + d;
    return constrain_float(output, -max_output, max_output);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_INTERFACE_H */
