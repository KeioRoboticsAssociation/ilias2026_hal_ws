/**
 * @file dc_motor_device.h
 * @brief DC motor device implementation using unified device interface
 *
 * Provides DC motor control with:
 * - Position control (with encoder feedback)
 * - Velocity control (with encoder feedback)
 * - Duty cycle control (open-loop)
 * - PID controllers for position and velocity
 * - Encoder-based feedback
 * - Runtime PID parameter tuning
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_DC_MOTOR_DEVICE_H
#define MAVLINK_DC_MOTOR_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../include/mavlink_device_interface.h"

/* ========================================================================== */
/*  PID CONTROLLER                                                            */
/* ========================================================================== */

/**
 * @brief PID controller structure
 */
typedef struct {
    float kp;                /**< Proportional gain */
    float ki;                /**< Integral gain */
    float kd;                /**< Derivative gain */
    float integral;          /**< Integral accumulator */
    float prev_error;        /**< Previous error for derivative */
    float output_min;        /**< Minimum output */
    float output_max;        /**< Maximum output */
    float integral_max;      /**< Maximum integral windup */
} pid_controller_t;

/* ========================================================================== */
/*  DC MOTOR PRIVATE DATA                                                     */
/* ========================================================================== */

/**
 * @brief Platform-specific DC motor data
 */
typedef struct {
    /* Hardware resources */
    void* timer_handle;         /**< PWM timer handle */
    uint32_t channel_fwd;       /**< Forward PWM channel */
    uint32_t channel_rev;       /**< Reverse PWM channel */
    void* encoder_handle;       /**< Encoder timer handle */
    uint32_t timer_frequency_hz;/**< PWM timer frequency */

    /* Encoder state */
    int32_t encoder_count;      /**< Current encoder count */
    int32_t prev_encoder_count; /**< Previous encoder count */
    float encoder_counts_per_rev; /**< Encoder counts per revolution */
    float gear_ratio;           /**< Gear ratio */

    /* Current state */
    float position;             /**< Position in radians */
    float velocity;             /**< Velocity in rad/s */
    float duty_cycle;           /**< Current duty cycle (-1.0 to 1.0) */
    float current;              /**< Motor current in Amps (if available) */

    /* PID controllers */
    pid_controller_t position_pid;
    pid_controller_t velocity_pid;

    /* Control mode */
    mavlink_control_mode_t control_mode;

    /* Target values */
    float target_position;
    float target_velocity;
    float target_duty_cycle;

} dc_motor_private_data_t;

/* ========================================================================== */
/*  DC MOTOR DEVICE FACTORY                                                   */
/* ========================================================================== */

/**
 * @brief Create DC motor device
 *
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param timer_handle PWM timer handle
 * @param channel_fwd Forward PWM channel
 * @param channel_rev Reverse PWM channel
 * @param encoder_handle Encoder timer handle
 * @param timer_frequency_hz PWM timer frequency
 * @param encoder_counts_per_rev Encoder counts per revolution
 * @param gear_ratio Gear ratio (output/input)
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* dc_motor_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t channel_fwd,
    uint32_t channel_rev,
    void* encoder_handle,
    uint32_t timer_frequency_hz,
    float encoder_counts_per_rev,
    float gear_ratio
);

/**
 * @brief Get DC motor vtable
 *
 * @return Pointer to DC motor device vtable
 */
const mavlink_device_vtable_t* dc_motor_device_get_vtable(void);

/* ========================================================================== */
/*  PID UTILITY FUNCTIONS                                                     */
/* ========================================================================== */

/**
 * @brief Initialize PID controller
 */
void pid_init(pid_controller_t* pid, float kp, float ki, float kd,
              float output_min, float output_max, float integral_max);

/**
 * @brief Update PID controller
 *
 * @param pid PID controller
 * @param setpoint Target value
 * @param measurement Current value
 * @param dt Time step in seconds
 * @return Control output
 */
float pid_update(pid_controller_t* pid, float setpoint, float measurement, float dt);

/**
 * @brief Reset PID controller
 */
void pid_reset(pid_controller_t* pid);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_DC_MOTOR_DEVICE_H */
