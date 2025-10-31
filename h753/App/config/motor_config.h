/**
 * @file motor_config.h
 * @brief Motor configuration system for H753 platform (C implementation)
 *
 * This file defines compile-time motor configurations following F446RE design principles.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Error Codes - unified error handling system
 * ============================================================================ */
typedef enum {
    ERROR_OK = 0,
    ERROR_NOT_INITIALIZED,
    ERROR_ALREADY_INITIALIZED,
    ERROR_HARDWARE_ERROR,
    ERROR_TIMEOUT,
    ERROR_CONFIG_ERROR,
    ERROR_SAFETY_VIOLATION,
    ERROR_INVALID_PARAMETER,
    ERROR_RESOURCE_EXHAUSTED,
    ERROR_COMM_ERROR
} error_code_t;

/* ============================================================================
 * Motor Types and Control Modes
 * ============================================================================ */
typedef enum {
    MOTOR_TYPE_SERVO = 0,
    MOTOR_TYPE_DC = 1,
    MOTOR_TYPE_ROBOMASTER = 2
} motor_type_t;

typedef enum {
    CONTROL_MODE_POSITION = 0,
    CONTROL_MODE_VELOCITY = 1,
    CONTROL_MODE_CURRENT = 2,
    CONTROL_MODE_DUTY_CYCLE = 3,
    CONTROL_MODE_DISABLED = 255
} control_mode_t;

/* ============================================================================
 * Servo Configuration
 * ============================================================================ */
typedef enum {
    SERVO_FAILSAFE_NEUTRAL = 0,
    SERVO_FAILSAFE_HOLD = 1,
    SERVO_FAILSAFE_DISABLE = 2
} servo_failsafe_t;

typedef struct {
    uint8_t id;
    float initial_offset;
    float min_angle;
    float max_angle;
    uint16_t pulse_min_us;
    uint16_t pulse_max_us;
    uint16_t pulse_neutral_us;
    bool direction_inverted;
    float max_velocity_deg_per_s;
    float max_acceleration_deg_per_s2;
    uint32_t watchdog_timeout_ms;
    float startup_angle_deg;
    bool start_disabled;
    servo_failsafe_t failsafe_behavior;
} servo_config_t;

/* ============================================================================
 * DC Motor Configuration
 * ============================================================================ */
typedef struct {
    uint8_t id;

    // Speed PID
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_max_integral;
    float speed_max_output;

    // Position PID
    float position_kp;
    float position_ki;
    float position_kd;
    float position_max_integral;
    float position_max_output;

    // Limits
    float max_speed_rad_s;
    float max_acceleration_rad_s2;
    float position_limit_min_rad;
    float position_limit_max_rad;
    bool use_position_limits;

    // Control
    uint32_t watchdog_timeout_ms;
    uint32_t control_period_ms;
    bool direction_inverted;
} dc_motor_config_t;

/* ============================================================================
 * RoboMaster Motor Configuration
 * ============================================================================ */
typedef struct {
    uint8_t id;
    uint16_t can_id;  // Changed to uint16_t to accommodate CAN IDs > 255

    // Angle PID
    float angle_kp;
    float angle_ki;
    float angle_kd;

    // Speed PID
    float speed_kp;
    float speed_ki;
    float speed_kd;

    // Limits
    float max_speed_rad_s;
    float max_acceleration_rad_s2;
    uint32_t watchdog_timeout_ms;
} robomaster_config_t;

/* ============================================================================
 * Hardware Timer Configuration
 * ============================================================================ */
typedef struct {
    uint8_t timer_id;  // TIM1=1, TIM2=2, etc.
    uint32_t channel;  // TIM_CHANNEL_1, TIM_CHANNEL_2, etc.
} timer_config_t;

/* ============================================================================
 * Motor Instance Mapping
 * ============================================================================ */
typedef struct {
    uint8_t id;
    motor_type_t type;
    uint8_t timer_id;
    uint32_t channel;
    bool enabled;
} motor_instance_t;

/* ============================================================================
 * System Configuration
 * ============================================================================ */
#define MAX_SERVOS          4
#define MAX_DC_MOTORS       2
#define MAX_ROBOMASTER      2
#define MAX_MOTORS          (MAX_SERVOS + MAX_DC_MOTORS + MAX_ROBOMASTER)

#define MAVLINK_SYSTEM_ID   1
#define MAVLINK_COMPONENT_ID 1
#define MAIN_LOOP_FREQ_HZ   100
#define CONTROL_LOOP_FREQ_HZ 100

/* ============================================================================
 * Default Motor Configurations
 * ============================================================================ */

// Servo configurations (4 servos on TIM1, TIM2)
static const servo_config_t SERVO_CONFIGS[MAX_SERVOS] = {
    {
        .id = 1,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 120.0f,
        .max_acceleration_deg_per_s2 = 240.0f,
        .watchdog_timeout_ms = 500,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = SERVO_FAILSAFE_NEUTRAL
    },
    {
        .id = 2,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 120.0f,
        .max_acceleration_deg_per_s2 = 240.0f,
        .watchdog_timeout_ms = 500,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = SERVO_FAILSAFE_NEUTRAL
    },
    {
        .id = 3,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 120.0f,
        .max_acceleration_deg_per_s2 = 240.0f,
        .watchdog_timeout_ms = 500,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = SERVO_FAILSAFE_NEUTRAL
    },
    {
        .id = 4,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 120.0f,
        .max_acceleration_deg_per_s2 = 240.0f,
        .watchdog_timeout_ms = 500,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = SERVO_FAILSAFE_NEUTRAL
    }
};

// DC Motor configurations
static const dc_motor_config_t DC_MOTOR_CONFIGS[MAX_DC_MOTORS] = {
    {
        .id = 10,
        .speed_kp = 0.1f,
        .speed_ki = 0.05f,
        .speed_kd = 0.0f,
        .speed_max_integral = 0.3f,
        .speed_max_output = 1.0f,
        .position_kp = 0.5f,
        .position_ki = 0.0f,
        .position_kd = 0.1f,
        .position_max_integral = 100.0f,
        .position_max_output = 10.0f,
        .max_speed_rad_s = 15.0f,
        .max_acceleration_rad_s2 = 50.0f,
        .position_limit_min_rad = -314.159f,
        .position_limit_max_rad = 314.159f,
        .use_position_limits = true,
        .watchdog_timeout_ms = 1000,
        .control_period_ms = 10,
        .direction_inverted = false
    },
    {
        .id = 11,
        .speed_kp = 0.1f,
        .speed_ki = 0.05f,
        .speed_kd = 0.0f,
        .speed_max_integral = 0.3f,
        .speed_max_output = 1.0f,
        .position_kp = 0.5f,
        .position_ki = 0.0f,
        .position_kd = 0.1f,
        .position_max_integral = 100.0f,
        .position_max_output = 10.0f,
        .max_speed_rad_s = 15.0f,
        .max_acceleration_rad_s2 = 50.0f,
        .position_limit_min_rad = -314.159f,
        .position_limit_max_rad = 314.159f,
        .use_position_limits = true,
        .watchdog_timeout_ms = 1000,
        .control_period_ms = 10,
        .direction_inverted = false
    }
};

// RoboMaster configurations
static const robomaster_config_t ROBOMASTER_CONFIGS[MAX_ROBOMASTER] = {
    {
        // GM6020 Motor #1 (Gimbal motor, voltage control)
        .id = 20,
        .can_id = 0x205,         // GM6020 motor 1 feedback ID
        .angle_kp = 0.1f,
        .angle_ki = 0.0f,
        .angle_kd = 0.0f,
        .speed_kp = 50.0f,       // Higher gain for GM6020 voltage control (triggers GM6020 detection)
        .speed_ki = 0.1f,
        .speed_kd = 0.0f,
        .max_speed_rad_s = 10.0f,
        .max_acceleration_rad_s2 = 30.0f,
        .watchdog_timeout_ms = 500
    },
    {
        // M3508 Motor #1 (Drive motor, current control) - for future use
        .id = 21,
        .can_id = 0x201,         // M3508 motor 1 feedback ID
        .angle_kp = 0.1f,
        .angle_ki = 0.0f,
        .angle_kd = 0.0f,
        .speed_kp = 0.5f,        // Lower gain for M3508 current control
        .speed_ki = 0.1f,
        .speed_kd = 0.0f,
        .max_speed_rad_s = 10.0f,
        .max_acceleration_rad_s2 = 30.0f,
        .watchdog_timeout_ms = 500
    }
};

/* ============================================================================
 * Motor Instance Mapping to Hardware
 * ============================================================================ */

// Note: TIM_CHANNEL_1 = 0, TIM_CHANNEL_2 = 4, TIM_CHANNEL_3 = 8, TIM_CHANNEL_4 = 12
// These values match STM32 HAL definitions

static const motor_instance_t MOTOR_INSTANCES[MAX_MOTORS] = {
    // Servos on TIM1
    {.id = 1, .type = MOTOR_TYPE_SERVO, .timer_id = 1, .channel = 0, .enabled = true},   // TIM1_CH1
    {.id = 2, .type = MOTOR_TYPE_SERVO, .timer_id = 1, .channel = 4, .enabled = true},   // TIM1_CH2

    // Servos on TIM2
    {.id = 3, .type = MOTOR_TYPE_SERVO, .timer_id = 2, .channel = 0, .enabled = true},   // TIM2_CH1
    {.id = 4, .type = MOTOR_TYPE_SERVO, .timer_id = 2, .channel = 4, .enabled = true},   // TIM2_CH2

    // DC Motors on TIM3
    {.id = 10, .type = MOTOR_TYPE_DC, .timer_id = 3, .channel = 0, .enabled = true},     // TIM3_CH1
    {.id = 11, .type = MOTOR_TYPE_DC, .timer_id = 3, .channel = 4, .enabled = true},     // TIM3_CH2

    // RoboMaster on CAN
    {.id = 20, .type = MOTOR_TYPE_ROBOMASTER, .timer_id = 0, .channel = 0, .enabled = true},
    {.id = 21, .type = MOTOR_TYPE_ROBOMASTER, .timer_id = 0, .channel = 0, .enabled = true}
};

/* ============================================================================
 * Configuration Helper Functions
 * ============================================================================ */

/**
 * @brief Get servo configuration by ID
 * @param id Servo ID
 * @return Pointer to configuration or NULL if not found
 */
static inline const servo_config_t* get_servo_config(uint8_t id) {
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (SERVO_CONFIGS[i].id == id) {
            return &SERVO_CONFIGS[i];
        }
    }
    return NULL;
}

/**
 * @brief Get DC motor configuration by ID
 * @param id DC motor ID
 * @return Pointer to configuration or NULL if not found
 */
static inline const dc_motor_config_t* get_dc_motor_config(uint8_t id) {
    for (int i = 0; i < MAX_DC_MOTORS; i++) {
        if (DC_MOTOR_CONFIGS[i].id == id) {
            return &DC_MOTOR_CONFIGS[i];
        }
    }
    return NULL;
}

/**
 * @brief Get RoboMaster configuration by ID
 * @param id RoboMaster motor ID
 * @return Pointer to configuration or NULL if not found
 */
static inline const robomaster_config_t* get_robomaster_config(uint8_t id) {
    for (int i = 0; i < MAX_ROBOMASTER; i++) {
        if (ROBOMASTER_CONFIGS[i].id == id) {
            return &ROBOMASTER_CONFIGS[i];
        }
    }
    return NULL;
}

/**
 * @brief Get motor instance by ID
 * @param id Motor ID
 * @return Pointer to instance or NULL if not found
 */
static inline const motor_instance_t* get_motor_instance(uint8_t id) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (MOTOR_INSTANCES[i].id == id) {
            return &MOTOR_INSTANCES[i];
        }
    }
    return NULL;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONFIG_H */
