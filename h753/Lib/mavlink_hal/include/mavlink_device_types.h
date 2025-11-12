/**
 * @file mavlink_device_types.h
 * @brief MAVLink device type definitions and common structures
 *
 * Unified device type system with tagged unions and compile-time type safety.
 * Supports motors, servos, sensors with minimal memory overhead.
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_DEVICE_TYPES_H
#define MAVLINK_DEVICE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*  DEVICE TYPE ENUMERATION                                                   */
/* ========================================================================== */

/**
 * @brief Device type enumeration
 *
 * Defines all supported device types in the unified interface.
 */
typedef enum {
    MAVLINK_DEVICE_TYPE_NONE = 0,

    /* Motor Types */
    MAVLINK_DEVICE_TYPE_SERVO = 1,
    MAVLINK_DEVICE_TYPE_DC_MOTOR = 2,
    MAVLINK_DEVICE_TYPE_BLDC_MOTOR = 3,
    MAVLINK_DEVICE_TYPE_STEPPER = 4,
    MAVLINK_DEVICE_TYPE_ROBOMASTER = 5,
    MAVLINK_DEVICE_TYPE_RS485_MOTOR = 6,

    /* Sensor Types */
    MAVLINK_DEVICE_TYPE_ENCODER = 100,
    MAVLINK_DEVICE_TYPE_IMU = 101,
    MAVLINK_DEVICE_TYPE_GPS = 102,
    MAVLINK_DEVICE_TYPE_ANALOG_SENSOR = 103,
    MAVLINK_DEVICE_TYPE_DIGITAL_IO = 104,

    /* Special Types */
    MAVLINK_DEVICE_TYPE_CUSTOM = 255
} mavlink_device_type_t;

/* ========================================================================== */
/*  DEVICE STATE                                                              */
/* ========================================================================== */

/**
 * @brief Device state enumeration
 */
typedef enum {
    MAVLINK_DEVICE_STATE_UNINITIALIZED = 0,
    MAVLINK_DEVICE_STATE_INITIALIZING = 1,
    MAVLINK_DEVICE_STATE_READY = 2,
    MAVLINK_DEVICE_STATE_ACTIVE = 3,
    MAVLINK_DEVICE_STATE_ERROR = 4,
    MAVLINK_DEVICE_STATE_DISABLED = 5,
    MAVLINK_DEVICE_STATE_CALIBRATING = 6
} mavlink_device_state_t;

/* ========================================================================== */
/*  CONTROL MODES                                                             */
/* ========================================================================== */

/**
 * @brief Generic control mode enumeration
 */
typedef enum {
    MAVLINK_CONTROL_MODE_POSITION = 0,
    MAVLINK_CONTROL_MODE_VELOCITY = 1,
    MAVLINK_CONTROL_MODE_CURRENT = 2,
    MAVLINK_CONTROL_MODE_DUTY_CYCLE = 3,
    MAVLINK_CONTROL_MODE_TORQUE = 4,
    MAVLINK_CONTROL_MODE_DISABLED = 255
} mavlink_control_mode_t;

/* ========================================================================== */
/*  ERROR CODES                                                               */
/* ========================================================================== */

/**
 * @brief Device error codes
 */
typedef enum {
    MAVLINK_DEVICE_ERROR_NONE = 0,
    MAVLINK_DEVICE_ERROR_NOT_INITIALIZED = -1,
    MAVLINK_DEVICE_ERROR_INVALID_PARAM = -2,
    MAVLINK_DEVICE_ERROR_HARDWARE_FAULT = -3,
    MAVLINK_DEVICE_ERROR_TIMEOUT = -4,
    MAVLINK_DEVICE_ERROR_OVERFLOW = -5,
    MAVLINK_DEVICE_ERROR_UNDERFLOW = -6,
    MAVLINK_DEVICE_ERROR_CALIBRATION_FAILED = -7,
    MAVLINK_DEVICE_ERROR_SAFETY_VIOLATION = -8,
    MAVLINK_DEVICE_ERROR_COMM_ERROR = -9,
    MAVLINK_DEVICE_ERROR_UNSUPPORTED = -10
} mavlink_device_error_t;

/* ========================================================================== */
/*  DEVICE CAPABILITIES                                                       */
/* ========================================================================== */

/**
 * @brief Device capability flags (bitfield)
 */
typedef enum {
    MAVLINK_DEVICE_CAP_POSITION_CONTROL = (1 << 0),
    MAVLINK_DEVICE_CAP_VELOCITY_CONTROL = (1 << 1),
    MAVLINK_DEVICE_CAP_CURRENT_CONTROL = (1 << 2),
    MAVLINK_DEVICE_CAP_TORQUE_CONTROL = (1 << 3),
    MAVLINK_DEVICE_CAP_FEEDBACK = (1 << 4),
    MAVLINK_DEVICE_CAP_ENCODER = (1 << 5),
    MAVLINK_DEVICE_CAP_CALIBRATION = (1 << 6),
    MAVLINK_DEVICE_CAP_SELF_TEST = (1 << 7),
    MAVLINK_DEVICE_CAP_SAFETY_LIMITS = (1 << 8),
    MAVLINK_DEVICE_CAP_FAILSAFE = (1 << 9),
    MAVLINK_DEVICE_CAP_TELEMETRY = (1 << 10),
    MAVLINK_DEVICE_CAP_PARAMETERS = (1 << 11)
} mavlink_device_capability_t;

/* ========================================================================== */
/*  DEVICE IDENTIFICATION                                                     */
/* ========================================================================== */

/**
 * @brief Device identification structure
 */
typedef struct {
    uint8_t id;                          /**< Unique device ID */
    mavlink_device_type_t type;          /**< Device type */
    char name[32];                       /**< Human-readable name */
    uint32_t capabilities;               /**< Capability flags (bitfield) */
    uint8_t hardware_version;            /**< Hardware version */
    uint8_t firmware_version;            /**< Firmware/driver version */
} mavlink_device_id_t;

/* ========================================================================== */
/*  DEVICE STATUS                                                             */
/* ========================================================================== */

/**
 * @brief Device status structure
 */
typedef struct {
    mavlink_device_state_t state;        /**< Current state */
    mavlink_device_error_t error;        /**< Last error code */
    mavlink_control_mode_t control_mode; /**< Active control mode */
    uint32_t uptime_ms;                  /**< Milliseconds since init */
    uint32_t command_count;              /**< Total commands received */
    uint32_t error_count;                /**< Total errors */
    bool enabled;                        /**< Device enabled flag */
} mavlink_device_status_t;

/* ========================================================================== */
/*  DEVICE FEEDBACK                                                           */
/* ========================================================================== */

/**
 * @brief Generic device feedback structure
 *
 * Tagged union for type-safe device feedback.
 */
typedef struct {
    mavlink_device_type_t type;          /**< Device type tag */

    union {
        /* Motor feedback */
        struct {
            float position;              /**< Position (rad or deg) */
            float velocity;              /**< Velocity (rad/s or RPS) */
            float current;               /**< Current (Amps) */
            float torque;                /**< Torque (N·m) */
            float temperature;           /**< Temperature (°C) */
        } motor;

        /* Sensor feedback */
        struct {
            float value;                 /**< Primary sensor value */
            float values[16];            /**< Multi-axis values */
            uint8_t value_count;         /**< Number of valid values */
            bool valid;                  /**< Data validity flag */
        } sensor;

        /* Raw data (for custom devices) */
        uint8_t raw[64];
    } data;
} mavlink_device_feedback_t;

/* ========================================================================== */
/*  DEVICE COMMAND                                                            */
/* ========================================================================== */

/**
 * @brief Generic device command structure
 *
 * Tagged union for type-safe device commands.
 */
typedef struct {
    mavlink_device_type_t type;          /**< Device type tag */
    mavlink_control_mode_t mode;         /**< Control mode */

    union {
        /* Position command */
        struct {
            float target;                /**< Target position */
            float velocity_limit;        /**< Max velocity (optional) */
            float acceleration_limit;    /**< Max acceleration (optional) */
        } position;

        /* Velocity command */
        struct {
            float target;                /**< Target velocity */
            float acceleration_limit;    /**< Max acceleration (optional) */
        } velocity;

        /* Current/torque command */
        struct {
            float target;                /**< Target current/torque */
        } current;

        /* Duty cycle command */
        struct {
            float duty;                  /**< Duty cycle (-1.0 to +1.0) */
        } duty_cycle;

        /* Raw command (for custom devices) */
        uint8_t raw[32];
    } data;
} mavlink_device_command_t;

/* ========================================================================== */
/*  DEVICE CONFIGURATION                                                      */
/* ========================================================================== */

/**
 * @brief Device limits structure
 */
typedef struct {
    float min_position;                  /**< Minimum position */
    float max_position;                  /**< Maximum position */
    float max_velocity;                  /**< Maximum velocity */
    float max_acceleration;              /**< Maximum acceleration */
    float max_current;                   /**< Maximum current (A) */
    float max_temperature;               /**< Maximum temperature (°C) */
} mavlink_device_limits_t;

/**
 * @brief Device failsafe configuration
 */
typedef enum {
    MAVLINK_FAILSAFE_HOLD = 0,           /**< Hold last position */
    MAVLINK_FAILSAFE_NEUTRAL = 1,        /**< Move to neutral position */
    MAVLINK_FAILSAFE_DISABLE = 2,        /**< Disable device */
    MAVLINK_FAILSAFE_CUSTOM = 3          /**< Custom failsafe behavior */
} mavlink_failsafe_action_t;

typedef struct {
    mavlink_failsafe_action_t action;    /**< Failsafe action */
    uint32_t timeout_ms;                 /**< Watchdog timeout */
    float custom_value;                  /**< Custom failsafe value */
} mavlink_device_failsafe_t;

/**
 * @brief PID controller configuration
 */
typedef struct {
    float kp;                            /**< Proportional gain */
    float ki;                            /**< Integral gain */
    float kd;                            /**< Derivative gain */
    float max_integral;                  /**< Integral windup limit */
    float max_output;                    /**< Output saturation limit */
} mavlink_pid_config_t;

/**
 * @brief Generic device configuration
 *
 * Tagged union for device-specific configuration.
 */
typedef struct {
    mavlink_device_type_t type;          /**< Device type tag */
    mavlink_device_limits_t limits;      /**< Device limits */
    mavlink_device_failsafe_t failsafe;  /**< Failsafe config */

    union {
        /* Servo configuration */
        struct {
            uint16_t min_pulse_us;       /**< Minimum pulse width */
            uint16_t max_pulse_us;       /**< Maximum pulse width */
            float neutral_angle;         /**< Neutral position */
        } servo;

        /* Motor configuration */
        struct {
            mavlink_pid_config_t pid_position;
            mavlink_pid_config_t pid_velocity;
            mavlink_pid_config_t pid_current;
            bool has_encoder;
            uint16_t encoder_ppr;        /**< Pulses per revolution */
        } motor;

        /* Sensor configuration */
        struct {
            float scale_factor;          /**< Scaling factor */
            float offset;                /**< Offset value */
            uint16_t sample_rate_hz;     /**< Sample rate */
            uint8_t filter_type;         /**< Digital filter type */
        } sensor;

        /* Raw config (for custom devices) */
        uint8_t raw[128];
    } config;
} mavlink_device_config_t;

/* ========================================================================== */
/*  DEVICE TELEMETRY                                                          */
/* ========================================================================== */

/**
 * @brief Device telemetry structure
 *
 * Periodic telemetry data for MAVLink transmission.
 */
typedef struct {
    uint32_t timestamp_ms;               /**< Timestamp */
    mavlink_device_feedback_t feedback;  /**< Current feedback */
    mavlink_device_status_t status;      /**< Current status */
    float health;                        /**< Health metric (0.0-1.0) */
} mavlink_device_telemetry_t;

/* ========================================================================== */
/*  DEVICE DIAGNOSTICS                                                        */
/* ========================================================================== */

/**
 * @brief Device diagnostic result
 */
typedef enum {
    MAVLINK_DIAG_RESULT_PASS = 0,
    MAVLINK_DIAG_RESULT_FAIL = 1,
    MAVLINK_DIAG_RESULT_WARNING = 2,
    MAVLINK_DIAG_RESULT_SKIPPED = 3
} mavlink_diag_result_t;

typedef struct {
    mavlink_diag_result_t result;        /**< Test result */
    char message[64];                    /**< Diagnostic message */
    uint32_t execution_time_us;          /**< Test execution time */
} mavlink_device_diagnostic_t;

/* ========================================================================== */
/*  UTILITY FUNCTIONS                                                         */
/* ========================================================================== */

/**
 * @brief Get device type name string
 * @param type Device type
 * @return Device type name
 */
const char* mavlink_device_type_name(mavlink_device_type_t type);

/**
 * @brief Get device state name string
 * @param state Device state
 * @return State name
 */
const char* mavlink_device_state_name(mavlink_device_state_t state);

/**
 * @brief Get control mode name string
 * @param mode Control mode
 * @return Mode name
 */
const char* mavlink_device_control_mode_name(mavlink_control_mode_t mode);

/**
 * @brief Check if device type is a motor
 * @param type Device type
 * @return true if motor type
 */
static inline bool mavlink_device_is_motor(mavlink_device_type_t type) {
    return type >= MAVLINK_DEVICE_TYPE_SERVO && type <= MAVLINK_DEVICE_TYPE_RS485_MOTOR;
}

/**
 * @brief Check if device type is a sensor
 * @param type Device type
 * @return true if sensor type
 */
static inline bool mavlink_device_is_sensor(mavlink_device_type_t type) {
    return type >= MAVLINK_DEVICE_TYPE_ENCODER && type <= MAVLINK_DEVICE_TYPE_DIGITAL_IO;
}

/**
 * @brief Check if device has capability
 * @param capabilities Capability flags
 * @param cap Capability to check
 * @return true if device has capability
 */
static inline bool mavlink_device_has_capability(uint32_t capabilities,
                                                  mavlink_device_capability_t cap) {
    return (capabilities & cap) != 0;
}

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_DEVICE_TYPES_H */
