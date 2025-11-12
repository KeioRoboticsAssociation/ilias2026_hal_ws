/**
 * @file mavlink_device_interface.h
 * @brief Unified device interface with polymorphic operations
 *
 * Provides a vtable-based polymorphic interface for all device types.
 * Enables runtime polymorphism in C through function pointers.
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_DEVICE_INTERFACE_H
#define MAVLINK_DEVICE_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_device_types.h"

/* Forward declarations */
struct mavlink_device_s;
typedef struct mavlink_device_s mavlink_device_t;

/* ========================================================================== */
/*  DEVICE OPERATIONS (VTABLE)                                                */
/* ========================================================================== */

/**
 * @brief Device initialization function
 *
 * @param device Device instance
 * @param config Device configuration
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_init_fn)(
    mavlink_device_t* device,
    const mavlink_device_config_t* config
);

/**
 * @brief Device update function (called periodically)
 *
 * @param device Device instance
 * @param dt_ms Delta time in milliseconds
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_update_fn)(
    mavlink_device_t* device,
    uint32_t dt_ms
);

/**
 * @brief Device shutdown function
 *
 * @param device Device instance
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_shutdown_fn)(
    mavlink_device_t* device
);

/**
 * @brief Device enable/disable function
 *
 * @param device Device instance
 * @param enable true to enable, false to disable
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_enable_fn)(
    mavlink_device_t* device,
    bool enable
);

/**
 * @brief Device command function
 *
 * @param device Device instance
 * @param command Command to execute
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_command_fn)(
    mavlink_device_t* device,
    const mavlink_device_command_t* command
);

/**
 * @brief Device feedback function
 *
 * @param device Device instance
 * @param feedback Output feedback structure
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_get_feedback_fn)(
    const mavlink_device_t* device,
    mavlink_device_feedback_t* feedback
);

/**
 * @brief Device status function
 *
 * @param device Device instance
 * @param status Output status structure
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_get_status_fn)(
    const mavlink_device_t* device,
    mavlink_device_status_t* status
);

/**
 * @brief Device parameter set function
 *
 * @param device Device instance
 * @param param_name Parameter name
 * @param value Parameter value
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_set_param_fn)(
    mavlink_device_t* device,
    const char* param_name,
    float value
);

/**
 * @brief Device parameter get function
 *
 * @param device Device instance
 * @param param_name Parameter name
 * @param value Output parameter value
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_get_param_fn)(
    const mavlink_device_t* device,
    const char* param_name,
    float* value
);

/**
 * @brief Device self-test function
 *
 * @param device Device instance
 * @param diagnostic Output diagnostic result
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_self_test_fn)(
    mavlink_device_t* device,
    mavlink_device_diagnostic_t* diagnostic
);

/**
 * @brief Device calibration function
 *
 * @param device Device instance
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
typedef mavlink_device_error_t (*mavlink_device_calibrate_fn)(
    mavlink_device_t* device
);

/**
 * @brief Device virtual function table (vtable)
 *
 * Contains function pointers for all polymorphic device operations.
 * Each device type implements this interface.
 */
typedef struct {
    /* Lifecycle operations */
    mavlink_device_init_fn init;
    mavlink_device_update_fn update;
    mavlink_device_shutdown_fn shutdown;

    /* Control operations */
    mavlink_device_enable_fn enable;
    mavlink_device_command_fn command;

    /* Feedback operations */
    mavlink_device_get_feedback_fn get_feedback;
    mavlink_device_get_status_fn get_status;

    /* Configuration operations */
    mavlink_device_set_param_fn set_param;
    mavlink_device_get_param_fn get_param;

    /* Diagnostic operations */
    mavlink_device_self_test_fn self_test;
    mavlink_device_calibrate_fn calibrate;

} mavlink_device_vtable_t;

/* ========================================================================== */
/*  DEVICE STRUCTURE                                                          */
/* ========================================================================== */

/**
 * @brief Unified device structure
 *
 * Base structure for all device types. Uses vtable for polymorphism.
 */
struct mavlink_device_s {
    /* Device identification */
    mavlink_device_id_t id;

    /* Device status */
    mavlink_device_status_t status;

    /* Device configuration */
    mavlink_device_config_t config;

    /* Virtual function table */
    const mavlink_device_vtable_t* vtable;

    /* Device-specific private data */
    void* private_data;

    /* Watchdog timer */
    uint32_t last_command_time_ms;

    /* Telemetry */
    mavlink_device_telemetry_t telemetry;
    uint32_t telemetry_rate_hz;
    uint32_t last_telemetry_time_ms;
};

/* ========================================================================== */
/*  DEVICE INTERFACE FUNCTIONS                                                */
/* ========================================================================== */

/**
 * @brief Initialize device
 *
 * @param device Device instance
 * @param type Device type
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param vtable Device vtable
 * @param private_data Device-specific private data
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_init(
    mavlink_device_t* device,
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    const mavlink_device_vtable_t* vtable,
    void* private_data
);

/**
 * @brief Update device (call periodically in main loop)
 *
 * @param device Device instance
 * @param dt_ms Delta time in milliseconds
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_update(
    mavlink_device_t* device,
    uint32_t dt_ms
);

/**
 * @brief Shutdown device
 *
 * @param device Device instance
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_shutdown(mavlink_device_t* device);

/**
 * @brief Enable/disable device
 *
 * @param device Device instance
 * @param enable true to enable, false to disable
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_enable(
    mavlink_device_t* device,
    bool enable
);

/**
 * @brief Send command to device
 *
 * @param device Device instance
 * @param command Command to execute
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_send_command(
    mavlink_device_t* device,
    const mavlink_device_command_t* command
);

/**
 * @brief Get device feedback
 *
 * @param device Device instance
 * @param feedback Output feedback structure
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_get_feedback(
    const mavlink_device_t* device,
    mavlink_device_feedback_t* feedback
);

/**
 * @brief Get device status
 *
 * @param device Device instance
 * @param status Output status structure
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_get_status(
    const mavlink_device_t* device,
    mavlink_device_status_t* status
);

/**
 * @brief Set device parameter
 *
 * @param device Device instance
 * @param param_name Parameter name
 * @param value Parameter value
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_set_param(
    mavlink_device_t* device,
    const char* param_name,
    float value
);

/**
 * @brief Get device parameter
 *
 * @param device Device instance
 * @param param_name Parameter name
 * @param value Output parameter value
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_get_param(
    const mavlink_device_t* device,
    const char* param_name,
    float* value
);

/**
 * @brief Run device self-test
 *
 * @param device Device instance
 * @param diagnostic Output diagnostic result
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_self_test(
    mavlink_device_t* device,
    mavlink_device_diagnostic_t* diagnostic
);

/**
 * @brief Run device calibration
 *
 * @param device Device instance
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_calibrate(mavlink_device_t* device);

/**
 * @brief Check if device needs telemetry update
 *
 * @param device Device instance
 * @param current_time_ms Current system time in milliseconds
 * @return true if telemetry should be sent
 */
bool mavlink_device_needs_telemetry(
    const mavlink_device_t* device,
    uint32_t current_time_ms
);

/**
 * @brief Update device telemetry
 *
 * @param device Device instance
 * @param current_time_ms Current system time in milliseconds
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_update_telemetry(
    mavlink_device_t* device,
    uint32_t current_time_ms
);

/**
 * @brief Check watchdog timeout
 *
 * @param device Device instance
 * @param current_time_ms Current system time in milliseconds
 * @return true if watchdog timeout occurred
 */
bool mavlink_device_watchdog_timeout(
    const mavlink_device_t* device,
    uint32_t current_time_ms
);

/**
 * @brief Handle watchdog timeout (execute failsafe)
 *
 * @param device Device instance
 * @return MAVLINK_DEVICE_ERROR_NONE on success, error code otherwise
 */
mavlink_device_error_t mavlink_device_handle_timeout(mavlink_device_t* device);

/* ========================================================================== */
/*  DEVICE FACTORY FUNCTIONS                                                  */
/* ========================================================================== */

/**
 * @brief Create servo device
 *
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific servo data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_create_servo(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data
);

/**
 * @brief Create motor device (DC, BLDC, Stepper, RoboMaster, RS485)
 *
 * @param type Motor type
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific motor data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_create_motor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data
);

/**
 * @brief Create sensor device
 *
 * @param type Sensor type
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific sensor data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_create_sensor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data
);

/**
 * @brief Destroy device and free resources
 *
 * @param device Device to destroy
 */
void mavlink_device_destroy(mavlink_device_t* device);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_DEVICE_INTERFACE_H */
