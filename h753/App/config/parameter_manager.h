/**
 * @file parameter_manager.h
 * @brief Parameter management system for MAVLink PARAM_* protocol
 *
 * This module manages runtime-adjustable parameters (primarily PID gains)
 * and integrates with MAVLink parameter protocol.
 */

#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Parameter Types
 * ============================================================================ */

typedef enum {
    PARAM_TYPE_FLOAT = 0,
    PARAM_TYPE_INT32 = 1,
    PARAM_TYPE_UINT32 = 2,
    PARAM_TYPE_INT16 = 3,
    PARAM_TYPE_UINT16 = 4,
    PARAM_TYPE_INT8 = 5,
    PARAM_TYPE_UINT8 = 6
} param_type_t;

/* ============================================================================
 * Parameter Entry
 * ============================================================================ */

typedef struct {
    char name[16];           // Parameter name (e.g., "RM_20_SPEED_KP")
    param_type_t type;       // Parameter type
    float* value_ptr;        // Pointer to actual value storage
    float default_value;     // Default value for reset
    float min_value;         // Minimum allowed value
    float max_value;         // Maximum allowed value
    bool read_only;          // Parameter is read-only
} param_entry_t;

/* ============================================================================
 * Parameter Manager
 * ============================================================================ */

#define MAX_PARAMETERS 64    // Maximum number of parameters

typedef struct {
    param_entry_t params[MAX_PARAMETERS];
    uint16_t param_count;
    bool initialized;
} param_manager_t;

/* ============================================================================
 * Global Parameter Manager
 * ============================================================================ */
extern param_manager_t g_param_manager;

/* ============================================================================
 * Parameter Manager API
 * ============================================================================ */

/**
 * @brief Initialize parameter manager
 * @return Error code
 */
error_code_t param_manager_init(void);

/**
 * @brief Register a parameter
 * @param name Parameter name (max 15 chars)
 * @param type Parameter type
 * @param value_ptr Pointer to value storage
 * @param default_value Default value
 * @param min_value Minimum allowed value
 * @param max_value Maximum allowed value
 * @param read_only Parameter is read-only
 * @return Error code
 */
error_code_t param_manager_register(
    const char* name,
    param_type_t type,
    float* value_ptr,
    float default_value,
    float min_value,
    float max_value,
    bool read_only);

/**
 * @brief Get parameter count
 * @return Number of registered parameters
 */
uint16_t param_manager_get_count(void);

/**
 * @brief Get parameter by index
 * @param index Parameter index (0 to count-1)
 * @return Pointer to parameter entry or NULL if invalid
 */
const param_entry_t* param_manager_get_by_index(uint16_t index);

/**
 * @brief Get parameter by name
 * @param name Parameter name
 * @return Pointer to parameter entry or NULL if not found
 */
const param_entry_t* param_manager_get_by_name(const char* name);

/**
 * @brief Get parameter index by name
 * @param name Parameter name
 * @return Parameter index or -1 if not found
 */
int16_t param_manager_get_index_by_name(const char* name);

/**
 * @brief Set parameter value
 * @param name Parameter name
 * @param value New value
 * @return Error code
 */
error_code_t param_manager_set(const char* name, float value);

/**
 * @brief Set parameter value by index
 * @param index Parameter index
 * @param value New value
 * @return Error code
 */
error_code_t param_manager_set_by_index(uint16_t index, float value);

/**
 * @brief Get parameter value
 * @param name Parameter name
 * @param value Output value
 * @return Error code
 */
error_code_t param_manager_get(const char* name, float* value);

/**
 * @brief Reset parameter to default value
 * @param name Parameter name
 * @return Error code
 */
error_code_t param_manager_reset(const char* name);

/**
 * @brief Reset all parameters to default values
 * @return Error code
 */
error_code_t param_manager_reset_all(void);

/**
 * @brief Save all parameters to Flash
 * @return Error code
 */
error_code_t param_manager_save_to_flash(void);

/**
 * @brief Load all parameters from Flash
 * @return Error code
 */
error_code_t param_manager_load_from_flash(void);

/**
 * @brief Register all motor PID parameters
 * @note Called during initialization to register RoboMaster, DC, and RS485 PID gains
 * @return Error code
 */
error_code_t param_manager_register_motor_params(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_MANAGER_H */
