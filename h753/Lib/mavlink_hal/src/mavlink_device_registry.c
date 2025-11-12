/**
 * @file mavlink_device_registry.c
 * @brief Device registry for managing multiple devices
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#include "mavlink_device_interface.h"
#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*  DEVICE REGISTRY CONFIGURATION                                             */
/* ========================================================================== */

#ifndef MAVLINK_DEVICE_REGISTRY_MAX_DEVICES
#define MAVLINK_DEVICE_REGISTRY_MAX_DEVICES 64
#endif

/* ========================================================================== */
/*  DEVICE REGISTRY STRUCTURE                                                 */
/* ========================================================================== */

typedef struct {
    mavlink_device_t* devices[MAVLINK_DEVICE_REGISTRY_MAX_DEVICES];
    uint32_t device_count;
    bool initialized;
} mavlink_device_registry_t;

/* Global registry instance */
static mavlink_device_registry_t g_registry = {0};

/* ========================================================================== */
/*  REGISTRY INITIALIZATION                                                   */
/* ========================================================================== */

/**
 * @brief Initialize device registry
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_init(void)
{
    if (g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NONE;
    }

    memset(&g_registry, 0, sizeof(mavlink_device_registry_t));
    g_registry.initialized = true;

    return MAVLINK_DEVICE_ERROR_NONE;
}

/**
 * @brief Shutdown device registry and all devices
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_shutdown(void)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    /* Shutdown and destroy all devices */
    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i]) {
            mavlink_device_destroy(g_registry.devices[i]);
            g_registry.devices[i] = NULL;
        }
    }

    g_registry.device_count = 0;
    g_registry.initialized = false;

    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  DEVICE REGISTRATION                                                       */
/* ========================================================================== */

/**
 * @brief Register device in registry
 * @param device Device to register
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_register(mavlink_device_t* device)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return MAVLINK_DEVICE_ERROR_INVALID_PARAM;
    }

    if (g_registry.device_count >= MAVLINK_DEVICE_REGISTRY_MAX_DEVICES) {
        return MAVLINK_DEVICE_ERROR_OVERFLOW;
    }

    /* Check for duplicate ID */
    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i] && g_registry.devices[i]->id.id == device->id.id) {
            return MAVLINK_DEVICE_ERROR_INVALID_PARAM;  /* Duplicate ID */
        }
    }

    /* Add device to registry */
    g_registry.devices[g_registry.device_count++] = device;

    return MAVLINK_DEVICE_ERROR_NONE;
}

/**
 * @brief Unregister device from registry
 * @param device_id Device ID to unregister
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_unregister(uint8_t device_id)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    /* Find device */
    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i] && g_registry.devices[i]->id.id == device_id) {
            /* Destroy device */
            mavlink_device_destroy(g_registry.devices[i]);

            /* Shift remaining devices */
            for (uint32_t j = i; j < g_registry.device_count - 1; j++) {
                g_registry.devices[j] = g_registry.devices[j + 1];
            }

            g_registry.device_count--;
            g_registry.devices[g_registry.device_count] = NULL;

            return MAVLINK_DEVICE_ERROR_NONE;
        }
    }

    return MAVLINK_DEVICE_ERROR_INVALID_PARAM;  /* Device not found */
}

/* ========================================================================== */
/*  DEVICE LOOKUP                                                             */
/* ========================================================================== */

/**
 * @brief Find device by ID
 * @param device_id Device ID
 * @return Pointer to device, NULL if not found
 */
mavlink_device_t* mavlink_device_registry_find(uint8_t device_id)
{
    if (!g_registry.initialized) {
        return NULL;
    }

    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i] && g_registry.devices[i]->id.id == device_id) {
            return g_registry.devices[i];
        }
    }

    return NULL;
}

/**
 * @brief Find devices by type
 * @param type Device type
 * @param devices Output array of device pointers
 * @param max_devices Maximum number of devices to return
 * @return Number of devices found
 */
uint32_t mavlink_device_registry_find_by_type(
    mavlink_device_type_t type,
    mavlink_device_t** devices,
    uint32_t max_devices)
{
    if (!g_registry.initialized || !devices) {
        return 0;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < g_registry.device_count && count < max_devices; i++) {
        if (g_registry.devices[i] && g_registry.devices[i]->id.type == type) {
            devices[count++] = g_registry.devices[i];
        }
    }

    return count;
}

/**
 * @brief Get all devices
 * @param devices Output array of device pointers
 * @param max_devices Maximum number of devices to return
 * @return Number of devices returned
 */
uint32_t mavlink_device_registry_get_all(
    mavlink_device_t** devices,
    uint32_t max_devices)
{
    if (!g_registry.initialized || !devices) {
        return 0;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < g_registry.device_count && count < max_devices; i++) {
        if (g_registry.devices[i]) {
            devices[count++] = g_registry.devices[i];
        }
    }

    return count;
}

/**
 * @brief Get device count
 * @return Number of registered devices
 */
uint32_t mavlink_device_registry_count(void)
{
    return g_registry.initialized ? g_registry.device_count : 0;
}

/* ========================================================================== */
/*  BULK OPERATIONS                                                           */
/* ========================================================================== */

/**
 * @brief Update all devices
 * @param dt_ms Delta time in milliseconds
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_update_all(uint32_t dt_ms)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    mavlink_device_error_t last_error = MAVLINK_DEVICE_ERROR_NONE;

    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i]) {
            mavlink_device_error_t err = mavlink_device_update(g_registry.devices[i], dt_ms);
            if (err != MAVLINK_DEVICE_ERROR_NONE) {
                last_error = err;
            }
        }
    }

    return last_error;
}

/**
 * @brief Enable/disable all devices
 * @param enable true to enable, false to disable
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_enable_all(bool enable)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    mavlink_device_error_t last_error = MAVLINK_DEVICE_ERROR_NONE;

    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i]) {
            mavlink_device_error_t err = mavlink_device_enable(g_registry.devices[i], enable);
            if (err != MAVLINK_DEVICE_ERROR_NONE) {
                last_error = err;
            }
        }
    }

    return last_error;
}

/**
 * @brief Emergency stop all devices
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_emergency_stop(void)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    /* Disable all devices immediately */
    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i]) {
            mavlink_device_enable(g_registry.devices[i], false);

            /* Set to safe state */
            mavlink_device_handle_timeout(g_registry.devices[i]);
        }
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  TELEMETRY OPERATIONS                                                      */
/* ========================================================================== */

/**
 * @brief Update telemetry for devices that need it
 * @param current_time_ms Current system time
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_update_telemetry(uint32_t current_time_ms)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i] &&
            mavlink_device_needs_telemetry(g_registry.devices[i], current_time_ms)) {
            mavlink_device_update_telemetry(g_registry.devices[i], current_time_ms);
        }
    }

    return MAVLINK_DEVICE_ERROR_NONE;
}

/**
 * @brief Get telemetry for all devices
 * @param telemetry Output array of telemetry structures
 * @param max_devices Maximum number of telemetry items
 * @return Number of telemetry items returned
 */
uint32_t mavlink_device_registry_get_telemetry(
    mavlink_device_telemetry_t* telemetry,
    uint32_t max_devices)
{
    if (!g_registry.initialized || !telemetry) {
        return 0;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < g_registry.device_count && count < max_devices; i++) {
        if (g_registry.devices[i]) {
            memcpy(&telemetry[count++], &g_registry.devices[i]->telemetry,
                   sizeof(mavlink_device_telemetry_t));
        }
    }

    return count;
}

/* ========================================================================== */
/*  DIAGNOSTIC OPERATIONS                                                     */
/* ========================================================================== */

/**
 * @brief Run self-test on all devices
 * @param diagnostics Output array of diagnostic results
 * @param max_devices Maximum number of diagnostics
 * @return Number of diagnostics performed
 */
uint32_t mavlink_device_registry_self_test_all(
    mavlink_device_diagnostic_t* diagnostics,
    uint32_t max_devices)
{
    if (!g_registry.initialized || !diagnostics) {
        return 0;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < g_registry.device_count && count < max_devices; i++) {
        if (g_registry.devices[i]) {
            mavlink_device_self_test(g_registry.devices[i], &diagnostics[count++]);
        }
    }

    return count;
}

/**
 * @brief Get registry statistics
 * @param total_devices Output: total device count
 * @param enabled_devices Output: enabled device count
 * @param error_devices Output: devices in error state
 * @return MAVLINK_DEVICE_ERROR_NONE on success
 */
mavlink_device_error_t mavlink_device_registry_get_stats(
    uint32_t* total_devices,
    uint32_t* enabled_devices,
    uint32_t* error_devices)
{
    if (!g_registry.initialized) {
        return MAVLINK_DEVICE_ERROR_NOT_INITIALIZED;
    }

    uint32_t total = 0, enabled = 0, errors = 0;

    for (uint32_t i = 0; i < g_registry.device_count; i++) {
        if (g_registry.devices[i]) {
            total++;
            if (g_registry.devices[i]->status.enabled) {
                enabled++;
            }
            if (g_registry.devices[i]->status.state == MAVLINK_DEVICE_STATE_ERROR) {
                errors++;
            }
        }
    }

    if (total_devices) *total_devices = total;
    if (enabled_devices) *enabled_devices = enabled;
    if (error_devices) *error_devices = errors;

    return MAVLINK_DEVICE_ERROR_NONE;
}

/* ========================================================================== */
/*  DEVICE CREATION HELPERS                                                   */
/* ========================================================================== */

/**
 * @brief Create and register a servo device
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_registry_create_servo(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    mavlink_device_t* device = mavlink_device_create_servo(id, name, config, private_data);
    if (device) {
        if (mavlink_device_registry_register(device) != MAVLINK_DEVICE_ERROR_NONE) {
            mavlink_device_destroy(device);
            return NULL;
        }
    }
    return device;
}

/**
 * @brief Create and register a motor device
 * @param type Motor type
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_registry_create_motor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    mavlink_device_t* device = mavlink_device_create_motor(type, id, name, config, private_data);
    if (device) {
        if (mavlink_device_registry_register(device) != MAVLINK_DEVICE_ERROR_NONE) {
            mavlink_device_destroy(device);
            return NULL;
        }
    }
    return device;
}

/**
 * @brief Create and register a sensor device
 * @param type Sensor type
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param private_data Platform-specific data
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* mavlink_device_registry_create_sensor(
    mavlink_device_type_t type,
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* private_data)
{
    mavlink_device_t* device = mavlink_device_create_sensor(type, id, name, config, private_data);
    if (device) {
        if (mavlink_device_registry_register(device) != MAVLINK_DEVICE_ERROR_NONE) {
            mavlink_device_destroy(device);
            return NULL;
        }
    }
    return device;
}
