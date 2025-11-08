/**
 * @file parameter_manager.c
 * @brief Parameter management system implementation
 */

#include "parameter_manager.h"
#include "motor_config.h"
#include "../motors/motor_registry.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Global Parameter Manager
 * ============================================================================ */
param_manager_t g_param_manager = {0};

/* ============================================================================
 * Flash Storage Configuration
 * ============================================================================ */

// Flash sector for parameter storage (use last sector of Flash)
// STM32H753 has 128KB Flash sectors
// This would need to be configured based on your linker script
#define PARAM_FLASH_SECTOR_ADDR 0x081E0000  // Example: last 128KB sector
#define PARAM_FLASH_MAGIC 0x50415241  // "PARA" magic number

typedef struct {
    uint32_t magic;
    uint16_t param_count;
    uint16_t crc16;
    float values[MAX_PARAMETERS];
} param_flash_storage_t;

/* ============================================================================
 * Private Helper Functions
 * ============================================================================ */

/**
 * @brief Calculate CRC16 for data
 */
static uint16_t calculate_crc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Validate parameter index
 */
static bool is_valid_index(uint16_t index) {
    return (index < g_param_manager.param_count);
}

/**
 * @brief Clamp value to min/max range
 */
static float clamp_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/* ============================================================================
 * Parameter Manager API Implementation
 * ============================================================================ */

error_code_t param_manager_init(void) {
    if (g_param_manager.initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    memset(&g_param_manager, 0, sizeof(param_manager_t));
    g_param_manager.param_count = 0;
    g_param_manager.initialized = true;

    return ERROR_OK;
}

error_code_t param_manager_register(
    const char* name,
    param_type_t type,
    float* value_ptr,
    float default_value,
    float min_value,
    float max_value,
    bool read_only)
{
    if (!g_param_manager.initialized) {
        return ERROR_NOT_INITIALIZED;
    }

    if (!name || !value_ptr) {
        return ERROR_INVALID_PARAMETER;
    }

    if (g_param_manager.param_count >= MAX_PARAMETERS) {
        return ERROR_RESOURCE_EXHAUSTED;
    }

    // Check for duplicate names
    if (param_manager_get_by_name(name) != NULL) {
        return ERROR_CONFIG_ERROR;  // Parameter already registered
    }

    param_entry_t* entry = &g_param_manager.params[g_param_manager.param_count];

    strncpy(entry->name, name, 15);
    entry->name[15] = '\0';
    entry->type = type;
    entry->value_ptr = value_ptr;
    entry->default_value = default_value;
    entry->min_value = min_value;
    entry->max_value = max_value;
    entry->read_only = read_only;

    // Set initial value to default
    *value_ptr = default_value;

    g_param_manager.param_count++;

    return ERROR_OK;
}

uint16_t param_manager_get_count(void) {
    return g_param_manager.param_count;
}

const param_entry_t* param_manager_get_by_index(uint16_t index) {
    if (!is_valid_index(index)) {
        return NULL;
    }

    return &g_param_manager.params[index];
}

const param_entry_t* param_manager_get_by_name(const char* name) {
    if (!name) {
        return NULL;
    }

    for (uint16_t i = 0; i < g_param_manager.param_count; i++) {
        if (strcmp(g_param_manager.params[i].name, name) == 0) {
            return &g_param_manager.params[i];
        }
    }

    return NULL;
}

int16_t param_manager_get_index_by_name(const char* name) {
    if (!name) {
        return -1;
    }

    for (uint16_t i = 0; i < g_param_manager.param_count; i++) {
        if (strcmp(g_param_manager.params[i].name, name) == 0) {
            return (int16_t)i;
        }
    }

    return -1;
}

error_code_t param_manager_set(const char* name, float value) {
    const param_entry_t* entry = param_manager_get_by_name(name);

    if (!entry) {
        return ERROR_INVALID_PARAMETER;
    }

    if (entry->read_only) {
        return ERROR_SAFETY_VIOLATION;
    }

    // Clamp to valid range
    float clamped_value = clamp_value(value, entry->min_value, entry->max_value);

    // Update value
    *(entry->value_ptr) = clamped_value;

    return ERROR_OK;
}

error_code_t param_manager_set_by_index(uint16_t index, float value) {
    if (!is_valid_index(index)) {
        return ERROR_INVALID_PARAMETER;
    }

    const param_entry_t* entry = &g_param_manager.params[index];

    if (entry->read_only) {
        return ERROR_SAFETY_VIOLATION;
    }

    // Clamp to valid range
    float clamped_value = clamp_value(value, entry->min_value, entry->max_value);

    // Update value
    *(entry->value_ptr) = clamped_value;

    return ERROR_OK;
}

error_code_t param_manager_get(const char* name, float* value) {
    const param_entry_t* entry = param_manager_get_by_name(name);

    if (!entry || !value) {
        return ERROR_INVALID_PARAMETER;
    }

    *value = *(entry->value_ptr);

    return ERROR_OK;
}

error_code_t param_manager_reset(const char* name) {
    const param_entry_t* entry = param_manager_get_by_name(name);

    if (!entry) {
        return ERROR_INVALID_PARAMETER;
    }

    if (entry->read_only) {
        return ERROR_SAFETY_VIOLATION;
    }

    *(entry->value_ptr) = entry->default_value;

    return ERROR_OK;
}

error_code_t param_manager_reset_all(void) {
    for (uint16_t i = 0; i < g_param_manager.param_count; i++) {
        param_entry_t* entry = &g_param_manager.params[i];
        if (!entry->read_only) {
            *(entry->value_ptr) = entry->default_value;
        }
    }

    return ERROR_OK;
}

error_code_t param_manager_save_to_flash(void) {
    // Note: Flash programming on STM32H7 requires special handling
    // This is a placeholder implementation
    // In production, you would:
    // 1. Unlock Flash
    // 2. Erase sector
    // 3. Write data
    // 4. Lock Flash

    param_flash_storage_t storage = {0};
    storage.magic = PARAM_FLASH_MAGIC;
    storage.param_count = g_param_manager.param_count;

    // Copy all parameter values
    for (uint16_t i = 0; i < g_param_manager.param_count; i++) {
        storage.values[i] = *(g_param_manager.params[i].value_ptr);
    }

    // Calculate CRC
    storage.crc16 = calculate_crc16((uint8_t*)storage.values,
                                    g_param_manager.param_count * sizeof(float));

    // TODO: Implement Flash write using HAL_FLASH_* functions
    // For now, return success

    return ERROR_OK;
}

error_code_t param_manager_load_from_flash(void) {
    // Note: This is a placeholder implementation
    // In production, you would:
    // 1. Read Flash sector
    // 2. Verify magic number and CRC
    // 3. Restore parameter values

    param_flash_storage_t* storage = (param_flash_storage_t*)PARAM_FLASH_SECTOR_ADDR;

    // Verify magic number
    if (storage->magic != PARAM_FLASH_MAGIC) {
        return ERROR_CONFIG_ERROR;  // No valid data in Flash
    }

    // Verify CRC
    uint16_t calculated_crc = calculate_crc16((uint8_t*)storage->values,
                                              storage->param_count * sizeof(float));
    if (calculated_crc != storage->crc16) {
        return ERROR_COMM_ERROR;  // CRC mismatch
    }

    // Verify param count matches
    if (storage->param_count != g_param_manager.param_count) {
        return ERROR_CONFIG_ERROR;  // Parameter count mismatch
    }

    // Restore values
    for (uint16_t i = 0; i < g_param_manager.param_count; i++) {
        float value = storage->values[i];
        param_entry_t* entry = &g_param_manager.params[i];

        // Validate and clamp
        value = clamp_value(value, entry->min_value, entry->max_value);
        *(entry->value_ptr) = value;
    }

    return ERROR_OK;
}

error_code_t param_manager_register_motor_params(void) {
    error_code_t err;

    // Note: We'll register pointers to runtime PID gain storage
    // These will be defined in the motor controllers

    // RoboMaster motor parameters
    for (int i = 0; i < MAX_ROBOMASTER; i++) {
        const robomaster_config_t* config = &ROBOMASTER_CONFIGS[i];
        motor_controller_t* controller = motor_registry_get(config->id);

        if (controller && controller->type == MOTOR_TYPE_ROBOMASTER) {
            robomaster_private_t* priv = (robomaster_private_t*)controller->private_data;
            char name[16];

            // Angle PID
            snprintf(name, 16, "RM_%d_ANG_KP", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.angle_kp, config->angle_kp, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "RM_%d_ANG_KI", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.angle_ki, config->angle_ki, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "RM_%d_ANG_KD", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.angle_kd, config->angle_kd, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            // Speed PID
            snprintf(name, 16, "RM_%d_SPD_KP", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_kp, config->speed_kp, 0.0f, 100.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "RM_%d_SPD_KI", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_ki, config->speed_ki, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "RM_%d_SPD_KD", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_kd, config->speed_kd, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;
        }
    }

    // DC motor parameters
    for (int i = 0; i < MAX_DC_MOTORS; i++) {
        const dc_motor_config_t* config = &DC_MOTOR_CONFIGS[i];
        motor_controller_t* controller = motor_registry_get(config->id);

        if (controller && controller->type == MOTOR_TYPE_DC) {
            dc_motor_private_t* priv = (dc_motor_private_t*)controller->private_data;
            char name[16];

            // Speed PID
            snprintf(name, 16, "DC_%d_SPD_KP", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_kp, config->speed_kp, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "DC_%d_SPD_KI", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_ki, config->speed_ki, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "DC_%d_SPD_KD", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.speed_kd, config->speed_kd, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            // Position PID
            snprintf(name, 16, "DC_%d_POS_KP", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.position_kp, config->position_kp, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "DC_%d_POS_KI", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.position_ki, config->position_ki, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;

            snprintf(name, 16, "DC_%d_POS_KD", config->id);
            err = param_manager_register(name, PARAM_TYPE_FLOAT,
                &priv->config.position_kd, config->position_kd, 0.0f, 10.0f, false);
            if (err != ERROR_OK) return err;
        }
    }

    return ERROR_OK;
}
