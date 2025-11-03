/**
 * @file motor_registry.c
 * @brief Motor registry and factory implementation
 */

#include "motor_registry.h"
#include "rs485_controller.h"
#include <string.h>

/* ============================================================================
 * Global Motor Registry
 * ============================================================================ */
motor_registry_t g_motor_registry = {0};

/* ============================================================================
 * Motor Registry Implementation
 * ============================================================================ */

error_code_t motor_registry_init(void) {
    if (g_motor_registry.initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    memset(&g_motor_registry, 0, sizeof(motor_registry_t));
    g_motor_registry.initialized = true;

    return ERROR_OK;
}

error_code_t motor_registry_create_all_motors(void) {
    if (!g_motor_registry.initialized) {
        return ERROR_NOT_INITIALIZED;
    }

    error_code_t err;

    // Create motors from configuration
    for (int i = 0; i < MAX_MOTORS; i++) {
        const motor_instance_t* inst = &MOTOR_INSTANCES[i];

        if (!inst->enabled) {
            continue;
        }

        // Find free slot
        motor_entry_t* entry = NULL;
        for (int j = 0; j < MAX_MOTORS; j++) {
            if (!g_motor_registry.motors[j].allocated) {
                entry = &g_motor_registry.motors[j];
                break;
            }
        }

        if (!entry) {
            return ERROR_RESOURCE_EXHAUSTED;
        }

        // Create motor based on type
        switch (inst->type) {
            case MOTOR_TYPE_SERVO: {
                const servo_config_t* config = get_servo_config(inst->id);
                if (!config) {
                    return ERROR_CONFIG_ERROR;
                }
                err = motor_factory_create_servo(inst->id, inst->timer_id, inst->channel, config, entry);
                break;
            }

            case MOTOR_TYPE_DC: {
                const dc_motor_config_t* config = get_dc_motor_config(inst->id);
                if (!config) {
                    return ERROR_CONFIG_ERROR;
                }
                err = motor_factory_create_dc_motor(inst->id, inst->timer_id, inst->channel, config, entry);
                break;
            }

            case MOTOR_TYPE_ROBOMASTER: {
                const robomaster_config_t* config = get_robomaster_config(inst->id);
                if (!config) {
                    return ERROR_CONFIG_ERROR;
                }
                err = motor_factory_create_robomaster(inst->id, config, entry);
                break;
            }

            case MOTOR_TYPE_RS485: {
                const rs485_config_t* config = get_rs485_config(inst->id);
                if (!config) {
                    return ERROR_CONFIG_ERROR;
                }
                err = motor_factory_create_rs485(inst->id, config, entry);
                break;
            }

            default:
                return ERROR_INVALID_PARAMETER;
        }

        if (err != ERROR_OK) {
            return err;
        }

        // Initialize the motor
        err = motor_initialize(&entry->controller);
        if (err != ERROR_OK) {
            return err;
        }

        entry->allocated = true;
        entry->type = inst->type;
        g_motor_registry.motor_count++;
    }

    return ERROR_OK;
}

error_code_t motor_registry_register(uint8_t id, motor_type_t type, motor_controller_t* controller) {
    if (!g_motor_registry.initialized) {
        return ERROR_NOT_INITIALIZED;
    }

    if (!controller) {
        return ERROR_INVALID_PARAMETER;
    }

    // Check if ID already exists
    if (motor_registry_get(id) != NULL) {
        return ERROR_ALREADY_INITIALIZED;
    }

    // Find free slot
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (!g_motor_registry.motors[i].allocated) {
            g_motor_registry.motors[i].controller = *controller;
            g_motor_registry.motors[i].type = type;
            g_motor_registry.motors[i].allocated = true;
            g_motor_registry.motor_count++;
            return ERROR_OK;
        }
    }

    return ERROR_RESOURCE_EXHAUSTED;
}

motor_controller_t* motor_registry_get(uint8_t id) {
    if (!g_motor_registry.initialized) {
        return NULL;
    }

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (g_motor_registry.motors[i].allocated &&
            g_motor_registry.motors[i].controller.id == id) {
            return &g_motor_registry.motors[i].controller;
        }
    }

    return NULL;
}

motor_state_t motor_registry_get_state(uint8_t id) {
    motor_controller_t* controller = motor_registry_get(id);
    if (controller) {
        return motor_get_state(controller);
    }

    motor_state_t empty = {0};
    empty.status = ERROR_INVALID_PARAMETER;
    return empty;
}

error_code_t motor_registry_send_command(uint8_t id, const motor_command_t* cmd) {
    if (!cmd) {
        return ERROR_INVALID_PARAMETER;
    }

    motor_controller_t* controller = motor_registry_get(id);
    if (!controller) {
        return ERROR_INVALID_PARAMETER;
    }

    return motor_set_command(controller, cmd);
}

error_code_t motor_registry_update_all(float delta_time) {
    if (!g_motor_registry.initialized) {
        return ERROR_NOT_INITIALIZED;
    }

    error_code_t last_error = ERROR_OK;

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (g_motor_registry.motors[i].allocated) {
            error_code_t err = motor_update(&g_motor_registry.motors[i].controller, delta_time);
            if (err != ERROR_OK) {
                last_error = err;
            }
        }
    }

    return last_error;
}

void motor_registry_emergency_stop_all(void) {
    if (!g_motor_registry.initialized) {
        return;
    }

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (g_motor_registry.motors[i].allocated) {
            motor_emergency_stop(&g_motor_registry.motors[i].controller);
        }
    }
}

uint8_t motor_registry_get_count(void) {
    return g_motor_registry.motor_count;
}

bool motor_registry_is_initialized(void) {
    return g_motor_registry.initialized;
}

/* ============================================================================
 * Motor Factory Implementation
 * ============================================================================ */

error_code_t motor_factory_create_servo(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const servo_config_t* config,
    motor_entry_t* entry)
{
    if (!config || !entry) {
        return ERROR_INVALID_PARAMETER;
    }

    // Create servo controller
    error_code_t err = servo_controller_create(
        id,
        timer_id,
        channel,
        config,
        &entry->controller,
        &entry->private_data.servo);

    if (err != ERROR_OK) {
        return err;
    }

    entry->type = MOTOR_TYPE_SERVO;
    return ERROR_OK;
}

error_code_t motor_factory_create_dc_motor(
    uint8_t id,
    uint8_t timer_id,
    uint32_t channel,
    const dc_motor_config_t* config,
    motor_entry_t* entry)
{
    if (!config || !entry) {
        return ERROR_INVALID_PARAMETER;
    }

    // Create DC motor controller
    error_code_t err = dc_motor_controller_create(
        id,
        timer_id,
        channel,
        config,
        &entry->controller,
        &entry->private_data.dc_motor);

    if (err != ERROR_OK) {
        return err;
    }

    entry->type = MOTOR_TYPE_DC;
    return ERROR_OK;
}

error_code_t motor_factory_create_robomaster(
    uint8_t id,
    const robomaster_config_t* config,
    motor_entry_t* entry)
{
    if (!config || !entry) {
        return ERROR_INVALID_PARAMETER;
    }

    // Create RoboMaster controller
    error_code_t err = robomaster_controller_create(
        id,
        config,
        &entry->controller,
        &entry->private_data.robomaster);

    if (err != ERROR_OK) {
        return err;
    }

    entry->type = MOTOR_TYPE_ROBOMASTER;
    return ERROR_OK;
}

/* ============================================================================
 * RS485 Motor Factory Function
 * ============================================================================ */
static error_code_t motor_factory_create_rs485(
    uint8_t id,
    const rs485_config_t* config,
    motor_entry_t* entry)
{
    if (!config || !entry) {
        return ERROR_INVALID_PARAMETER;
    }

    error_code_t err = rs485_controller_create(
        id,
        config,
        &entry->controller,
        &entry->private_data.rs485);

    if (err != ERROR_OK) {
        return err;
    }

    entry->type = MOTOR_TYPE_RS485;
    return ERROR_OK;
}
