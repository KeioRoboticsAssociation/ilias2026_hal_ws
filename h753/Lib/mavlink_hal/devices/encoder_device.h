/**
 * @file encoder_device.h
 * @brief Encoder sensor device implementation using unified device interface
 *
 * Provides encoder feedback as a sensor device:
 * - Position feedback (radians)
 * - Velocity feedback (rad/s)
 * - Hardware encoder via timer
 * - Configurable counts per revolution and gear ratio
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_ENCODER_DEVICE_H
#define MAVLINK_ENCODER_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../include/mavlink_device_interface.h"

/* ========================================================================== */
/*  ENCODER PRIVATE DATA                                                      */
/* ========================================================================== */

/**
 * @brief Platform-specific encoder data
 */
typedef struct {
    /* Hardware resources */
    void* timer_handle;          /**< Encoder timer handle */
    uint32_t timer_frequency_hz; /**< Timer frequency */

    /* Encoder state */
    int32_t encoder_count;       /**< Current encoder count */
    int32_t prev_encoder_count;  /**< Previous encoder count */
    float encoder_counts_per_rev;/**< Encoder counts per revolution */
    float gear_ratio;            /**< Gear ratio (output/input) */

    /* Calculated feedback */
    float position;              /**< Position in radians */
    float velocity;              /**< Velocity in rad/s */

} encoder_private_data_t;

/* ========================================================================== */
/*  ENCODER DEVICE FACTORY                                                    */
/* ========================================================================== */

/**
 * @brief Create encoder device
 *
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param timer_handle Encoder timer handle
 * @param timer_frequency_hz Timer frequency
 * @param encoder_counts_per_rev Encoder counts per revolution
 * @param gear_ratio Gear ratio (output/input)
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* encoder_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t timer_frequency_hz,
    float encoder_counts_per_rev,
    float gear_ratio
);

/**
 * @brief Get encoder vtable
 *
 * @return Pointer to encoder device vtable
 */
const mavlink_device_vtable_t* encoder_device_get_vtable(void);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_ENCODER_DEVICE_H */
