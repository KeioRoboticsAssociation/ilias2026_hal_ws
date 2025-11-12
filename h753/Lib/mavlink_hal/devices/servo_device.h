/**
 * @file servo_device.h
 * @brief Servo device implementation using unified device interface
 *
 * Provides PWM-based servo control with:
 * - Angle position control
 * - Configurable pulse width mapping
 * - Watchdog timeout and failsafe
 * - Parameter tuning (min/max pulse, neutral angle)
 *
 * @author Claude Code (AI Assistant)
 * @date 2025-11-12
 */

#ifndef MAVLINK_SERVO_DEVICE_H
#define MAVLINK_SERVO_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../include/mavlink_device_interface.h"

/* ========================================================================== */
/*  SERVO PRIVATE DATA                                                        */
/* ========================================================================== */

/**
 * @brief Platform-specific servo data
 *
 * Contains hardware-specific information for PWM control.
 * This structure is stored in device->private_data.
 */
typedef struct {
    /* Hardware resources */
    void* timer_handle;         /**< Timer handle (e.g., TIM_HandleTypeDef*) */
    uint32_t channel;            /**< Timer channel (e.g., TIM_CHANNEL_1) */
    uint32_t timer_frequency_hz; /**< Timer frequency in Hz */

    /* Current state */
    float current_angle;         /**< Current commanded angle */
    uint32_t current_pulse_us;   /**< Current pulse width in microseconds */

    /* Configuration cache */
    float min_angle;
    float max_angle;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    float neutral_angle;

} servo_private_data_t;

/* ========================================================================== */
/*  SERVO DEVICE FACTORY                                                      */
/* ========================================================================== */

/**
 * @brief Create servo device
 *
 * @param id Device ID
 * @param name Device name
 * @param config Device configuration
 * @param timer_handle Platform-specific timer handle
 * @param channel Timer channel
 * @param timer_frequency_hz Timer frequency in Hz
 * @return Pointer to created device, NULL on error
 */
mavlink_device_t* servo_device_create(
    uint8_t id,
    const char* name,
    const mavlink_device_config_t* config,
    void* timer_handle,
    uint32_t channel,
    uint32_t timer_frequency_hz
);

/**
 * @brief Get servo vtable
 *
 * @return Pointer to servo device vtable
 */
const mavlink_device_vtable_t* servo_device_get_vtable(void);

/* ========================================================================== */
/*  SERVO UTILITY FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Convert angle to pulse width
 *
 * @param angle Angle in degrees
 * @param min_angle Minimum angle
 * @param max_angle Maximum angle
 * @param min_pulse_us Minimum pulse width in microseconds
 * @param max_pulse_us Maximum pulse width in microseconds
 * @return Pulse width in microseconds
 */
uint16_t servo_angle_to_pulse(
    float angle,
    float min_angle,
    float max_angle,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us
);

/**
 * @brief Convert pulse width to angle
 *
 * @param pulse_us Pulse width in microseconds
 * @param min_angle Minimum angle
 * @param max_angle Maximum angle
 * @param min_pulse_us Minimum pulse width in microseconds
 * @param max_pulse_us Maximum pulse width in microseconds
 * @return Angle in degrees
 */
float servo_pulse_to_angle(
    uint16_t pulse_us,
    float min_angle,
    float max_angle,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us
);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_SERVO_DEVICE_H */
