/**
 * @file mavlink_hal_interface.h
 * @brief Main Hardware Abstraction Layer interface for portable MAVLink
 *
 * This header-only HAL provides a platform-agnostic interface for MAVLink
 * communication across STM32, Arduino, ESP32, and other microcontrollers.
 *
 * Design principles:
 * - Pure C99 implementation
 * - Header-only with inline functions
 * - No dynamic memory allocation
 * - Platform-agnostic function pointers
 * - Thread-safe operation support
 *
 * @author Auto-generated for Epic1 Task1
 * @date 2025-11-10
 */

#ifndef MAVLINK_HAL_INTERFACE_H
#define MAVLINK_HAL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_hal_types.h"

/* ============================================================================
 * Platform Detection
 * ============================================================================ */

/**
 * Platform detection defines - use different names than enum to avoid conflicts
 */
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32L4)
    #define MAVLINK_HAL_ON_STM32
#elif defined(ARDUINO)
    #define MAVLINK_HAL_ON_ARDUINO
#elif defined(ESP32)
    #define MAVLINK_HAL_ON_ESP32
#elif defined(__linux__)
    #define MAVLINK_HAL_ON_LINUX
#else
    #define MAVLINK_HAL_ON_UNKNOWN
#endif

/* ============================================================================
 * Function Pointer Types - UART Operations
 * ============================================================================ */

/**
 * @brief Initialize UART peripheral
 * @param handle UART handle (platform-specific)
 * @param config UART configuration
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_init_fn)(
    mavlink_hal_uart_handle_t handle,
    const mavlink_hal_uart_config_t* config
);

/**
 * @brief Deinitialize UART peripheral
 * @param handle UART handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_deinit_fn)(
    mavlink_hal_uart_handle_t handle
);

/**
 * @brief Send data over UART (blocking)
 * @param handle UART handle
 * @param data Data to send
 * @param length Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 * @param bytes_sent Number of bytes actually sent (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_send_fn)(
    mavlink_hal_uart_handle_t handle,
    const uint8_t* data,
    size_t length,
    mavlink_hal_timeout_ms_t timeout_ms,
    size_t* bytes_sent
);

/**
 * @brief Receive data from UART (blocking)
 * @param handle UART handle
 * @param buffer Buffer to store received data
 * @param length Maximum number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 * @param bytes_received Number of bytes actually received (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_receive_fn)(
    mavlink_hal_uart_handle_t handle,
    uint8_t* buffer,
    size_t length,
    mavlink_hal_timeout_ms_t timeout_ms,
    size_t* bytes_received
);

/**
 * @brief Register UART RX callback
 * @param handle UART handle
 * @param callback Callback function
 * @param user_data User context pointer
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_register_rx_callback_fn)(
    mavlink_hal_uart_handle_t handle,
    mavlink_hal_uart_rx_callback_t callback,
    void* user_data
);

/**
 * @brief Get number of bytes available in RX buffer
 * @param handle UART handle
 * @param available Number of bytes available (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_uart_available_fn)(
    mavlink_hal_uart_handle_t handle,
    size_t* available
);

/* ============================================================================
 * Function Pointer Types - GPIO Operations
 * ============================================================================ */

/**
 * @brief Initialize GPIO pin
 * @param handle GPIO handle
 * @param config GPIO configuration
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_gpio_init_fn)(
    mavlink_hal_gpio_handle_t handle,
    const mavlink_hal_gpio_config_t* config
);

/**
 * @brief Write GPIO pin state
 * @param handle GPIO handle
 * @param state Pin state (true=high, false=low)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_gpio_write_fn)(
    mavlink_hal_gpio_handle_t handle,
    bool state
);

/**
 * @brief Read GPIO pin state
 * @param handle GPIO handle
 * @param state Pin state (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_gpio_read_fn)(
    mavlink_hal_gpio_handle_t handle,
    bool* state
);

/**
 * @brief Toggle GPIO pin state
 * @param handle GPIO handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_gpio_toggle_fn)(
    mavlink_hal_gpio_handle_t handle
);

/**
 * @brief Register GPIO interrupt callback
 * @param handle GPIO handle
 * @param callback Callback function
 * @param user_data User context pointer
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_gpio_register_irq_fn)(
    mavlink_hal_gpio_handle_t handle,
    mavlink_hal_gpio_irq_callback_t callback,
    void* user_data
);

/* ============================================================================
 * Function Pointer Types - Timer Operations
 * ============================================================================ */

/**
 * @brief Get current time in milliseconds
 * @return Milliseconds since system start
 */
typedef uint32_t (*mavlink_hal_time_millis_fn)(void);

/**
 * @brief Get current time in microseconds
 * @return Microseconds since system start
 */
typedef mavlink_hal_timestamp_us_t (*mavlink_hal_time_micros_fn)(void);

/**
 * @brief Delay for specified milliseconds (blocking)
 * @param ms Milliseconds to delay
 */
typedef void (*mavlink_hal_delay_ms_fn)(uint32_t ms);

/**
 * @brief Delay for specified microseconds (blocking)
 * @param us Microseconds to delay
 */
typedef void (*mavlink_hal_delay_us_fn)(uint32_t us);

/**
 * @brief Start a periodic timer
 * @param handle Timer handle
 * @param period_ms Period in milliseconds
 * @param callback Callback function
 * @param user_data User context pointer
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_timer_start_fn)(
    mavlink_hal_timer_handle_t handle,
    uint32_t period_ms,
    mavlink_hal_timer_callback_t callback,
    void* user_data
);

/**
 * @brief Stop a timer
 * @param handle Timer handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_timer_stop_fn)(
    mavlink_hal_timer_handle_t handle
);

/* ============================================================================
 * Function Pointer Types - PWM Operations
 * ============================================================================ */

/**
 * @brief Initialize PWM channel
 * @param handle PWM handle
 * @param config PWM configuration
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_pwm_init_fn)(
    mavlink_hal_pwm_handle_t handle,
    const mavlink_hal_pwm_config_t* config
);

/**
 * @brief Set PWM duty cycle
 * @param handle PWM handle
 * @param duty_cycle Duty cycle value (0 to max resolution)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_pwm_set_duty_fn)(
    mavlink_hal_pwm_handle_t handle,
    uint16_t duty_cycle
);

/**
 * @brief Set PWM duty cycle as percentage
 * @param handle PWM handle
 * @param percent Duty cycle percentage (0.0 - 100.0)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_pwm_set_duty_percent_fn)(
    mavlink_hal_pwm_handle_t handle,
    float percent
);

/**
 * @brief Start PWM output
 * @param handle PWM handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_pwm_start_fn)(
    mavlink_hal_pwm_handle_t handle
);

/**
 * @brief Stop PWM output
 * @param handle PWM handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_pwm_stop_fn)(
    mavlink_hal_pwm_handle_t handle
);

/* ============================================================================
 * Function Pointer Types - ADC Operations
 * ============================================================================ */

/**
 * @brief Initialize ADC channel
 * @param handle ADC handle
 * @param config ADC configuration
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_adc_init_fn)(
    mavlink_hal_adc_handle_t handle,
    const mavlink_hal_adc_config_t* config
);

/**
 * @brief Read ADC value (blocking)
 * @param handle ADC handle
 * @param value Raw ADC value (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_adc_read_fn)(
    mavlink_hal_adc_handle_t handle,
    uint16_t* value
);

/**
 * @brief Read ADC value as voltage
 * @param handle ADC handle
 * @param voltage Voltage value (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_adc_read_voltage_fn)(
    mavlink_hal_adc_handle_t handle,
    float* voltage
);

/* ============================================================================
 * Function Pointer Types - CAN Operations (Optional)
 * ============================================================================ */

#ifdef MAVLINK_HAL_ENABLE_CAN

/**
 * @brief Initialize CAN bus
 * @param handle CAN handle
 * @param config CAN configuration
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_can_init_fn)(
    mavlink_hal_can_handle_t handle,
    const mavlink_hal_can_config_t* config
);

/**
 * @brief Send CAN message
 * @param handle CAN handle
 * @param msg CAN message to send
 * @param timeout_ms Timeout in milliseconds
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_can_send_fn)(
    mavlink_hal_can_handle_t handle,
    const mavlink_hal_can_msg_t* msg,
    mavlink_hal_timeout_ms_t timeout_ms
);

/**
 * @brief Receive CAN message
 * @param handle CAN handle
 * @param msg Received CAN message (output)
 * @param timeout_ms Timeout in milliseconds
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_can_receive_fn)(
    mavlink_hal_can_handle_t handle,
    mavlink_hal_can_msg_t* msg,
    mavlink_hal_timeout_ms_t timeout_ms
);

/**
 * @brief Register CAN RX callback
 * @param handle CAN handle
 * @param callback Callback function
 * @param user_data User context pointer
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_can_register_rx_callback_fn)(
    mavlink_hal_can_handle_t handle,
    mavlink_hal_can_rx_callback_t callback,
    void* user_data
);

#endif /* MAVLINK_HAL_ENABLE_CAN */

/* ============================================================================
 * Function Pointer Types - Thread Safety (Optional)
 * ============================================================================ */

#ifdef MAVLINK_HAL_ENABLE_MUTEX

/**
 * @brief Create a mutex
 * @param mutex Mutex handle (output)
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_mutex_create_fn)(
    mavlink_hal_mutex_handle_t* mutex
);

/**
 * @brief Destroy a mutex
 * @param mutex Mutex handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_mutex_destroy_fn)(
    mavlink_hal_mutex_handle_t mutex
);

/**
 * @brief Lock a mutex
 * @param mutex Mutex handle
 * @param timeout_ms Timeout in milliseconds
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_mutex_lock_fn)(
    mavlink_hal_mutex_handle_t mutex,
    mavlink_hal_timeout_ms_t timeout_ms
);

/**
 * @brief Unlock a mutex
 * @param mutex Mutex handle
 * @return Error code
 */
typedef mavlink_hal_error_t (*mavlink_hal_mutex_unlock_fn)(
    mavlink_hal_mutex_handle_t mutex
);

#endif /* MAVLINK_HAL_ENABLE_MUTEX */

/* ============================================================================
 * Debug/Logging Function Pointers (Optional)
 * ============================================================================ */

#ifdef MAVLINK_HAL_ENABLE_DEBUG

/**
 * @brief Debug print callback
 * @param level Debug level (0=error, 1=warn, 2=info, 3=debug)
 * @param message Message string
 */
typedef void (*mavlink_hal_debug_print_fn)(uint8_t level, const char* message);

#endif /* MAVLINK_HAL_ENABLE_DEBUG */

/* ============================================================================
 * HAL Interface Structure
 * ============================================================================ */

/**
 * @brief Main HAL interface structure containing all function pointers
 *
 * Platform implementations should populate this structure with
 * platform-specific function implementations.
 */
typedef struct {
    /* Platform information */
    mavlink_hal_platform_t  platform;
    uint32_t                features;
    uint8_t                 version_major;
    uint8_t                 version_minor;
    uint8_t                 version_patch;
    const char*             platform_name;

    /* UART operations */
    mavlink_hal_uart_init_fn                uart_init;
    mavlink_hal_uart_deinit_fn              uart_deinit;
    mavlink_hal_uart_send_fn                uart_send;
    mavlink_hal_uart_receive_fn             uart_receive;
    mavlink_hal_uart_register_rx_callback_fn uart_register_rx_callback;
    mavlink_hal_uart_available_fn           uart_available;

    /* GPIO operations */
    mavlink_hal_gpio_init_fn                gpio_init;
    mavlink_hal_gpio_write_fn               gpio_write;
    mavlink_hal_gpio_read_fn                gpio_read;
    mavlink_hal_gpio_toggle_fn              gpio_toggle;
    mavlink_hal_gpio_register_irq_fn        gpio_register_irq;

    /* Timer operations */
    mavlink_hal_time_millis_fn              time_millis;
    mavlink_hal_time_micros_fn              time_micros;
    mavlink_hal_delay_ms_fn                 delay_ms;
    mavlink_hal_delay_us_fn                 delay_us;
    mavlink_hal_timer_start_fn              timer_start;
    mavlink_hal_timer_stop_fn               timer_stop;

    /* PWM operations */
    mavlink_hal_pwm_init_fn                 pwm_init;
    mavlink_hal_pwm_set_duty_fn             pwm_set_duty;
    mavlink_hal_pwm_set_duty_percent_fn     pwm_set_duty_percent;
    mavlink_hal_pwm_start_fn                pwm_start;
    mavlink_hal_pwm_stop_fn                 pwm_stop;

    /* ADC operations */
    mavlink_hal_adc_init_fn                 adc_init;
    mavlink_hal_adc_read_fn                 adc_read;
    mavlink_hal_adc_read_voltage_fn         adc_read_voltage;

#ifdef MAVLINK_HAL_ENABLE_CAN
    /* CAN operations (optional) */
    mavlink_hal_can_init_fn                 can_init;
    mavlink_hal_can_send_fn                 can_send;
    mavlink_hal_can_receive_fn              can_receive;
    mavlink_hal_can_register_rx_callback_fn can_register_rx_callback;
#endif

#ifdef MAVLINK_HAL_ENABLE_MUTEX
    /* Mutex operations (optional) */
    mavlink_hal_mutex_create_fn             mutex_create;
    mavlink_hal_mutex_destroy_fn            mutex_destroy;
    mavlink_hal_mutex_lock_fn               mutex_lock;
    mavlink_hal_mutex_unlock_fn             mutex_unlock;
#endif

#ifdef MAVLINK_HAL_ENABLE_DEBUG
    /* Debug operations (optional) */
    mavlink_hal_debug_print_fn              debug_print;
#endif

} mavlink_hal_interface_t;

/* ============================================================================
 * Global HAL Instance
 * ============================================================================ */

/**
 * @brief Global HAL interface instance
 *
 * Platform implementations should register their functions by populating
 * this structure using MAVLINK_HAL_REGISTER() macro.
 */
extern mavlink_hal_interface_t g_mavlink_hal;

/* ============================================================================
 * Registration Macro
 * ============================================================================ */

/**
 * @brief Register HAL implementation
 *
 * Platform implementations should use this macro to register their
 * function pointers with the global HAL interface.
 *
 * Example usage in platform implementation:
 * @code
 * MAVLINK_HAL_REGISTER(
 *     .platform = MAVLINK_HAL_PLATFORM_STM32,
 *     .features = MAVLINK_HAL_FEATURE_UART | MAVLINK_HAL_FEATURE_GPIO,
 *     .uart_init = stm32_uart_init,
 *     .uart_send = stm32_uart_send,
 *     // ... other functions
 * );
 * @endcode
 */
#define MAVLINK_HAL_REGISTER(...) \
    mavlink_hal_interface_t g_mavlink_hal = { __VA_ARGS__ }

/* ============================================================================
 * Convenience Inline Functions
 * ============================================================================ */

/**
 * @brief Check if HAL is initialized
 * @return true if initialized, false otherwise
 */
static inline bool mavlink_hal_is_initialized(void)
{
    return (g_mavlink_hal.platform != MAVLINK_HAL_PLATFORM_UNKNOWN);
}

/**
 * @brief Get platform name
 * @return Platform name string
 */
static inline const char* mavlink_hal_get_platform_name(void)
{
    return g_mavlink_hal.platform_name;
}

/**
 * @brief Check if feature is supported
 * @param feature Feature flag to check
 * @return true if supported, false otherwise
 */
static inline bool mavlink_hal_has_feature(mavlink_hal_features_t feature)
{
    return MAVLINK_HAL_HAS_FEATURE(g_mavlink_hal.features, feature);
}

/**
 * @brief Validate HAL interface completeness
 * @return Error code indicating missing functions
 */
static inline mavlink_hal_error_t mavlink_hal_validate(void)
{
    /* Check required UART functions */
    if (!g_mavlink_hal.uart_init || !g_mavlink_hal.uart_send ||
        !g_mavlink_hal.uart_receive) {
        return MAVLINK_HAL_ERR_NOT_INITIALIZED;
    }

    /* Check required timer functions */
    if (!g_mavlink_hal.time_millis || !g_mavlink_hal.delay_ms) {
        return MAVLINK_HAL_ERR_NOT_INITIALIZED;
    }

    return MAVLINK_HAL_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_HAL_INTERFACE_H */
