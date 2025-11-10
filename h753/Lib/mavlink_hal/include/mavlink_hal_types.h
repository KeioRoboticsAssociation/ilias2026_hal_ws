/**
 * @file mavlink_hal_types.h
 * @brief Common types and error codes for portable MAVLink HAL
 *
 * This file defines platform-agnostic types and constants for the MAVLink
 * Hardware Abstraction Layer. It is designed to be C99 compatible and
 * header-only.
 *
 * @author Auto-generated for Epic1 Task1
 * @date 2025-11-10
 */

#ifndef MAVLINK_HAL_TYPES_H
#define MAVLINK_HAL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * Version Information
 * ============================================================================ */

#define MAVLINK_HAL_VERSION_MAJOR    1
#define MAVLINK_HAL_VERSION_MINOR    0
#define MAVLINK_HAL_VERSION_PATCH    0

/* ============================================================================
 * Error Codes
 * ============================================================================ */

typedef enum {
    MAVLINK_HAL_OK                  = 0,    /**< Operation successful */
    MAVLINK_HAL_ERR_GENERIC         = -1,   /**< Generic error */
    MAVLINK_HAL_ERR_INVALID_PARAM   = -2,   /**< Invalid parameter */
    MAVLINK_HAL_ERR_NOT_INITIALIZED = -3,   /**< HAL not initialized */
    MAVLINK_HAL_ERR_ALREADY_INIT    = -4,   /**< Already initialized */
    MAVLINK_HAL_ERR_TIMEOUT         = -5,   /**< Operation timeout */
    MAVLINK_HAL_ERR_BUSY            = -6,   /**< Resource busy */
    MAVLINK_HAL_ERR_NO_MEM          = -7,   /**< Out of memory */
    MAVLINK_HAL_ERR_NOT_SUPPORTED   = -8,   /**< Feature not supported */
    MAVLINK_HAL_ERR_HW_FAULT        = -9,   /**< Hardware fault */
    MAVLINK_HAL_ERR_IO              = -10,  /**< I/O error */
    MAVLINK_HAL_ERR_CRC             = -11,  /**< CRC/checksum error */
    MAVLINK_HAL_ERR_OVERFLOW        = -12,  /**< Buffer overflow */
    MAVLINK_HAL_ERR_UNDERFLOW       = -13,  /**< Buffer underflow */
} mavlink_hal_error_t;

/* ============================================================================
 * Platform Types
 * ============================================================================ */

/**
 * @brief Platform identifier
 */
typedef enum {
    MAVLINK_HAL_PLATFORM_UNKNOWN    = 0,
    MAVLINK_HAL_PLATFORM_STM32      = 1,
    MAVLINK_HAL_PLATFORM_ARDUINO    = 2,
    MAVLINK_HAL_PLATFORM_ESP32      = 3,
    MAVLINK_HAL_PLATFORM_LINUX      = 4,
    MAVLINK_HAL_PLATFORM_TEST       = 99,
} mavlink_hal_platform_t;

/**
 * @brief Platform features bitmask
 */
typedef enum {
    MAVLINK_HAL_FEATURE_UART        = (1 << 0),
    MAVLINK_HAL_FEATURE_GPIO        = (1 << 1),
    MAVLINK_HAL_FEATURE_TIMER       = (1 << 2),
    MAVLINK_HAL_FEATURE_PWM         = (1 << 3),
    MAVLINK_HAL_FEATURE_ADC         = (1 << 4),
    MAVLINK_HAL_FEATURE_CAN         = (1 << 5),
    MAVLINK_HAL_FEATURE_SPI         = (1 << 6),
    MAVLINK_HAL_FEATURE_I2C         = (1 << 7),
    MAVLINK_HAL_FEATURE_DMA         = (1 << 8),
    MAVLINK_HAL_FEATURE_MUTEX       = (1 << 9),
    MAVLINK_HAL_FEATURE_IRQ_SAFE    = (1 << 10),
} mavlink_hal_features_t;

/* ============================================================================
 * Handle Types
 * ============================================================================ */

/**
 * @brief Generic handle type for hardware resources
 */
typedef void* mavlink_hal_handle_t;

/**
 * @brief UART handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_uart_handle_t;

/**
 * @brief GPIO pin handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_gpio_handle_t;

/**
 * @brief Timer handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_timer_handle_t;

/**
 * @brief PWM channel handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_pwm_handle_t;

/**
 * @brief ADC channel handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_adc_handle_t;

/**
 * @brief CAN bus handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_can_handle_t;

/**
 * @brief SPI bus handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_spi_handle_t;

/**
 * @brief I2C bus handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_i2c_handle_t;

/**
 * @brief Mutex handle type
 */
typedef mavlink_hal_handle_t mavlink_hal_mutex_handle_t;

/* ============================================================================
 * Configuration Types
 * ============================================================================ */

/**
 * @brief UART configuration
 */
typedef struct {
    uint32_t baudrate;              /**< Baud rate (e.g., 115200) */
    uint8_t  data_bits;             /**< Data bits (7 or 8) */
    uint8_t  stop_bits;             /**< Stop bits (1 or 2) */
    uint8_t  parity;                /**< Parity: 0=none, 1=odd, 2=even */
    bool     flow_control;          /**< Hardware flow control enabled */
    uint16_t rx_buffer_size;        /**< RX buffer size in bytes */
    uint16_t tx_buffer_size;        /**< TX buffer size in bytes */
} mavlink_hal_uart_config_t;

/**
 * @brief GPIO mode
 */
typedef enum {
    MAVLINK_HAL_GPIO_MODE_INPUT         = 0,
    MAVLINK_HAL_GPIO_MODE_OUTPUT        = 1,
    MAVLINK_HAL_GPIO_MODE_INPUT_PULLUP  = 2,
    MAVLINK_HAL_GPIO_MODE_INPUT_PULLDOWN = 3,
    MAVLINK_HAL_GPIO_MODE_OUTPUT_OD     = 4,  /**< Open-drain */
} mavlink_hal_gpio_mode_t;

/**
 * @brief GPIO configuration
 */
typedef struct {
    uint16_t                 pin;       /**< Platform-specific pin number */
    mavlink_hal_gpio_mode_t  mode;      /**< GPIO mode */
    bool                     initial_state; /**< Initial output state (if output) */
} mavlink_hal_gpio_config_t;

/**
 * @brief PWM configuration
 */
typedef struct {
    uint32_t frequency_hz;      /**< PWM frequency in Hz */
    uint16_t resolution_bits;   /**< PWM resolution (8, 10, 12, 16 bits) */
    uint16_t initial_duty;      /**< Initial duty cycle (0-max) */
} mavlink_hal_pwm_config_t;

/**
 * @brief ADC configuration
 */
typedef struct {
    uint8_t  channel;           /**< ADC channel number */
    uint16_t resolution_bits;   /**< ADC resolution (8, 10, 12, 16 bits) */
    uint32_t sampling_time_us;  /**< Sampling time in microseconds */
    float    reference_voltage; /**< Reference voltage (e.g., 3.3V) */
} mavlink_hal_adc_config_t;

/**
 * @brief CAN configuration
 */
typedef struct {
    uint32_t baudrate;          /**< CAN baudrate (e.g., 500000) */
    bool     loopback_mode;     /**< Loopback mode for testing */
    bool     silent_mode;       /**< Silent mode (listen only) */
    uint8_t  tx_fifo_depth;     /**< TX FIFO depth */
    uint8_t  rx_fifo_depth;     /**< RX FIFO depth */
} mavlink_hal_can_config_t;

/**
 * @brief CAN message
 */
typedef struct {
    uint32_t id;                /**< CAN identifier (11-bit or 29-bit) */
    bool     extended_id;       /**< Extended ID flag */
    bool     remote_frame;      /**< Remote frame flag */
    uint8_t  dlc;               /**< Data length code (0-8) */
    uint8_t  data[8];           /**< Data bytes */
} mavlink_hal_can_msg_t;

/**
 * @brief SPI configuration
 */
typedef struct {
    uint32_t clock_speed_hz;    /**< SPI clock speed */
    uint8_t  mode;              /**< SPI mode (0-3) */
    uint8_t  bit_order;         /**< 0=MSB first, 1=LSB first */
    uint8_t  cs_pin;            /**< Chip select pin */
} mavlink_hal_spi_config_t;

/**
 * @brief I2C configuration
 */
typedef struct {
    uint32_t clock_speed_hz;    /**< I2C clock speed (100kHz, 400kHz) */
    uint8_t  address_bits;      /**< 7 or 10 bit addressing */
    bool     general_call;      /**< General call enable */
} mavlink_hal_i2c_config_t;

/* ============================================================================
 * Time Types
 * ============================================================================ */

/**
 * @brief Timestamp type (microseconds)
 */
typedef uint64_t mavlink_hal_timestamp_us_t;

/**
 * @brief Timeout type (milliseconds)
 */
typedef uint32_t mavlink_hal_timeout_ms_t;

/**
 * @brief Special timeout values
 */
#define MAVLINK_HAL_TIMEOUT_INFINITE    0xFFFFFFFFUL
#define MAVLINK_HAL_TIMEOUT_IMMEDIATE   0UL

/* ============================================================================
 * Callback Types
 * ============================================================================ */

/**
 * @brief UART RX callback
 * @param data Received data pointer
 * @param length Number of bytes received
 * @param user_data User-provided context
 */
typedef void (*mavlink_hal_uart_rx_callback_t)(const uint8_t* data, size_t length, void* user_data);

/**
 * @brief UART TX complete callback
 * @param bytes_sent Number of bytes transmitted
 * @param user_data User-provided context
 */
typedef void (*mavlink_hal_uart_tx_callback_t)(size_t bytes_sent, void* user_data);

/**
 * @brief GPIO interrupt callback
 * @param pin Pin number that triggered the interrupt
 * @param user_data User-provided context
 */
typedef void (*mavlink_hal_gpio_irq_callback_t)(uint16_t pin, void* user_data);

/**
 * @brief Timer callback
 * @param user_data User-provided context
 */
typedef void (*mavlink_hal_timer_callback_t)(void* user_data);

/**
 * @brief CAN RX callback
 * @param msg Received CAN message
 * @param user_data User-provided context
 */
typedef void (*mavlink_hal_can_rx_callback_t)(const mavlink_hal_can_msg_t* msg, void* user_data);

/* ============================================================================
 * Utility Macros
 * ============================================================================ */

/**
 * @brief Check if error code indicates success
 */
#define MAVLINK_HAL_IS_OK(err)      ((err) == MAVLINK_HAL_OK)

/**
 * @brief Check if error code indicates failure
 */
#define MAVLINK_HAL_IS_ERROR(err)   ((err) != MAVLINK_HAL_OK)

/**
 * @brief Convert duty cycle percentage to raw value
 * @param percent Percentage (0.0 - 100.0)
 * @param max_value Maximum raw value
 */
#define MAVLINK_HAL_DUTY_PERCENT_TO_RAW(percent, max_value) \
    ((uint16_t)((percent) * (max_value) / 100.0f))

/**
 * @brief Convert raw ADC value to voltage
 * @param raw Raw ADC value
 * @param resolution Resolution in bits
 * @param vref Reference voltage
 */
#define MAVLINK_HAL_ADC_RAW_TO_VOLTAGE(raw, resolution, vref) \
    ((float)(raw) * (vref) / (float)((1UL << (resolution)) - 1))

/* ============================================================================
 * Feature Detection Macros
 * ============================================================================ */

/**
 * @brief Check if a feature is supported
 */
#define MAVLINK_HAL_HAS_FEATURE(features, feature) \
    (((features) & (feature)) != 0)

/* ============================================================================
 * Constants
 * ============================================================================ */

/**
 * @brief Maximum string length for debug messages
 */
#define MAVLINK_HAL_MAX_DEBUG_MSG_LEN   128

/**
 * @brief Maximum number of callbacks per peripheral
 */
#define MAVLINK_HAL_MAX_CALLBACKS       4

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_HAL_TYPES_H */
