/**
 * @file hardware_manager.h
 * @brief Hardware Abstraction Layer for H753 motor control
 *
 * Provides unified interface to STM32 HAL for timer, GPIO, CAN access.
 */

#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "../config/motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Hardware Manager Structures
 * ============================================================================ */

/**
 * @brief Timer handle wrapper
 */
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    bool initialized;
    uint32_t period;        // Timer period (auto-reload value)
    uint32_t prescaler;     // Timer prescaler
} hw_timer_t;

/**
 * @brief CAN handle wrapper (FDCAN for STM32H7)
 */
typedef struct {
    FDCAN_HandleTypeDef* hfdcan;
    bool initialized;
} hw_can_t;

/**
 * @brief GPIO handle wrapper
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    bool active_low;
} hw_gpio_t;

/**
 * @brief UART handle wrapper (for RS485)
 */
typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t uart_id;            // 1=USART1, 2=USART2, 6=USART6
    bool initialized;
} hw_uart_t;

/**
 * @brief Hardware manager main structure
 */
typedef struct {
    hw_timer_t timers[4];       // TIM1-TIM4 (index 0-3)
    hw_can_t can;               // CAN interface
    hw_uart_t uarts[3];         // USART1/2/6 (index 0/1/2)
    bool initialized;
} hardware_manager_t;

/* ============================================================================
 * Global Hardware Manager Instance
 * ============================================================================ */
extern hardware_manager_t g_hw_manager;

/* ============================================================================
 * Hardware Manager API
 * ============================================================================ */

/**
 * @brief Initialize hardware manager
 * @return Error code
 */
error_code_t hw_manager_init(void);

/**
 * @brief Check if hardware manager is initialized
 * @return true if initialized
 */
bool hw_manager_is_initialized(void);

/**
 * @brief Get system tick in milliseconds
 * @return Current tick count
 */
uint32_t hw_get_tick(void);

/**
 * @brief Delay for specified milliseconds
 * @param ms Delay in milliseconds
 */
void hw_delay_ms(uint32_t ms);

/* ============================================================================
 * Timer API
 * ============================================================================ */

/**
 * @brief Register a timer handle
 * @param timer_id Timer ID (1-4 for TIM1-TIM4)
 * @param htim Pointer to HAL timer handle
 * @return Error code
 */
error_code_t hw_timer_register(uint8_t timer_id, TIM_HandleTypeDef* htim);

/**
 * @brief Start PWM on timer channel
 * @param timer_id Timer ID (1-4)
 * @param channel Timer channel (TIM_CHANNEL_1, etc.)
 * @return Error code
 */
error_code_t hw_timer_start_pwm(uint8_t timer_id, uint32_t channel);

/**
 * @brief Stop PWM on timer channel
 * @param timer_id Timer ID (1-4)
 * @param channel Timer channel
 * @return Error code
 */
error_code_t hw_timer_stop_pwm(uint8_t timer_id, uint32_t channel);

/**
 * @brief Set PWM duty cycle
 * @param timer_id Timer ID (1-4)
 * @param channel Timer channel
 * @param pulse Pulse width in timer ticks
 * @return Error code
 */
error_code_t hw_timer_set_pwm(uint8_t timer_id, uint32_t channel, uint32_t pulse);

/**
 * @brief Set PWM pulse width in microseconds
 * @param timer_id Timer ID (1-4)
 * @param channel Timer channel
 * @param pulse_us Pulse width in microseconds
 * @return Error code
 */
error_code_t hw_timer_set_pwm_us(uint8_t timer_id, uint32_t channel, uint32_t pulse_us);

/**
 * @brief Get timer period
 * @param timer_id Timer ID (1-4)
 * @return Timer period in ticks, or 0 on error
 */
uint32_t hw_timer_get_period(uint8_t timer_id);

/**
 * @brief Get timer frequency
 * @param timer_id Timer ID (1-4)
 * @return Timer frequency in Hz, or 0 on error
 */
uint32_t hw_timer_get_frequency(uint8_t timer_id);

/**
 * @brief Get pointer to timer handle
 * @param timer_id Timer ID (1-4)
 * @return Pointer to timer handle or NULL
 */
TIM_HandleTypeDef* hw_timer_get_handle(uint8_t timer_id);

/* ============================================================================
 * GPIO API
 * ============================================================================ */

/**
 * @brief Read GPIO pin
 * @param port GPIO port
 * @param pin GPIO pin
 * @return Pin state (true = high, false = low)
 */
bool hw_gpio_read(GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief Write GPIO pin
 * @param port GPIO port
 * @param pin GPIO pin
 * @param state Pin state to write
 */
void hw_gpio_write(GPIO_TypeDef* port, uint16_t pin, bool state);

/**
 * @brief Toggle GPIO pin
 * @param port GPIO port
 * @param pin GPIO pin
 */
void hw_gpio_toggle(GPIO_TypeDef* port, uint16_t pin);

/* ============================================================================
 * CAN API
 * ============================================================================ */

/**
 * @brief Register CAN handle (FDCAN for STM32H7)
 * @param hfdcan Pointer to HAL FDCAN handle
 * @return Error code
 */
error_code_t hw_can_register(FDCAN_HandleTypeDef* hfdcan);

/**
 * @brief Transmit CAN message
 * @param can_id CAN identifier
 * @param data Pointer to data buffer (8 bytes max)
 * @param length Data length (0-8)
 * @return Error code
 */
error_code_t hw_can_transmit(uint32_t can_id, const uint8_t* data, uint8_t length);

/**
 * @brief Check if CAN TX mailbox is available
 * @return true if mailbox available
 */
bool hw_can_tx_available(void);

/* ============================================================================
 * UART/RS485 API
 * ============================================================================ */

/**
 * @brief Register UART handle for RS485 communication
 * @param uart_id UART peripheral ID (1=USART1, 2=USART2, 6=USART6)
 * @param huart Pointer to HAL UART handle
 * @return Error code
 */
error_code_t hw_uart_register(uint8_t uart_id, UART_HandleTypeDef* huart);

/**
 * @brief Transmit data via UART (blocking)
 * @param uart_id UART peripheral ID (1, 2, or 6)
 * @param data Pointer to data buffer
 * @param size Number of bytes to transmit
 * @return Error code
 */
error_code_t hw_uart_transmit(uint8_t uart_id, const uint8_t* data, uint16_t size);

/**
 * @brief Receive data via UART (blocking with timeout)
 * @param uart_id UART peripheral ID (1, 2, or 6)
 * @param data Pointer to receive buffer
 * @param size Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 * @return Error code
 */
error_code_t hw_uart_receive(uint8_t uart_id, uint8_t* data, uint16_t size, uint32_t timeout_ms);

/**
 * @brief Flush UART receive buffer
 * @param uart_id UART peripheral ID (1, 2, or 6)
 * @return Error code
 */
error_code_t hw_uart_flush(uint8_t uart_id);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_MANAGER_H */
