/**
 * @file hardware_manager.c
 * @brief Hardware Abstraction Layer implementation for H753
 */

#include "hardware_manager.h"
#include <string.h>

/* ============================================================================
 * Global Variables
 * ============================================================================ */
hardware_manager_t g_hw_manager = {0};

/* ============================================================================
 * Private Helper Functions
 * ============================================================================ */

/**
 * @brief Convert timer ID to array index
 */
static inline int timer_id_to_index(uint8_t timer_id) {
    if (timer_id >= 1 && timer_id <= 4) {
        return timer_id - 1;
    }
    return -1;
}

/**
 * @brief Get CCR register pointer for a timer channel
 */
static inline volatile uint32_t* get_ccr_register(TIM_HandleTypeDef* htim, uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1: return &htim->Instance->CCR1;
        case TIM_CHANNEL_2: return &htim->Instance->CCR2;
        case TIM_CHANNEL_3: return &htim->Instance->CCR3;
        case TIM_CHANNEL_4: return &htim->Instance->CCR4;
        default: return NULL;
    }
}

/* ============================================================================
 * Hardware Manager API Implementation
 * ============================================================================ */

error_code_t hw_manager_init(void) {
    if (g_hw_manager.initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    memset(&g_hw_manager, 0, sizeof(hardware_manager_t));
    g_hw_manager.initialized = true;

    return ERROR_OK;
}

bool hw_manager_is_initialized(void) {
    return g_hw_manager.initialized;
}

uint32_t hw_get_tick(void) {
    return HAL_GetTick();
}

void hw_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/* ============================================================================
 * Timer API Implementation
 * ============================================================================ */

error_code_t hw_timer_register(uint8_t timer_id, TIM_HandleTypeDef* htim) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0 || !htim) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (timer->initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    timer->htim = htim;
    timer->period = htim->Init.Period;
    timer->prescaler = htim->Init.Prescaler;
    timer->initialized = true;

    return ERROR_OK;
}

error_code_t hw_timer_start_pwm(uint8_t timer_id, uint32_t channel) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized || !timer->htim) {
        return ERROR_NOT_INITIALIZED;
    }

    if (HAL_TIM_PWM_Start(timer->htim, channel) != HAL_OK) {
        return ERROR_HARDWARE_ERROR;
    }

    return ERROR_OK;
}

error_code_t hw_timer_stop_pwm(uint8_t timer_id, uint32_t channel) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized || !timer->htim) {
        return ERROR_NOT_INITIALIZED;
    }

    if (HAL_TIM_PWM_Stop(timer->htim, channel) != HAL_OK) {
        return ERROR_HARDWARE_ERROR;
    }

    return ERROR_OK;
}

error_code_t hw_timer_set_pwm(uint8_t timer_id, uint32_t channel, uint32_t pulse) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized || !timer->htim) {
        return ERROR_NOT_INITIALIZED;
    }

    // Constrain pulse to timer period
    if (pulse > timer->period) {
        pulse = timer->period;
    }

    volatile uint32_t* ccr = get_ccr_register(timer->htim, channel);
    if (!ccr) {
        return ERROR_INVALID_PARAMETER;
    }

    *ccr = pulse;

    return ERROR_OK;
}

error_code_t hw_timer_set_pwm_us(uint8_t timer_id, uint32_t channel, uint32_t pulse_us) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized || !timer->htim) {
        return ERROR_NOT_INITIALIZED;
    }

    // Calculate timer frequency
    // For H753: Timer clock can be up to 240 MHz (or 120 MHz depending on APB)
    // Timer frequency = Timer clock / (Prescaler + 1)
    // For standard servo PWM at 50Hz: Period should be 20ms = 20000us

    // Assume timer is configured with 1MHz tick (1us resolution)
    // This is typically: Prescaler = (Timer_Clock / 1MHz) - 1
    // If you have different timer setup, adjust calculation accordingly

    uint32_t pulse_ticks = pulse_us;  // Assuming 1us per tick

    return hw_timer_set_pwm(timer_id, channel, pulse_ticks);
}

uint32_t hw_timer_get_period(uint8_t timer_id) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return 0;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized) {
        return 0;
    }

    return timer->period;
}

uint32_t hw_timer_get_frequency(uint8_t timer_id) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return 0;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized || !timer->htim) {
        return 0;
    }

    // Calculate timer frequency
    // Frequency = Timer_Clock / ((Prescaler + 1) * (Period + 1))
    // This is approximate - actual calculation depends on system clock configuration

    return 1000000;  // Placeholder: Assume 1MHz for servo PWM
}

TIM_HandleTypeDef* hw_timer_get_handle(uint8_t timer_id) {
    int idx = timer_id_to_index(timer_id);
    if (idx < 0) {
        return NULL;
    }

    hw_timer_t* timer = &g_hw_manager.timers[idx];
    if (!timer->initialized) {
        return NULL;
    }

    return timer->htim;
}

/* ============================================================================
 * GPIO API Implementation
 * ============================================================================ */

bool hw_gpio_read(GPIO_TypeDef* port, uint16_t pin) {
    if (!port) {
        return false;
    }
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
}

void hw_gpio_write(GPIO_TypeDef* port, uint16_t pin, bool state) {
    if (!port) {
        return;
    }
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hw_gpio_toggle(GPIO_TypeDef* port, uint16_t pin) {
    if (!port) {
        return;
    }
    HAL_GPIO_TogglePin(port, pin);
}

/* ============================================================================
 * CAN API Implementation (FDCAN for STM32H7)
 * ============================================================================ */

error_code_t hw_can_register(FDCAN_HandleTypeDef* hfdcan) {
    if (!hfdcan) {
        return ERROR_INVALID_PARAMETER;
    }

    if (g_hw_manager.can.initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    g_hw_manager.can.hfdcan = hfdcan;
    g_hw_manager.can.initialized = true;

    return ERROR_OK;
}

error_code_t hw_can_transmit(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (!g_hw_manager.can.initialized || !g_hw_manager.can.hfdcan) {
        return ERROR_NOT_INITIALIZED;
    }

    if (!data || length > 8) {
        return ERROR_INVALID_PARAMETER;
    }

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = length << 16;  // FDCAN_DLC_BYTES_X format
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(g_hw_manager.can.hfdcan, &tx_header, (uint8_t*)data) != HAL_OK) {
        return ERROR_HARDWARE_ERROR;
    }

    return ERROR_OK;
}

bool hw_can_tx_available(void) {
    if (!g_hw_manager.can.initialized || !g_hw_manager.can.hfdcan) {
        return false;
    }

    return (HAL_FDCAN_GetTxFifoFreeLevel(g_hw_manager.can.hfdcan) > 0);
}

/* ============================================================================
 * UART/RS485 API Implementation
 * ============================================================================ */

static inline int uart_id_to_index(uint8_t uart_id) {
    switch (uart_id) {
        case 1: return 0;  // USART1
        case 2: return 1;  // USART2
        case 6: return 2;  // USART6
        default: return -1;
    }
}

error_code_t hw_uart_register(uint8_t uart_id, UART_HandleTypeDef* huart) {
    int idx = uart_id_to_index(uart_id);
    if (idx < 0 || !huart) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_uart_t* uart = &g_hw_manager.uarts[idx];
    if (uart->initialized) {
        return ERROR_ALREADY_INITIALIZED;
    }

    uart->huart = huart;
    uart->uart_id = uart_id;
    uart->initialized = true;

    return ERROR_OK;
}

error_code_t hw_uart_transmit(uint8_t uart_id, const uint8_t* data, uint16_t size) {
    int idx = uart_id_to_index(uart_id);
    if (idx < 0 || !data || size == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_uart_t* uart = &g_hw_manager.uarts[idx];
    if (!uart->initialized || !uart->huart) {
        return ERROR_NOT_INITIALIZED;
    }

    // Blocking transmit with 100ms timeout
    if (HAL_UART_Transmit(uart->huart, (uint8_t*)data, size, 100) != HAL_OK) {
        return ERROR_HARDWARE_ERROR;
    }

    return ERROR_OK;
}

error_code_t hw_uart_receive(uint8_t uart_id, uint8_t* data, uint16_t size, uint32_t timeout_ms) {
    int idx = uart_id_to_index(uart_id);
    if (idx < 0 || !data || size == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_uart_t* uart = &g_hw_manager.uarts[idx];
    if (!uart->initialized || !uart->huart) {
        return ERROR_NOT_INITIALIZED;
    }

    // Blocking receive with specified timeout
    HAL_StatusTypeDef status = HAL_UART_Receive(uart->huart, data, size, timeout_ms);
    if (status == HAL_TIMEOUT) {
        return ERROR_TIMEOUT;
    } else if (status != HAL_OK) {
        return ERROR_HARDWARE_ERROR;
    }

    return ERROR_OK;
}

error_code_t hw_uart_flush(uint8_t uart_id) {
    int idx = uart_id_to_index(uart_id);
    if (idx < 0) {
        return ERROR_INVALID_PARAMETER;
    }

    hw_uart_t* uart = &g_hw_manager.uarts[idx];
    if (!uart->initialized || !uart->huart) {
        return ERROR_NOT_INITIALIZED;
    }

    // Abort any ongoing reception
    HAL_UART_AbortReceive(uart->huart);

    return ERROR_OK;
}
