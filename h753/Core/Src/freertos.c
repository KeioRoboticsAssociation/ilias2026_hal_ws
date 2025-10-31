/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/udp.h"
#include <string.h>
#include <stdio.h>
#include "../../App/comm/mavlink_udp.h"
#include "../../App/hal/hardware_manager.h"
#include "../../App/motors/motor_registry.h"
#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// MAVLink UDP handler
static mavlink_udp_t mavlink_handler;

// Motor system initialized flag
static bool motors_initialized = false;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
  * @brief  MAVLink message handler callback
  * @param  msg: Received MAVLink message
  * @param  handler: MAVLink UDP handler instance
  * @retval None
  */
void mavlink_message_handler(const mavlink_message_t *msg, mavlink_udp_t *handler)
{
  // Handle different MAVLink message types
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      // Handle heartbeat message
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);
      // Echo back heartbeat or process as needed
      break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
      if (!motors_initialized) break;

      // Handle RC channels override for motor control
      mavlink_rc_channels_override_t rc_override;
      mavlink_msg_rc_channels_override_decode(msg, &rc_override);

      // Map RC channels to motors (channels 1-8, typically 1000-2000 us PWM)
      // Channel 1 -> Motor ID 1, Channel 2 -> Motor ID 2, etc.
      uint16_t channels[8] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw,
        rc_override.chan4_raw, rc_override.chan5_raw, rc_override.chan6_raw,
        rc_override.chan7_raw, rc_override.chan8_raw
      };

      for (int i = 0; i < 8; i++) {
        if (channels[i] == UINT16_MAX || channels[i] == 0) {
          continue;  // Channel not set
        }

        motor_command_t cmd = {0};
        cmd.motor_id = i + 1;  // Motor IDs 1-8
        cmd.timestamp = HAL_GetTick();
        cmd.enable = (channels[i] > 900 && channels[i] < 2100);

        // Convert PWM to appropriate control value
        // For servos: map 1000-2000us to -90 to +90 degrees
        // For DC motors: map 1000-2000us to -1.0 to +1.0 duty cycle
        float normalized = (float)(channels[i] - 1500) / 500.0f;  // -1.0 to +1.0

        // Check motor type and set appropriate command
        motor_controller_t* controller = motor_registry_get(cmd.motor_id);
        if (controller) {
          if (controller->type == MOTOR_TYPE_SERVO) {
            cmd.mode = CONTROL_MODE_POSITION;
            cmd.target_value = normalized * 90.0f;  // -90 to +90 degrees
          } else {
            cmd.mode = CONTROL_MODE_DUTY_CYCLE;
            cmd.target_value = normalized;  // -1.0 to +1.0
          }

          motor_registry_send_command(cmd.motor_id, &cmd);
        }
      }
      break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
      if (!motors_initialized) break;

      // Handle manual control (joystick-like inputs)
      mavlink_manual_control_t manual;
      mavlink_msg_manual_control_decode(msg, &manual);

      // manual.x, manual.y, manual.z, manual.r are normalized -1000 to 1000
      // Map to motors as needed for your application
      break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
      // Handle parameter request list
      // Send back parameter list if needed
      break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
      // Handle parameter request read
      break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:
    {
      // Handle parameter set
      break;
    }

    default:
      // Unknown message, ignore or log
      break;
  }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  // LED startup sequence
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  // Configure MAVLink UDP
  mavlink_udp_config_t config;
  config.system_id = 1;
  config.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
  config.local_port = 14550;  // Standard MAVLink port
  config.remote_port = 14550; // Standard MAVLink port
  IP4_ADDR(&config.remote_addr, 192, 168, 11, 2); // Remote PC IP address

  // Initialize MAVLink UDP handler
  err_t err = mavlink_udp_init(&mavlink_handler, &config, mavlink_message_handler);

  if (err != ERR_OK) {
    // Initialization failed, blink LED rapidly to indicate error
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(100);
    }
  }

  // Initialize hardware manager
  error_code_t hw_err = hw_manager_init();
  if (hw_err != ERROR_OK) {
    // HW manager init failed
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(50);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(50);
    }
  }

  // Register and start FDCAN1 for RoboMaster motors
  hw_err = hw_can_register(&hfdcan1);
  if (hw_err != ERROR_OK) {
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(75);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(75);
    }
  }

  // Configure RoboMaster CAN filter (accept 0x201-0x208)
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x201;
  sFilterConfig.FilterID2 = 0x208;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // Start FDCAN peripheral
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  // Initialize motor registry
  hw_err = motor_registry_init();
  if (hw_err != ERROR_OK) {
    // Motor registry init failed
    while(1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      osDelay(200);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      osDelay(200);
    }
  }

  // Create all motors from configuration
  // Note: You need to configure timers in CubeMX and register them first
  // Example: hw_timer_register(1, &htim1); (add htim1, htim2, etc. as extern)
  hw_err = motor_registry_create_all_motors();
  if (hw_err == ERROR_OK) {
    motors_initialized = true;
  }
  // Note: If motors fail to initialize, system continues without motor control

  // Variables for timing
  uint32_t last_heartbeat_time = 0;
  uint32_t last_motor_update_time = 0;
  const uint32_t heartbeat_interval = 1000; // 1 Hz heartbeat
  const uint32_t motor_update_interval = 10; // 100 Hz motor update

  /* Infinite loop */
  for(;;)
  {
    uint32_t current_time = HAL_GetTick();

    // Update motors at 100 Hz
    if (motors_initialized && (current_time - last_motor_update_time >= motor_update_interval)) {
      float delta_time = (current_time - last_motor_update_time) / 1000.0f;
      motor_registry_update_all(delta_time);
      last_motor_update_time = current_time;
    }

    // Send heartbeat at 1 Hz
    if (current_time - last_heartbeat_time >= heartbeat_interval) {
      mavlink_udp_send_heartbeat(&mavlink_handler,
                                  MAV_TYPE_ONBOARD_CONTROLLER,
                                  MAV_AUTOPILOT_INVALID,
                                  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                  motors_initialized ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
      last_heartbeat_time = current_time;

      // Toggle LED to indicate heartbeat sent
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }

    // Process other tasks here...

    // Small delay to prevent busy-waiting
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
