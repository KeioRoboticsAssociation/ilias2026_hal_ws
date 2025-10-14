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

  // Variables for heartbeat timing
  uint32_t last_heartbeat_time = 0;
  const uint32_t heartbeat_interval = 1000; // 1 Hz heartbeat

  /* Infinite loop */
  for(;;)
  {
    uint32_t current_time = HAL_GetTick();

    // Send heartbeat at 1 Hz
    if (current_time - last_heartbeat_time >= heartbeat_interval) {
      mavlink_udp_send_heartbeat(&mavlink_handler,
                                  MAV_TYPE_ONBOARD_CONTROLLER,
                                  MAV_AUTOPILOT_INVALID,
                                  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                  MAV_STATE_ACTIVE);
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
