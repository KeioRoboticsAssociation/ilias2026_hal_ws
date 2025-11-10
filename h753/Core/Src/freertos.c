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
#include "stm32h7xx_nucleo.h"
#include "../../App/comm/mavlink_udp.h"
#include "../../App/hal/hardware_manager.h"
#include "../../App/motors/motor_registry.h"
#include "../../App/motors/robomaster_controller.h"
#include "../../App/config/parameter_manager.h"
#include "fdcan.h"
#include "usart.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern motor_registry_t g_motor_registry;
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
      // Handle parameter request list - send all parameters
      uint16_t param_count = param_manager_get_count();

      for (uint16_t i = 0; i < param_count; i++) {
        const param_entry_t* param = param_manager_get_by_index(i);
        if (param) {
          mavlink_message_t response;
          mavlink_msg_param_value_pack(
            handler->config.system_id,
            handler->config.component_id,
            &response,
            param->name,
            *(param->value_ptr),
            MAV_PARAM_TYPE_REAL32,
            param_count,
            i
          );
          mavlink_udp_send_message(handler, &response);

          // Small delay to avoid flooding the network
          osDelay(5);
        }
      }
      break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
      // Handle parameter request read - send specific parameter
      mavlink_param_request_read_t req;
      mavlink_msg_param_request_read_decode(msg, &req);

      const param_entry_t* param = NULL;

      // Check if request is by index or by name
      if (req.param_index >= 0) {
        param = param_manager_get_by_index((uint16_t)req.param_index);
      } else {
        // Request by name
        char param_name[17] = {0};
        strncpy(param_name, req.param_id, 16);
        param = param_manager_get_by_name(param_name);
      }

      if (param) {
        mavlink_message_t response;
        uint16_t param_count = param_manager_get_count();
        int16_t param_index = param_manager_get_index_by_name(param->name);

        mavlink_msg_param_value_pack(
          handler->config.system_id,
          handler->config.component_id,
          &response,
          param->name,
          *(param->value_ptr),
          MAV_PARAM_TYPE_REAL32,
          param_count,
          param_index
        );
        mavlink_udp_send_message(handler, &response);
      }
      break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:
    {
      // Handle parameter set - update parameter value
      mavlink_param_set_t set_req;
      mavlink_msg_param_set_decode(msg, &set_req);

      char param_name[17] = {0};
      strncpy(param_name, set_req.param_id, 16);

      // Attempt to set parameter
      error_code_t err = param_manager_set(param_name, set_req.param_value);

      // Always send back the current value (MAVLink protocol requirement)
      const param_entry_t* param = param_manager_get_by_name(param_name);
      if (param) {
        mavlink_message_t response;
        uint16_t param_count = param_manager_get_count();
        int16_t param_index = param_manager_get_index_by_name(param->name);

        mavlink_msg_param_value_pack(
          handler->config.system_id,
          handler->config.component_id,
          &response,
          param->name,
          *(param->value_ptr),  // Send actual current value (may be clamped)
          MAV_PARAM_TYPE_REAL32,
          param_count,
          param_index
        );
        mavlink_udp_send_message(handler, &response);
      }

      (void)err;  // Suppress unused warning
      break;
    }

    case MAVLINK_MSG_ID_MOTOR_COMMAND:
    {
      if (!motors_initialized) break;

      // Handle generic motor command for RS485 and extended motor IDs
      mavlink_motor_command_t motor_cmd_msg;
      mavlink_msg_motor_command_decode(msg, &motor_cmd_msg);

      motor_command_t cmd = {0};
      cmd.motor_id = motor_cmd_msg.motor_id;
      cmd.mode = (control_mode_t)motor_cmd_msg.control_mode;
      cmd.target_value = motor_cmd_msg.target_value;
      cmd.enable = motor_cmd_msg.enable;
      cmd.timestamp = HAL_GetTick();

      // Send command to motor registry
      motor_registry_send_command(cmd.motor_id, &cmd);
      break;
    }

    default:
      // Unknown message, ignore or log
      break;
  }
}

/**
  * @brief  FDCAN RX FIFO0 callback - processes incoming CAN messages from RoboMaster motors
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: Interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // Retrieve message from RX FIFO0
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
      // Route CAN message to appropriate RoboMaster motor controller
      // We need to find which motor matches this CAN ID
      if (motors_initialized)
      {
        // Search through all motors to find matching RoboMaster controller
        for (uint8_t motor_id = 20; motor_id < 30; motor_id++)  // RoboMaster IDs: 20-29
        {
          motor_controller_t* controller = motor_registry_get(motor_id);
          if (controller && controller->type == MOTOR_TYPE_ROBOMASTER)
          {
            // Pass message to RoboMaster controller for processing
            robomaster_process_can_message(controller, rx_header.Identifier, rx_data, 8);
          }
        }
      }
    }
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
  // Initialize BSP LEDs
  BSP_LED_Init(LED1);  // Green LED
  BSP_LED_Init(LED2);  // Blue LED
  BSP_LED_Init(LED3);  // Red LED

  // LED startup sequence - 3 blinks on LED1 (Green)
  for (int i = 0; i < 3; i++) {
    BSP_LED_On(LED1);
    osDelay(100);
    BSP_LED_Off(LED1);
    osDelay(100);
  }

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
    // Initialization failed, blink all LEDs rapidly to indicate error
    while(1) {
      BSP_LED_On(LED1);
      BSP_LED_On(LED2);
      BSP_LED_On(LED3);
      osDelay(100);
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      osDelay(100);
    }
  }

  // Initialize hardware manager
  error_code_t hw_err = hw_manager_init();
  if (hw_err != ERROR_OK) {
    // HW manager init failed - blink RED LED rapidly
    while(1) {
      BSP_LED_On(LED3);
      osDelay(50);
      BSP_LED_Off(LED3);
      osDelay(50);
    }
  }

  // Register and start FDCAN1 for RoboMaster motors
  hw_err = hw_can_register(&hfdcan1);
  if (hw_err != ERROR_OK) {
    // CAN registration failed - blink RED and GREEN alternately
    while(1) {
      BSP_LED_On(LED3);
      osDelay(75);
      BSP_LED_Off(LED3);
      BSP_LED_On(LED1);
      osDelay(75);
      BSP_LED_Off(LED1);
    }
  }

  // Configure RoboMaster CAN filter (accept 0x201-0x20B for both M3508 and GM6020)
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x201;  // M3508 motor 1 / GM6020 range start
  sFilterConfig.FilterID2 = 0x20B;  // GM6020 motor 7
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // Start FDCAN peripheral
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  // Enable FDCAN RX FIFO0 interrupt to receive motor feedback
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }

  // Register UART handles for RS485 motors
  hw_err = hw_uart_register(1, &huart1);  // USART1
  if (hw_err != ERROR_OK) {
    // UART1 registration failed - blink BLUE LED
    while(1) {
      BSP_LED_On(LED2);
      osDelay(60);
      BSP_LED_Off(LED2);
      osDelay(60);
    }
  }

  hw_err = hw_uart_register(2, &huart2);  // USART2
  if (hw_err != ERROR_OK) {
    // UART2 registration failed - blink BLUE and RED LEDs
    while(1) {
      BSP_LED_On(LED2);
      BSP_LED_On(LED3);
      osDelay(60);
      BSP_LED_Off(LED2);
      BSP_LED_Off(LED3);
      osDelay(60);
    }
  }

  // Initialize motor registry
  hw_err = motor_registry_init();
  if (hw_err != ERROR_OK) {
    // Motor registry init failed - slow blink RED LED
    while(1) {
      BSP_LED_On(LED3);
      osDelay(200);
      BSP_LED_Off(LED3);
      osDelay(200);
    }
  }

  // Create all motors from configuration
  // Note: Motors without proper hardware (missing timers/UARTs) will be skipped automatically
  // The system will work with whatever motors successfully initialize
  hw_err = motor_registry_create_all_motors();
  // Always enable motor system - it will work with successfully initialized motors only
  motors_initialized = true;

  // Log motor count for diagnostics
  if (g_motor_registry.motor_count > 0) {
    // At least some motors initialized successfully
  } else {
    // No motors initialized - check hardware connections and CubeMX configuration
  }

  // Initialize parameter manager (for PID tuning via MAVLink)
  hw_err = param_manager_init();
  if (hw_err == ERROR_OK && motors_initialized) {
    // Register all motor PID parameters
    hw_err = param_manager_register_motor_params();

    // Try to load saved parameters from Flash
    // If no valid data exists, defaults will be used
    param_manager_load_from_flash();
  }

  // Variables for timing
  uint32_t last_heartbeat_time = 0;
  uint32_t last_motor_update_time = 0;
  uint32_t last_motor_status_time = 0;
  const uint32_t heartbeat_interval = 1000; // 1 Hz heartbeat
  const uint32_t motor_update_interval = 10; // 100 Hz motor update
  const uint32_t motor_status_interval = 100; // 10 Hz motor status broadcast

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

      // Toggle GREEN LED to indicate heartbeat sent
      BSP_LED_Toggle(LED1);
    }

    // Broadcast motor status at 10 Hz (for ROS2 visualization)
    if (motors_initialized && (current_time - last_motor_status_time >= motor_status_interval)) {
      // Send status for all active RoboMaster motors (IDs 20-29)
      for (uint8_t motor_id = 20; motor_id < 30; motor_id++) {
        motor_controller_t* controller = motor_registry_get(motor_id);
        if (controller && controller->type == MOTOR_TYPE_ROBOMASTER) {
          motor_state_t state = motor_get_state(controller);

          // Send ROBOMASTER_MOTOR_STATUS message
          mavlink_message_t msg;
          mavlink_msg_robomaster_motor_status_pack(
            mavlink_handler.config.system_id,
            mavlink_handler.config.component_id,
            &msg,
            motor_id,                          // motor_id
            0,                                 // control_mode (0=position, 1=velocity, 2=current)
            (uint8_t)state.status,             // status
            state.current_position,            // current_position_rad
            state.current_velocity,            // current_speed_rad_s
            0.0f,                              // current_duty_cycle
            0.0f,                              // position_error_rad
            current_time                       // timestamp_ms
          );
          mavlink_udp_send_message(&mavlink_handler, &msg);
        }
      }
      last_motor_status_time = current_time;
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
