/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t encoder_count = 0;       // Encoder counter value
int16_t encoder_diff = 0;         // Difference in encoder count
uint32_t last_encoder_count = 0;  // Previous encoder count
uint16_t pwm_value = 500;         // PWM duty cycle (0-1000)
//14Jan//uint8_t rx_buffer[RX_BUFFER_SIZE];  // Buffer to store received data
//14Jan//uint8_t rx_index = 0;

char rx_buffer[20]; // Buffer to hold the received string
    float deviation = 0.0f; // Variable to store the parsed float
    uint8_t rx_index = 0;   // Index for tracking received characters

uint32_t readings_count = 0;

/* PID constants */
//#define Kp 50.0  // Proportional gain (adjust as needed)
#define Ki 0  // Integral gain (adjust as needed)
#define Kd 0  // Derivative gain (adjust as needed)
//#define MOTOR_MAX_SPEED 1000  // Maximum motor speed

//#define ENCODER_PPR 2500 // Replace this with the actual pulses per revolution of your encoder
//#define MAX_ROTATIONS 3
#define MAX_ENCODER_COUNT (ENCODER_PPR * MAX_ROTATIONS)

#define ENCODER_PPR 500 // Replace this with the actual pulses per revolution of your encoder
#define MAX_ROTATIONS 1.0f // Maximum rotations for high deviation
#define MIN_ROTATIONS 0.5f // Minimum rotations for low deviation
#define MAX_MOTOR_SPEED 140 // Maximum motor speed
#define MIN_MOTOR_SPEED 40  // Minimum motor speed
#define MAX_DEVIATION 3.0f  // Maximum deviation value for scaling
#define KP 50.0f           // Proportional gain for PID control
#define KI 0
#define KD 10.0f            // Derivative gain for PID control
#define MAX_PID_OUTPUT 255 //MAX_PID_OUTPUT 0 ~ 255
/* Global variables for PID control */
float integral = 0.0;
float previous_error = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Print_Encoder_Data(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void MotorControl(uint8_t command);
void MotorControl(float deviation, int16_t encoder_position);
static void Leaving_App_Handler();
static void Write_RTC_backup_reg(uint32_t reg ,uint32_t data);
//void Motor_Control(int16_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  /* Start TIM3 in Encoder Mode */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // Initialize the buffer
  memset(rx_buffer, 0, sizeof(rx_buffer));

  /* Send welcome message over USART1 */
//char msg[] = "STM32 Encoder and Motor Control Test\n\r";
  char msg[] = "77777777777777777\n\r";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Receive 8 readings
	     // if (readings_count < 8) {
	          uint8_t received_byte;
	          HAL_UART_Receive(&huart2, &received_byte, 1, HAL_MAX_DELAY); // Receive 1 byte
	          HAL_UART_Transmit(&huart2, &received_byte, 1, HAL_MAX_DELAY); // Echo received byte (for debugging)

	          // Check for newline to indicate end of float string
	          if (received_byte == '\n') {
	              rx_buffer[rx_index] = '\0'; // Null-terminate the string

	              // Convert the string to a float
	              if (sscanf(rx_buffer, "%f", &deviation) == 1) {
	                  // Successfully parsed the float

	                  // Read the encoder position
	                  int16_t encoder_position = __HAL_TIM_GET_COUNTER(&htim3);
	                  encoder_position = - encoder_position; // Reverse the encoder reading if necessary

	                  // Control the motor based on the deviation
	                  MotorControl(deviation, encoder_position);
	              }

	              // Reset the buffer and index for the next reception
	              memset(rx_buffer, 0, sizeof(rx_buffer));
	              rx_index = 0;

	              // Increment the readings counter after processing each valid reading
	              readings_count++;
	          } else {
	              // Store the received character in the buffer
	              rx_buffer[rx_index++] = received_byte;

	              // Ensure index does not exceed buffer size
	              if (rx_index >= sizeof(rx_buffer) - 1) {
	                  // Buffer overflow protection (reset if exceeded)
	                  memset(rx_buffer, 0, sizeof(rx_buffer));
	                  rx_index = 0;
	              }
	          }
	    //  } else {
	          // After 8 readings, reset the system (exit the app)
	         // Leaving_App_Handler(); // This will cause a system reset and return control to the bootloader
	     // }
  }
	  /* the code is working well but need adjusmtments in returning the wheel to its straight position

	  uint8_t received_byte;
	         HAL_UART_Receive(&huart2, &received_byte, 1, HAL_MAX_DELAY); // Receive 1 byte
	         //		// Echo the received byte back via USART2 (optional for debugging)
	         		HAL_UART_Transmit(&huart2, &received_byte, 1, HAL_MAX_DELAY);

	         // Check for newline to indicate end of float string
	         if (received_byte == '\n') {
	             rx_buffer[rx_index] = '\0'; // Null-terminate the string

	             // Convert the string to a float
	             if (sscanf(rx_buffer, "%f", &deviation) == 1) {
	                 // Successfully parsed the float

	                 // Read the encoder position
	                 int16_t encoder_position = __HAL_TIM_GET_COUNTER(&htim3);

	                 // Control the motor based on the deviation
	                 MotorControl(deviation, encoder_position);
	             }

	             // Reset the buffer and index for the next reception
	             memset(rx_buffer, 0, sizeof(rx_buffer));
	             rx_index = 0;
	         } else {
	             // Store the received character in the buffer
	             rx_buffer[rx_index++] = received_byte;

	             // Ensure index does not exceed buffer size
	             if (rx_index >= sizeof(rx_buffer) - 1) {
	                 // Buffer overflow protection (reset if exceeded)
	                 memset(rx_buffer, 0, sizeof(rx_buffer));
	                 rx_index = 0;
	             }
	         }
	     }
*/






















//
//	  /* Read the encoder count */
//	    /* IMPORTANTTTTTTTTTTTTTTT
//	  	encoder_count = __HAL_TIM_GET_COUNTER(&htim3);
//	    encoder_diff = encoder_count - last_encoder_count;
//	    last_encoder_count = encoder_count;
//*/
//	    /* Read the deviation from image processing */
//	    float deviation = received_data;  // Obtain deviation (e.g., from camera algorithm)
//
//	    /* Read the encoder position (optional for limiting steering movement) */
//	    int16_t encoder_position = __HAL_TIM_GET_COUNTER(&htim3);
//
//	    /* Control motor based on deviation */
//	    MotorControl(deviation, encoder_position);
//
//	    /* Print encoder data */
//	    Print_Encoder_Data();
//
///*
//	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 300);
//	    HAL_Delay(3000);
//	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
//*/
//
//	    // Re-enable the interrupt for the next byte
//		HAL_UART_Receive(&huart2, &rx_buffer[rx_index], 1, HAL_MAX_DELAY);
//		uint8_t received_data = rx_buffer[rx_index]; // Store the received byte
//
//		// Process received data for motor control
////		MotorControl(received_data); IMPORTNANTTTTTTTTTTTTTTTT
//
//		// Echo the received byte back via USART2 (optional for debugging)
//		HAL_UART_Transmit(&huart2, &received_data, 1, HAL_MAX_DELAY);
//
//
//
//	    /* Example motor control: Change PWM and direction */
///*	    Motor_Control(80);  // Positive value → Forward direction
//	    HAL_Delay(1000);
//	    Motor_Control(-80); // Negative value → Reverse direction
//	    HAL_Delay(1000);
//*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Function to print encoder data */
void Print_Encoder_Data(void)
{
  char buffer[50];
  sprintf(buffer, "Encoder Count: %lu, Diff: %d\n\r", encoder_count, encoder_diff);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
/* Function for motor contor */



static void Leaving_App_Handler(){
    //make bootManager enter bootloader

	Write_RTC_backup_reg(0x08020000, 0);
    Write_RTC_backup_reg(0x08030000, 0);

    //Write_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS, N_ENTER);
    //Write_RTC_backup_reg(BOOTLOADER_UPDATER_ENTER_FLAG_ADDRESS, ENTER);
    //sw reset
    NVIC_SystemReset();
}
static void Write_RTC_backup_reg(uint32_t reg ,uint32_t data){

HAL_PWR_EnableBkUpAccess();
HAL_RTCEx_BKUPWrite(&hrtc, reg, data);
HAL_PWR_DisableBkUpAccess();

}

void MotorControl(float deviation, int16_t encoder_position) {


	  static float prev_error = 0.0f; // Previous error for derivative calculation
	  float integral = 0;
	  static int16_t target_position = 0; // Target encoder position (default = 0)

	    float error = deviation; // Deviation as the current error
	    integral += error; // Accumulate integral
	    float derivative = error - prev_error; // Derivative calculation
	    float pid_output = KP * error + KD * derivative ;//+ KI * integral;

	    // Update the previous error
	    prev_error = error;

	    // Determine target encoder position based on deviation
	    if (deviation > 0.05f) {
	        target_position = (int16_t)((deviation / MAX_DEVIATION) * MAX_ENCODER_COUNT);
	    } else if (deviation < -0.05f) {
	        target_position = (int16_t)((deviation / MAX_DEVIATION) * MAX_ENCODER_COUNT); // Handle negative deviation
	    } else {
	        target_position = 0; // When deviation is zero or close to zero, return to primary position
	    }

	    // **Clamp the target position within ±500 PPR**
	    if (target_position > 500) target_position = 500;
	    if (target_position < -500) target_position = -500;
	    // Calculate the error to the target position
	    int16_t position_error = target_position - encoder_position;

	    // Compute motor speed based on position error
//	    uint32_t motor_speed = (uint32_t)((fabs(position_error) / MAX_ENCODER_COUNT) *(MAX_MOTOR_SPEED - MIN_MOTOR_SPEED) + MIN_MOTOR_SPEED);

	    uint32_t motor_speed = (uint32_t)((fabs(pid_output) / MAX_PID_OUTPUT) * (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED) + MIN_MOTOR_SPEED);

	    if (motor_speed > MAX_MOTOR_SPEED) motor_speed = MAX_MOTOR_SPEED;
	    if (motor_speed < MIN_MOTOR_SPEED) motor_speed = MIN_MOTOR_SPEED;

	    // Control motor direction and speed
	    if (position_error > 0) {
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // DIR = 1 (move right)
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
	    } else if (position_error < 0) {
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);     // DIR = 0 (move left)
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
	    } else {
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // Stop the motor when in position
	    }



	/*	 // Static variables for storing the previous error
	    static float prev_error = 0.0f;

	    // Compute PID output
	    float error = deviation; // Deviation as the current error
	    float derivative = error - prev_error;
	    float pid_output = KP * error + KD * derivative;

	    // Update the previous error
	    prev_error = error;

	    // Scale the motor speed and maximum rotation based on deviation
	    float abs_deviation = fabs(deviation);
	    uint32_t motor_speed = (uint32_t)((abs_deviation / MAX_DEVIATION) * (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED) + MIN_MOTOR_SPEED);
	    if (motor_speed > MAX_MOTOR_SPEED) motor_speed = MAX_MOTOR_SPEED;
	    if (motor_speed < MIN_MOTOR_SPEED) motor_speed = MIN_MOTOR_SPEED;

	    float max_rotations = (abs_deviation / MAX_DEVIATION) * (MAX_ROTATIONS - MIN_ROTATIONS) + MIN_ROTATIONS;
	    if (max_rotations > MAX_ROTATIONS) max_rotations = MAX_ROTATIONS;
	    if (max_rotations < MIN_ROTATIONS) max_rotations = MIN_ROTATIONS;

	    int32_t max_encoder_count = (int32_t)(ENCODER_PPR);

	    // Limit the encoder position to within the dynamically scaled max rotation range
	    if (encoder_position >= max_encoder_count && pid_output > 0) {
	        pid_output = 0; // Prevent further movement to the right
	    } else if (encoder_position <= -max_encoder_count && pid_output < 0) {
	        pid_output = 0; // Prevent further movement to the left
	    }

	    // Control motor direction and speed
	    if (pid_output > 0) {
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // DIR = 1 (right)
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
	    } else if (pid_output < 0) {
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // DIR = 0 (left)
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
	    } else {
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // Stop the motor
	    }
*/


	//    // Define constants
//    const uint32_t MAX_MOTOR_SPEED = 300; // Adjust as needed
//    const float KP = 100.0f; // Proportional gain for PID control
//    const float KD = 10.0f;  // Derivative gain for PID control
//
//    // Static variables for storing the previous error
//    static float prev_error = 0.0f;
//
//    // Compute PID output
//    float error = deviation; // Deviation as the current error
//    float derivative = error - prev_error;
//    float pid_output = KP * error + KD * derivative;
//
//    // Update the previous error
//    prev_error = error;
//
//    // Limit the encoder position to within the max rotation range
//    if (encoder_position >= MAX_ENCODER_COUNT && pid_output > 0) {
//        pid_output = 0; // Prevent further movement to the right
//    } else if (encoder_position <= -MAX_ENCODER_COUNT && pid_output < 0) {
//        pid_output = 0; // Prevent further movement to the left
//    }
//
//    // Determine motor speed and direction
//    uint32_t motor_speed = (uint32_t)fabs(pid_output);
//    if (motor_speed > MAX_MOTOR_SPEED) {
//        motor_speed = MAX_MOTOR_SPEED; // Limit the motor speed
//    }
//
//    if (pid_output > 0) {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // DIR = 1 (right)
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
//    } else if (pid_output < 0) {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // DIR = 0 (left)
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed); // Set PWM
//    } else {
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // Stop the motor
//    }
}





//    // PID calculations
//    float error = deviation;  // The deviation is the error
//    integral += error;        // Accumulate integral
//    float derivative = error - previous_error;  // Calculate derivative
//    previous_error = error;   // Update previous error
//
//    // PID output
//    float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
//
//    // Convert PID output to motor speed and direction
//    uint32_t motor_speed = (uint32_t)fabs(pid_output);
//    if (motor_speed > MOTOR_MAX_SPEED) {
//        motor_speed = MOTOR_MAX_SPEED;  // Clamp motor speed to maximum
//    }
//
//    if (pid_output > 0) {
//        // Deviation to the right, turn steering wheel left
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // DIR = 0
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed);  // Set PWM duty cycle
//    } else if (pid_output < 0) {
//        // Deviation to the left, turn steering wheel right
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);    // DIR = 1
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_speed);  // Set PWM duty cycle
//    } else {
//        // Centered, stop the motor
//        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);  // Set PWM to 0
//    }
//}


/* IMPORTANTTTTTTTTTTTTTTTTTTT
void MotorControl(uint8_t command) {
   // Define constants
    const uint32_t MOTOR_SPEED = 300; // Constant speed (adjust as needed)

    if (command == 0x1) {
        // Turn the motor to the right
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // DIR = 1
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_SPEED); // Set PWM duty cycle

    } else if (command == 0x2) {
        // Turn the motor to the left
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // DIR = 0
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_SPEED); // Set PWM duty cycle

    } else if (command == 0x0) {
        // Stop the motor
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // Set PWM to 0
    }
}
*/
/* Function to control motor PWM and direction */
//void Motor_Control(int16_t speed)
//{
/*
  if (speed >= 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // Forward direction
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // Reverse direction
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, -speed);
  }
*/
//}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (huart->Instance == USART2) { // Check if the interrupt is for USART2
        uint8_t received_data = rx_buffer[rx_index]; // Store the received byte

        // Process received data for motor control
        MotorControl(received_data);

        // Echo the received byte back via USART2 (optional for debugging)
        HAL_UART_Transmit(&huart2, &received_data, 1, HAL_MAX_DELAY);

        // Re-enable the interrupt for the next byte
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
    }
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
