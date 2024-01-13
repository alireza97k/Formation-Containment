/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
//#include "sd_hal_mpu6050.h"
#include "String.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */



int Speed_Motor_Left;
int Speed_Motor_Right;
float Distance1;


uint8_t Counter_10ms;

// Motor Left Parameter
int Pulse_Left;
int Pulse_Left_pre;
uint8_t Revolve_Left;
int Speed_Left;
int  Setpoint_Left =0;
int error_left;
int Sum_error_left;
int error_left_pre;
int D_error_left;
int Kp_Left = 250 ;
int Ki_Left = 5 ;
int Kd_Left = 1 ;
int Output_Contol_Left;
int Max_sum_Error = 18000;
int Min_sum_Error = 9000;
int Max_D_Error = 2000;


// Motor Right Parameter
int Pulse_Right;
int Pulse_Right_pre;
uint8_t Revolve_Right;
int Speed_Right;
int  Setpoint_Right = -0;
int error_Right;
int Sum_error_Right;
int error_Right_pre;
int D_error_Right;
int Kp_Right = 250 ;
int Ki_Right = 5 ;
int Kd_Right = 1 ;
int Output_Contol_Right;
int Max_sum_Error_Right = 18000;
int Min_sum_Error_Right = 9000;
int Max_D_Error_Right = 2000;
 
int State;
int Control_Signal_Deg;
int Control_Signal_Dis;


uint32_t X_Roobot;
double Y_Roobot;
double Deg_Roobot;
uint8_t Test[6]; 
uint8_t Ref ; 
uint8_t Start_Stop;
int Speed_Robot;

char Test2[5];
uint8_t Data_R[22];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_FREEZE_TIM1_DBGMCU();
  /*initial encoder Motor Right*/

	__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_2);	
  
	/*initial encoder Motor Left*/
	__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_2);
	
	
	  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		TIM2->CCR1 = Speed_Motor_Right;
		HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction11_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction12_Pin,GPIO_PIN_SET);
		TIM2->CCR2 = Speed_Motor_Left;
		HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction21_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction22_Pin,GPIO_PIN_SET);
	
	   
	  HAL_UART_Receive_DMA(&huart1,Data_R,18);
	  
	 
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		  if( Start_Stop == 0x30){
//				if( Ref == 'w'){
//  				Setpoint_Left = 250 ;
//					Setpoint_Right = 250 ;
//				}
//				else if( Ref == 'a'){
//					Setpoint_Left = 100 ;
//					Setpoint_Right = 250 ;
//				}
//				else if( Ref == 'd'){
//					Setpoint_Left = 250 ;
//					Setpoint_Right = 100 ;
//				}
//			}
//			else{
//				Setpoint_Left = 0 ;
//				Setpoint_Right = 0 ;
//			}
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			HAL_Delay(100);
//		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
		  HAL_Delay(1);
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
//		  HAL_Delay(1000);
//		  Setpoint_Right = Setpoint_Left;
//      TIM2->CCR2 = Speed_Motor_Right;
		  //TIM2->CCR1 = Speed_Motor_Left;

		
  }
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Ref = Data_R[2];
	Start_Stop = Data_R[4];
	for(int i =0 ; i<6;i++){
	  Test[i] = Data_R[i];
	}
	//X_Roobot = atoi(Test);
	Control_Signal_Deg = atoi(Test);
	
//	float Zavie =  Control_Signal_Deg/100.0;
	for(int i =6 ; i<12;i++){
	  Test[i-6] = Data_R[i];
	}
	Control_Signal_Dis = atoi(Test);
	for(int i =12 ; i<18;i++){
	  Test[i-12] = Data_R[i];
	}
	Speed_Robot = atoi(Test);
	
	
	if( Control_Signal_Dis >600){
		Control_Signal_Dis = 600;
	}
//if( Control_Signal_Dis <20){
//	Control_Signal_Dis = 0;
//}
//	
//	if( Control_Signal_Deg < 200 && Control_Signal_Deg > -200){
//		Setpoint_Right = Control_Signal_Dis;
//		Setpoint_Left = Control_Signal_Dis ;
//	}	
// 	else{
//		if( Control_Signal_Deg > 0 ){
//		  Setpoint_Right = Control_Signal_Dis - Zavie;
//		  Setpoint_Left = Control_Signal_Dis ;
//			if(Setpoint_Right < 100){
//				Setpoint_Right = 100;
//			}
//	  }
//	  else{
//	  	Setpoint_Right = Control_Signal_Dis ;
//		  Setpoint_Left = Control_Signal_Dis + Zavie;
//				if(Setpoint_Left < 100){
//				Setpoint_Left	= 100;
//			}
//	  }
//	}
	
	
//if( Distance1 > 20){
   if( Control_Signal_Dis >25){
		 Setpoint_Right = Control_Signal_Deg;
		 Setpoint_Left = Speed_Robot ;
	}
	else{
		 Setpoint_Right = 0;
		 Setpoint_Left = 0 ;
	}
//}
//else{
//	    Setpoint_Right = 0;
//		  Setpoint_Left = 0 ;
//}
//	
//	for(int i =5 ; i<10;i++){
//	  Test[i-5] = Data_R[i];
//	}
//	Y_Roobot = atoi(Test);
//	for(int i =10 ; i<15;i++){
//	  Test[i-10] = Data_R[i];
//	}
//	Deg_Roobot = atoi(Test);
	
  //X_Roobot = Data_R[0]+Data_R[1]*256 + Data_R[2]*65536; 
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_SET){
		   HAL_TIM_Base_Start(&htim4);
		}
		else{
			HAL_TIM_Base_Stop(&htim4);
			Distance1 = TIM4->CNT * 0.17;
			TIM4->CNT = 0 ; 
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	   
if(htim->Instance == TIM2 ){
			  Counter_10ms+=1;
        if(Counter_10ms > 10 ){
					
					// Calculate Speed Left
					
 					Counter_10ms = 0;
          Pulse_Left = __HAL_TIM_GET_COUNTER(&htim1);		
          if(Setpoint_Left >= 0){
						Speed_Left = 65535*Revolve_Left + Pulse_Left - Pulse_Left_pre;
					}
					else{
						Speed_Left = -65535*Revolve_Left + Pulse_Left - Pulse_Left_pre; 
					}
					Pulse_Left_pre = Pulse_Left;
          Revolve_Left = 0;	
					
					// Calculate Speed Right
					
 					
          Pulse_Right = __HAL_TIM_GET_COUNTER(&htim3);
          if(Setpoint_Right >= 0){					
					  Speed_Right = 65535*Revolve_Right + Pulse_Right - Pulse_Right_pre;
					}
					else{
						Speed_Right = -65535*Revolve_Right + Pulse_Right - Pulse_Right_pre;
					}
					Pulse_Right_pre = Pulse_Right;
          Revolve_Right = 0;	
          //Speed_Right = Speed_Right*(-1);
         // Calculate Controller Output
          
					
					
					
					if( Setpoint_Left >=0){
						HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction11_Pin,GPIO_PIN_RESET);
		        HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction12_Pin,GPIO_PIN_SET);
						error_left = Setpoint_Left - Speed_Left;
					}
					else{
						HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction11_Pin,GPIO_PIN_SET);
		        HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction12_Pin,GPIO_PIN_RESET);
						error_left = -Setpoint_Left + Speed_Left;
					}
          //error_left = -Setpoint_Left + Speed_Left;
          Sum_error_left += Ki_Left*error_left;					
          D_error_left = Kd_Left*(error_left-error_left_pre);
					
					if( Sum_error_left > Max_sum_Error){
					   Sum_error_left = Max_sum_Error;
					}
					if( Sum_error_left < Min_sum_Error){
					   Sum_error_left = Min_sum_Error;
					}
					if( D_error_left > Max_D_Error){
					   D_error_left = Max_D_Error;
					}
					if( D_error_left < -Max_D_Error){
					   D_error_left = -Max_D_Error;
					}
            Output_Contol_Left = Kp_Left*error_left + D_error_left + Sum_error_left;
					if( Output_Contol_Left > 36000){
					   Output_Contol_Left = 36000;
					}
					if( Output_Contol_Left < 0){
					   Output_Contol_Left = 0;
					}
					if(Setpoint_Left == 0 ){
						Output_Contol_Left = 0;
					}
					TIM2->CCR1 = abs(Output_Contol_Left);
					
					
					if( Setpoint_Right >=0){
						HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction21_Pin,GPIO_PIN_RESET);
	        	HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction22_Pin,GPIO_PIN_SET);
						error_Right = Setpoint_Right - Speed_Right;
					}
					else{
						HAL_GPIO_WritePin(Direction11_GPIO_Port,Direction21_Pin,GPIO_PIN_SET);
		        HAL_GPIO_WritePin(Direction12_GPIO_Port,Direction22_Pin,GPIO_PIN_RESET);
		 				error_Right = -Setpoint_Right + Speed_Right;
					}
					
          Sum_error_Right += Ki_Right*error_Right;					
          D_error_Right = Kd_Right*(error_Right-error_Right_pre);
					
					if( Sum_error_Right > Max_sum_Error_Right){
					   Sum_error_Right = Max_sum_Error_Right;
					}
					if( Sum_error_Right < Min_sum_Error_Right){
					   Sum_error_Right = Min_sum_Error_Right;
					}
					if( D_error_Right > Max_D_Error_Right){
					   D_error_Right = Max_D_Error_Right;
					}
					if( D_error_Right < -Max_D_Error_Right){
					   D_error_Right = -Max_D_Error_Right;
					}
            Output_Contol_Right = Kp_Right*error_Right + D_error_Right + Sum_error_Right;
					if( Output_Contol_Right > 36000){
					   Output_Contol_Right = 36000;
					}
					if( Output_Contol_Right < 0){
					   Output_Contol_Right = 0;
					}
					if(Setpoint_Right == 0 ){
						Output_Contol_Right = 0;
					}
					TIM2->CCR2 = Output_Contol_Right;
							
				}		
						
				
		 }   

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{    
     
		 if(htim->Instance == TIM1){
			 Revolve_Left = 1;
		 }
		 else if(htim->Instance == TIM3){
			 Revolve_Right = 1;
		 }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
