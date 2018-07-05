
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;
static uint8_t Devide_ID;
#define SoftVer_MAIN 1
#define SoftVer_SUB  0
typedef struct
{
	uint8_t SOH;
	uint8_t Version;
	uint8_t Length;
	uint8_t Address;
	uint8_t PacketID;
	uint8_t CMD;
	uint8_t *PUD;
	uint16_t CRC16;
}CDMP;
/**
  * @brief  继电器序号
  */
typedef enum   
{
	RELAY_HIGH = 0U,
	RELAY_MID,
	RELAY_LOW,
	RELAY_ALL
} Relay_num;
/**
  * @brief  继电器开关状态
  */
typedef enum
{
	RELAY_ON = 0U,
	RELAY_OFF
} Relay_state;
/**
  * @brief  电压输出
  */
typedef enum
{
	VOLATAGE_3 = 0U,
	VOLATAGE_5,
	VOLATAGE_8
} VOLATAGE_OUT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void CheckDevideID(uint8_t *pDevideID);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t RX_BUF[50];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//DMA 接收地址设置
	HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,UsartType1.rx_len);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);   //开启空闲中断
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOB,LED_L_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_DIR_GPIO_Port,UART_DIR_Pin,GPIO_PIN_RESET);  //上电的时候设置接收状态
	UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
	//1、查看设备地址
	CheckDevideID(&Devide_ID);    //每次上电的时候查询地址 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if(UsartType1.receive_flag == 1)   //产生了空闲中断
	{
		UsartType1.receive_flag = 0; //清除标记
		if(UsartType1.rx_len != 0)
		{
			SendDataUSART1_DMA(UsartType1.usartDMA_rxBuf,UsartType1.rx_len);//串口打印收到的数据
		}
		
		sprintf((char *)RX_BUF,"\ndata length: %d\n",UsartType1.rx_len);
		SendDataUSART1_DMA(RX_BUF,20);	
	}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  查询设备的地址，向传入的地址写入参数
  * @param  pDevideID   设备地址
  * @note   
  * @retval None
	* @author 王秀峰
	* @time   2018/07/04
  */
void CheckDevideID(uint8_t *pDevideID)
{
	
	*pDevideID = HAL_GPIO_ReadPin(ADD1_GPIO_Port,ADD1_Pin)|\
			HAL_GPIO_ReadPin(ADD2_GPIO_Port,ADD2_Pin) << 1|\
			HAL_GPIO_ReadPin(ADD3_GPIO_Port,ADD3_Pin) << 2|\
			HAL_GPIO_ReadPin(ADD4_GPIO_Port,ADD4_Pin) << 3;
}
/**
  * @brief  对继电器序号断言
  */
#define IS_RELAY_NUM(R_NUM) (((R_NUM) == RELAY_HIGH) || \
														((R_NUM) == RELAY_MID) || \
                            ((R_NUM) == RELAY_LOW) ||\
														((R_NUM) == RELAY_ALL))
/**
  * @brief  对继电器状态断言
  */
#define IS_RELAY_STATE(R_STATE) (((R_STATE) == RELAY_ON) || \
																((R_STATE) == RELAY_OFF))
/**
  * @brief  单独控制每个继电器的状态开关
  * @param  Relay_num：继电器序号  
	*											RELAY_HIGH
	*											RELAY_MID
	*											RELAY_LOW
	*					Relay_state：继电器状态 
	*											RELAY_OFF  继电器闭合 
	*											RELAY_ON   继电器打开
  * @note   
  * @retval None
	* @author 王秀峰
	* @time   2018/07/04
  */
void RelaySwitch(Relay_num r_num,Relay_state r_state)
{
	//assert_param(IS_RELAY_NUM(r_num));
	//assert_param(IS_RELAY_STATE(r_state));
	switch(r_num)
	{
		case RELAY_HIGH:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin,GPIO_PIN_SET);
			}
				
			else
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_MID:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_M_GPIO_Port,LED_M_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_MID_GPIO_Port,R_MID_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_M_GPIO_Port,LED_M_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_MID_GPIO_Port,R_MID_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_LOW:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_L_GPIO_Port,LED_L_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_LOW_GPIO_Port,R_LOW_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_L_GPIO_Port,LED_L_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_LOW_GPIO_Port,R_LOW_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_ALL:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin|LED_M_Pin|LED_L_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin|R_MID_Pin|R_LOW_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin|LED_M_Pin|LED_L_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin|R_MID_Pin|R_LOW_Pin,GPIO_PIN_RESET);
			}
			break;
		default:
			break;
	}
}
/**
  * @brief  输出3-5-8V电压
  * @param    v_out     VOLATAGE_3
	*					  			    VOLATAGE_5
	*					  			    VOLATAGE_8
  * @note   比较捕获寄存器重载值与电压对应关系
	*           htim2.Instance->CCR1 = 306;     3V
	*						htim2.Instance->CCR1 = 510;     5V
	*					  htim2.Instance->CCR1 = 817;     8V
  * @retval None
	* @author 王秀峰
	* @time   2018/07/04
  */
void VoltageOutput(VOLATAGE_OUT v_out)
{
	switch(v_out)
	{
		case VOLATAGE_3:
			htim2.Instance->CCR1 = 306;
			break;
		case VOLATAGE_5:
			htim2.Instance->CCR1 = 510;
			break;
		case VOLATAGE_8:
			htim2.Instance->CCR1 = 817;
			break;
		default:
			htim2.Instance->CCR1 = 0;
			break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
