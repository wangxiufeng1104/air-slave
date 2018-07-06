
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
#include "crc16_modbus.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;
static uint8_t Devide_ID;
#define SoftVer_MAIN 1
#define SoftVer_SUB  0
#define CDMP_SOH 0x7E
#define ADDRESS 0x55
typedef enum{
	READ_ALL = 0U,/*!< ���������ݣ�Ҫ���豸�ϴ���������*/
	READ_Date,    /*!< ��ָ�����ݣ�Ҫ���豸�ϴ�ָ����������*/
	WRITE_DATE,   /*!< дָ�����ݣ�Ҫ���豸����ָ�����ݲ���*/
	RESPOND_DATE, /*!< ��Ӧ���ݣ����ڶ����ģ������������ݣ�����д���������ģ����ز���״̬*/
	UPDATE_DATE,  /*!< �������󣬶���ͬһ��������Packet IDӦ��ͬ*/
	UPDATE_RES    /*!< ������Ӧ����ӦPacket ID���������ֵӦ��ͬ*/
}CMD_TYPE;
typedef struct
{
	uint8_t   SOH;
	uint8_t   Version;
	uint8_t   Length;
	uint16_t  Address;
	uint8_t   PacketID;
	CMD_TYPE  CMD;
	uint8_t   *PUD;
	uint16_t  CRC16;
}CDMP;
/**
  * @brief  �̵������
  */
typedef enum   
{
	RELAY_HIGH = 0U,
	RELAY_MID,
	RELAY_LOW,
	RELAY_ALL
} Relay_num;
/**
  * @brief  �̵�������״̬
  */
typedef enum
{
	RELAY_ON = 0U,
	RELAY_OFF
} Relay_state;
/**
  * @brief  ��ѹ���
  */
typedef enum
{
	VOLATAGE_3 = 0U,
	VOLATAGE_5,
	VOLATAGE_8
} VOLATAGE_OUT;
/**
  * @brief  ��������
  */
typedef enum
{
	FAN_SPEED_LOW = 0U,    /*!< ����*/
	FAN_SPEED_MID,         /*!< ����*/
	FAN_SPEED_HIG,         /*!< ����*/
	FAN_SPEED_OFF = 0xFF   /*!< �ر�*/
}FAN_SPEED;
extern INS_STRUCT ins_struct;      //ָ��buf
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void CheckDevideID(uint8_t *pDevideID);
void Exe_action(FAN_SPEED f_speed);
void Init_Insbuf(INS_STRUCT *ins);
void Protocol_Resolution(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


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
	//DMA ���յ�ַ����
	HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,0x400);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);   //���������ж�
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(UART_DIR_GPIO_Port,UART_DIR_Pin,GPIO_PIN_RESET);  //�ϵ��ʱ�����ý���״̬
	UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
	//1���鿴�豸��ַ
	CheckDevideID(&Devide_ID);    //ÿ���ϵ��ʱ���ѯ��ַ 
	//2����ʼ��ָ���
	Init_Insbuf(&ins_struct);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	/*1< ����ָ��buf��δ�����ָ�������һָ֡������ô���ָ��flag*/
		
	/*2< ����׼���õ�ָ��*/
//		if(UsartType1.receive_flag == 1)   //�����˿����ж�
//		{
//			UsartType1.receive_flag = 0; //������
//			if(UsartType1.rx_len != 0)
//			{
//				//SendDataUSART1_DMA(UsartType1.usartDMA_rxBuf,UsartType1.rx_len);//���ڴ�ӡ�յ�������
//				_dbg_printf("%s",UsartType1.usartDMA_rxBuf);
//			}	
//		}
		Protocol_Resolution();
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
void Protocol_Resolution(void)
{
	if(ins_struct.ins_length > 0)   //ָ������д���δ�����ָ��
	{
		//1��Ѱ��SOH
		if(CDMP_SOH == *ins_struct.insp_current && ADDRESS == ins_struct.ins_Buf[3])//��SOH || ��ַ����
		{
			//2���ж��ڻ��ζ����е�λ��
			uint8_t Length = *(ins_struct.insp_current + 2);
			_dbg_printf("Length = %d\n",Length);
			if(Length <= 8 || Length > ins_struct.ins_length)
			{
				_dbg_printf("Length ERROR\n");
				ins_struct.insp_current ++;
				ins_struct.ins_length --;
				_dbg_printf("ins_struct.ins_length = %d\n",ins_struct.ins_length);
				return ;
			}
			uint16_t crc16 = CRC16_MODBUS(ins_struct.insp_current,Length - 2);
			_dbg_printf("crc16 = %x\n",crc16);
			if(crc16 == ((ins_struct.insp_current[Length - 2] << 8)|(ins_struct.insp_current[Length - 1])))
			{
				switch((CMD_TYPE)ins_struct.insp_current[5])
				{
					case READ_ALL:
						_dbg_printf("read all data\n");
						break;
					case READ_Date:
						_dbg_printf("read some data\n");
						break;
					case WRITE_DATE:
						_dbg_printf("write date\n");
						break;
					case RESPOND_DATE:
						_dbg_printf("respond data\n");
						break;
					case UPDATE_DATE:
						_dbg_printf("update date\n");
						break;
					case UPDATE_RES:
						_dbg_printf("update respoond\n");
						break;
					default:
						_dbg_printf("invalid cmd\n");
						break;
				}
				_dbg_printf("\nData:");
				for(int i = 0;i < Length - 8;i ++)
				{
					_dbg_printf("%x ",ins_struct.insp_current[i + 6]);
				}
				_dbg_printf("\n");
				ins_struct.insp_current += Length;
				ins_struct.ins_length -= Length;
				_dbg_printf("ins_struct.ins_length = %d\n",ins_struct.ins_length);
			}
			else
			{
				_dbg_printf("data error\n");
				ins_struct.insp_current ++;
				ins_struct.ins_length --;
				_dbg_printf("ins_struct.ins_length = %d\n",ins_struct.ins_length);
			}
		}
		else //����ƶ�
		{
			_dbg_printf("not SOH || address error\n");
			ins_struct.insp_current ++;
			ins_struct.ins_length --;
			_dbg_printf("ins_struct.ins_length = %d\n",ins_struct.ins_length);
		}	
	}
}

/**
  * @brief  ��ѯ�豸�ĵ�ַ������ĵ�ַд�����
  * @param  pDevideID   �豸��ַ
  * @note   
  * @retval None
	* @author �����
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
  * @brief  �Լ̵�����Ŷ���
  */
#define IS_RELAY_NUM(R_NUM) (((R_NUM) == RELAY_HIGH) || \
														((R_NUM) == RELAY_MID) || \
                            ((R_NUM) == RELAY_LOW) ||\
														((R_NUM) == RELAY_ALL))
/**
  * @brief  �Լ̵���״̬����
  */
#define IS_RELAY_STATE(R_STATE) (((R_STATE) == RELAY_ON) || \
																((R_STATE) == RELAY_OFF))
/**
  * @brief  ��������ÿ���̵�����״̬����
  * @param  Relay_num���̵������  
	*											RELAY_HIGH
	*											RELAY_MID
	*											RELAY_LOW
	*					Relay_state���̵���״̬ 
	*											RELAY_OFF  �̵����պ� 
	*											RELAY_ON   �̵�����
  * @note   
  * @retval None
	* @author �����
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
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin,GPIO_PIN_SET);
			}
				
			else
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_MID:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_M_GPIO_Port,LED_M_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_MID_GPIO_Port,R_MID_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_M_GPIO_Port,LED_M_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_MID_GPIO_Port,R_MID_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_LOW:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_L_GPIO_Port,LED_L_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_LOW_GPIO_Port,R_LOW_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_L_GPIO_Port,LED_L_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_LOW_GPIO_Port,R_LOW_Pin,GPIO_PIN_RESET);
			}
			break;
		case RELAY_ALL:
			if(r_state == RELAY_OFF)
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin|LED_M_Pin|LED_L_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin|R_MID_Pin|R_LOW_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_H_GPIO_Port,LED_H_Pin|LED_M_Pin|LED_L_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_HIGH_GPIO_Port,R_HIGH_Pin|R_MID_Pin|R_LOW_Pin,GPIO_PIN_RESET);
			}
			break;
		default:
			break;
	}
}
/**
  * @brief  ���3-5-8V��ѹ
  * @param    v_out     VOLATAGE_3
	*					  			    VOLATAGE_5
	*					  			    VOLATAGE_8
  * @note   �Ƚϲ���Ĵ�������ֵ���ѹ��Ӧ��ϵ
	*           htim2.Instance->CCR1 = 306;     3V
	*						htim2.Instance->CCR1 = 510;     5V
	*					  htim2.Instance->CCR1 = 817;     8V
  * @retval None
	* @author �����
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
/**
  * @brief  ִ�ж����ӿ�
  * @param    FAN_SPEED FAN_SPEED_LOW
	*					  			    FAN_SPEED_MID
	*					  			    FAN_SPEED_HIG
	*											FAN_SPEED_OFF
  * @note   ��������ķ���Ҫ����ƾ���ļ̵�������
	*           
  * @retval None
	* @author �����
	* @time   2018/07/05
  */
void Exe_action(FAN_SPEED f_speed)
{
	RelaySwitch(RELAY_ALL ,RELAY_ON);   //�̵�����
	switch(f_speed)
	{
		case FAN_SPEED_LOW:
			RelaySwitch(RELAY_LOW ,RELAY_OFF);
			VoltageOutput(VOLATAGE_3);
			break;
		case FAN_SPEED_MID:
			RelaySwitch(RELAY_MID ,RELAY_OFF);
			VoltageOutput(VOLATAGE_5);
			break;
		case FAN_SPEED_HIG:
			RelaySwitch(RELAY_HIGH ,RELAY_OFF);
			VoltageOutput(VOLATAGE_8);
			break;
		case FAN_SPEED_OFF:
		default:
			break;
	}
}
/**
  * @brief  ��ʼ��ָ��buf
  * @param  ins Ҫ��ʼ����ָ��buf
  * @note   
	*           
  * @retval None
	* @author �����
	* @time   2018/07/05
  */
void Init_Insbuf(INS_STRUCT *ins)
{
	ins->insp_current = ins->ins_Buf;
	ins->insp_end = ins->ins_Buf;
	ins->ins_length = 0;
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
