
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
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;
static uint8_t Device_ID;
extern DMA_HandleTypeDef hdma_usart1_tx;
#define SoftVer_MAIN 1
#define SoftVer_SUB  0
#define CDMP_SOH     0x7E

#define DEV_BOOT			  0x00	  // 空设备，特指Boot Loader
#define DEV_FAN_COIL		0x01	  // 风机盘管
#define DEV_FLOOR_HEAT	0x02	  // 地暖
#define DEV_FRESH			  0x03	  // 新风
#define DEV_HUMID			  0x04	  // 加湿机
#define DEV_DEHUMID		  0x05	  // 除湿机

#define DEVICE_TYPE  DEV_FAN_COIL
uint8_t P_frame[1024] = {0};
typedef struct
{
	uint8_t CDM_LOW;
	uint8_t CMD_MIN;
	uint8_t CMD_HIGH;
	uint8_t CMD_VALVE;
	uint8_t STATE_LOW;
	uint8_t STATE_MID;
	uint8_t STATE_HIGH;
	uint8_t STATE_VALVE;
	uint8_t LEN_LOW;
	uint8_t LEN_MIN;
	uint8_t LEN_HIGH;
	uint8_t LEN_VALVE;
}DEVICE_REGISTER;
DEVICE_REGISTER dev_reg;
typedef enum{
	READ_SINGLE = 0x81,
	READ_MUL ,
	WRITE_SINGLE,
	WRITE_MUL,
	UPDATE_DATE,
	INVENTORY
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
/**
  * @brief  风速类型
  */
typedef enum
{
	FAN_SPEED_LOW = 0U,    /*!< 低速*/
	FAN_SPEED_MID,         /*!< 中速*/
	FAN_SPEED_HIG,         /*!< 高速*/
	FAN_SPEED_OFF = 0xFF   /*!< 关闭*/
}FAN_SPEED;
extern INS_STRUCT ins_struct;      //指令buf
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
void Init_dev_reg(void);
uint8_t EXE_READ_SINGLE(uint8_t *p,uint32_t len);
void EXE_READ_MUL(uint8_t *p,uint32_t len);
void EXE_WRITE_SINGLE(uint8_t *p,uint32_t len);
void EXE_WRITE_MUL(uint8_t *p,uint32_t len);
void EXE_UPDATE_DATE(uint8_t *p,uint32_t len);
void RelaySwitch(Relay_num r_num,Relay_state r_state);
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
	//DMA 接收地址设置
	HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,0x400);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);   //开启空闲中断
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(UART_DIR_GPIO_Port,UART_DIR_Pin,GPIO_PIN_RESET);  //上电的时候设置接收状态
	UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
	//1、查看设备地址
	CheckDevideID(&Device_ID);    //每次上电的时候查询地址 
	//2、初始化指令缓存
	Init_Insbuf(&ins_struct);
	Init_dev_reg();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	/*1< 处理指令buf中未处理的指令，解析完一帧指令后设置处理指令flag*/
		
	/*2< 处理准备好的指令*/
//		if(UsartType1.receive_flag == 1)   //产生了空闲中断
//		{
//			UsartType1.receive_flag = 0; //清除标记
//			if(UsartType1.rx_len != 0)
//			{
//				//SendDataUSART1_DMA(UsartType1.usartDMA_rxBuf,UsartType1.rx_len);//串口打印收到的数据
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
/**
  * @brief 	移动指令队列中当前处理数据指针的位置
  * @param  step	指针向后移动的单位
  * @note   
  * @retval None
	* @author 王秀峰
	* @time   2018/07/09
  */
void right_shift_current(uint32_t step)
{
	uint32_t L_end;
	L_end = &ins_struct.ins_Buf[INS_MAX - 1] - ins_struct.insp_current;
	if(L_end >= step)
	{
		ins_struct.insp_current += step;
	}
	else
	{
		uint32_t L_start;
		L_start = step - L_end;
		ins_struct.insp_current = &ins_struct.ins_Buf[L_start - 1];
	}
	ins_struct.ins_length -= step;
}
/**
  * @brief 	从指令队列中提取数据解析
  * @param  None
  * @note   
  * @retval None
	* @author 王秀峰
	* @time   2018/07/09
  */
void Protocol_Resolution(void)
{
	if(ins_struct.ins_length >= 8)   //指令队列中存在未处理的指令
	{
		//1、寻找SOH
		if(CDMP_SOH == *ins_struct.insp_current )//非SOH || 地址错误
		{
			uint32_t Frame_Length;
			uint32_t L_end;
			L_end = &ins_struct.ins_Buf[INS_MAX - 1] - ins_struct.insp_current + 1;
			if(L_end > 2)
			{
				Frame_Length = ins_struct.insp_current[2];
			}
			else
			{
				Frame_Length = ins_struct.ins_Buf[2 - L_end];
			}
			if(Frame_Length > ins_struct.ins_length)
			{
				right_shift_current(1);
			}
			else    //将数据移动出来做进一步的检查
			{
				uint16_t crc16;
			
				if(Frame_Length <= L_end)
				{
					memcpy(P_frame,ins_struct.insp_current,Frame_Length);
				}
				else
				{
					memcpy(P_frame,ins_struct.insp_current,L_end);
					memcpy(P_frame + L_end,ins_struct.ins_Buf,Frame_Length - L_end);
				}
				crc16 = CRC16_MODBUS(P_frame,Frame_Length - 2);
				if((P_frame[3] == Device_ID) && (crc16 == ((P_frame[Frame_Length - 2] << 8)|(P_frame[Frame_Length - 1]))))
				{
					switch((CMD_TYPE)P_frame[5])
					{
						case 	READ_SINGLE:
							P_frame[2] = Frame_Length + 1;
							P_frame[7] = EXE_READ_SINGLE(&P_frame[6],Frame_Length - 8);
							crc16 = CRC16_MODBUS(P_frame,Frame_Length - 2 + 1);
							P_frame[9] = (uint8_t)crc16;
							P_frame[8] = (uint8_t)(crc16 >> 8);
							SendDataUSART1_DMA(P_frame, Frame_Length + 1);
							break;
						case READ_MUL:
							EXE_READ_MUL(&P_frame[6],Frame_Length - 8);
							break;
						case WRITE_SINGLE:
							EXE_WRITE_SINGLE(&P_frame[6],Frame_Length - 8);
							P_frame[2] = Frame_Length - 1;
							crc16 = CRC16_MODBUS(P_frame,Frame_Length - 2 - 1);
							P_frame[Frame_Length - 2] = (uint8_t)crc16;
							P_frame[Frame_Length - 2 - 1] = (uint8_t)(crc16 >> 8);
							SendDataUSART1_DMA(P_frame, Frame_Length - 1);
							break;
						case 	WRITE_MUL:
							EXE_WRITE_MUL(&P_frame[6],Frame_Length - 8);
							break;
						case UPDATE_DATE:
							EXE_UPDATE_DATE(&P_frame[6],Frame_Length - 8);
							break;
						case INVENTORY://盘点设备
							P_frame[2] = Frame_Length + 1;
							P_frame[6] = (uint8_t)DEVICE_TYPE;
							crc16 = CRC16_MODBUS(P_frame,Frame_Length - 2 + 1);
							P_frame[Frame_Length - 1 + 1] = (uint8_t)crc16;
							P_frame[Frame_Length - 2 + 1] = (uint8_t)(crc16 >> 8);
							SendDataUSART1_DMA(P_frame, Frame_Length + 1);
							break;
						default:
							_dbg_printf("invalid cmd\n");
							break;
					}
					right_shift_current(Frame_Length);
				}
				else
				{
					right_shift_current(1);
				}
			}
		}
		else //向后移动
		{
			right_shift_current(1);
		}	
	}
}
uint8_t EXE_READ_SINGLE(uint8_t *p,uint32_t len)
{
	switch(p[0])
	{
		case 0:
			return dev_reg.STATE_LOW;
		case 1:
			return dev_reg.STATE_MID;
		case 2:
			return dev_reg.STATE_HIGH;
		case 3:
			return dev_reg.STATE_VALVE;
		default:
			return 0xff;
	}
}
void EXE_READ_MUL(uint8_t *p,uint32_t len)
{
	
}
void EXE_WRITE_SINGLE(uint8_t *p,uint32_t len)
{
	switch(p[0])
	{
		case 0:
			if(p[1] == 0)
			{
				RelaySwitch(RELAY_LOW,RELAY_ON);
				dev_reg.STATE_LOW = 0;
			}
			else
			{
				RelaySwitch(RELAY_LOW,RELAY_OFF);
				dev_reg.STATE_LOW = 1;
			}
			break;
		case 1:
			if(p[1] == 0)
			{
				dev_reg.STATE_MID = 0;
				RelaySwitch(RELAY_MID,RELAY_ON);
			}
			else
			{
				RelaySwitch(RELAY_MID,RELAY_OFF);
				dev_reg.STATE_MID = 1;
			}
			break;
		case 2:
			if(p[1] == 0)
			{
				RelaySwitch(RELAY_HIGH,RELAY_ON);
				dev_reg.STATE_HIGH = 0;
			}
			else
			{
				RelaySwitch(RELAY_HIGH,RELAY_OFF);
				dev_reg.STATE_HIGH = 1;
			}
			break;
		case 3:
			if(p[1] == 0)
			{
				RelaySwitch(RELAY_LOW,RELAY_ON);
				dev_reg.STATE_VALVE = 0;
			}
			else
			{
				RelaySwitch(RELAY_LOW,RELAY_OFF);
				dev_reg.STATE_VALVE = 0;
			}
				
			break;
		default:
			break;
	}
}
void EXE_WRITE_MUL(uint8_t *p,uint32_t len)
{
	
}
void EXE_UPDATE_DATE(uint8_t *p,uint32_t len)
{
	
}							
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
/**
  * @brief  执行动作接口
  * @param    FAN_SPEED FAN_SPEED_LOW
	*					  			    FAN_SPEED_MID
	*					  			    FAN_SPEED_HIG
	*											FAN_SPEED_OFF
  * @note   根据输入的风速要求控制具体的继电器开关
	*           
  * @retval None
	* @author 王秀峰
	* @time   2018/07/05
  */
void Exe_action(FAN_SPEED f_speed)
{
	RelaySwitch(RELAY_ALL ,RELAY_ON);   //继电器打开
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
  * @brief  初始化设备寄存器
  * @param  None
  * @note   
	*           
  * @retval None
	* @author 王秀峰
	* @time   2018/07/11
  */
void Init_dev_reg(void)
{
	dev_reg.CDM_LOW 		= 0;
	dev_reg.CMD_MIN 		= 0;
	dev_reg.CMD_HIGH 		= 0;
	dev_reg.CMD_VALVE 	= 0;
	dev_reg.STATE_LOW 	= 0;
	dev_reg.STATE_MID 	= 0;
	dev_reg.STATE_HIGH 	= 0;
	dev_reg.STATE_VALVE = 0;
	dev_reg.LEN_LOW 		= 1;
	dev_reg.LEN_MIN 		= 1;
	dev_reg.LEN_HIGH 		= 1;
	dev_reg.LEN_VALVE 	= 1;
	
}
/**
  * @brief  初始化指令buf
  * @param  ins 要初始化的指令buf
  * @note   
	*           
  * @retval None
	* @author 王秀峰
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
