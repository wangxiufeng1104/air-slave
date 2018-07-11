/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdarg.h>

INS_STRUCT ins_struct;      //ָ��buf
USART_RECEIVETYPE UsartType1;
#define PRINTF_BUF_SIZE  0x200
static uint8_t print_buffer[PRINTF_BUF_SIZE];//��ӡ����
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  ��ʽ��ӡ
  * @param  format  Printf��ʽ
  * @note   
  * @retval None
	* @author �����
	* @time   2018/07/06
  */
void _dbg_printf(const char *format,...)
{
	uint32_t length;
	va_list args;
	while(UsartType1.dmaSend_flag == USART_DMA_SENDING) HAL_Delay(1);
	va_start(args, format);
	length = vsnprintf((char*)print_buffer, sizeof(print_buffer), (char*)format, args);
	va_end(args);
	SendDataUSART1_DMA((uint8_t *)print_buffer,length);
}
/**
  * @brief  ��ӡ16��������
  * @param  hex         ����
	*					hex_length	���鳤��
  * @note   
  * @retval None
	* @author �����
	* @time   2018/07/06
  */
void Printf_Hex(const uint8_t* hex, uint16_t hex_length)
{
	const uint8_t char_table[] = "0123456789ABCDEF";
	uint16_t j=0;
	while(UsartType1.dmaSend_flag == USART_DMA_SENDING) HAL_Delay(1);
	for(uint16_t i=0;(i<hex_length)&&j<sizeof(print_buffer);i++)
	{
		print_buffer[j++] = char_table[(hex[i]&0xF0)>>4];
		print_buffer[j++] = char_table[hex[i]&0x0F];
		print_buffer[j++] = ' ';
	}
	print_buffer[j++] = '\n';
	SendDataUSART1_DMA(print_buffer,j);//����
}
/**
  * @brief  ���������ݿ�����ָ���
  * @param  pSrc  �������ݻ���ĵ�ַ
	*					pDes  ָ���ĵ�ַ
	*					Len   Ҫ�����ĳ���
  * @note   
  * @retval None
	* @author �����
	* @time   2018/07/04
  */
void InsCopy(uint8_t *pSrc,uint8_t *pDes,uint8_t Len)
{
	//1���жϸ��Ƶķ�ʽ   a �����ĸ���   b ��ָ����β�����θ���   c ���������ָ��
	uint32_t Last_Size;//ָ�����ʣ��Ŀռ�
	uint32_t L_end,L_start;
	Last_Size = ((uint32_t)&ins_struct.ins_Buf[INS_MAX - 1] - (uint32_t)ins_struct.insp_end) + 1;
	if(Last_Size >= Len)  //a
	{
		memcpy(pDes,pSrc,Len);
		ins_struct.insp_end = ins_struct.insp_end + Len;//���µ�ַ
		ins_struct.ins_length += Len;
	}
	else
	{
		L_end = Last_Size;         //β��ʣ��ռ�
		L_start = Len - Last_Size; //ͷ��ʣ��ռ�
		//�ж��Ƿ����
		if(L_start < ((uint32_t)ins_struct.insp_current - (uint32_t)ins_struct.ins_Buf))  //b
		{
			memcpy(pDes,pSrc,L_end);
			memcpy(ins_struct.ins_Buf,pSrc+L_end,L_start);
			ins_struct.insp_end = &ins_struct.ins_Buf[L_start];//���µ�ַ
			ins_struct.ins_length += Len;
		}
		//��������κ����飬�����Զ�������      c
	}
}
void UsartReceive_IDLE(UART_HandleTypeDef *huart)
{
	uint32_t temp;
 
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		temp = huart1.hdmarx->Instance->CNDTR;
		UsartType1.rx_len =  RECEIVELEN - temp; 
		if(UsartType1.rx_len == 0x400) UsartType1.rx_len = 0;
		UsartType1.receive_flag=1;   //�յ�����
		HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,RECEIVELEN);	//��DMA�յ����ݷŵ�UsartType1.usartDMA_rxBuf��
		//�������ݵ�ָ��buf
		InsCopy(UsartType1.usartDMA_rxBuf,ins_struct.insp_end,UsartType1.rx_len);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		__HAL_DMA_DISABLE(huart->hdmatx);
		UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
		HAL_GPIO_WritePin(UART_DIR_GPIO_Port,UART_DIR_Pin,GPIO_PIN_RESET);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_GPIO_TogglePin(LED_H_GPIO_Port,LED_H_Pin);
	}
}
void SendDataUSART1_DMA(uint8_t *pData, uint16_t Size)
{
	while(UsartType1.dmaSend_flag == USART_DMA_SENDING) HAL_Delay(1);
	UsartType1.dmaSend_flag = USART_DMA_SENDING;
	UART_DIR_GPIO_Port->BSRR = UART_DIR_Pin;
	HAL_UART_Transmit_DMA(&huart1, pData, Size);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
