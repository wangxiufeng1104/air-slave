/**
  ******************************************************************************
  * File Name          : crc16_modbus.h
  * Description        : This file contains all the function prototypes for
  *                      the dma.c file
  ******************************************************************************/
#ifndef __crc16_modbus_H
#define __crc16_modbus_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen);
#ifdef __cplusplus
}
#endif

#endif /* __dma_H */

/**
  * @}
  */

