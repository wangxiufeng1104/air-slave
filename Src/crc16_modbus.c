#include "crc16_modbus.h"

static void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{
    int32_t i;
    uint8_t tmp[4];
    tmp[0] = 0;
    for(i=0;i< 8;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(7-i);
    }
    dBuf[0] = tmp[0];
}
static void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf)
{
    int32_t i;
    uint16_t tmp[4];
    tmp[0] = 0;
    for(i=0;i< 16;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(15 - i);
    }
    dBuf[0] = tmp[0];
}
//static void InvertUint32(uint32_t *dBuf,uint32_t *srcBuf)
//{
//    int32_t i;
//    uint32_t tmp[4];
//    tmp[0] = 0;
//    for(i=0;i< 32;i++)
//    {
//      if(srcBuf[0]& (1 << i))
//        tmp[0]|=1<<(15 - i);
//    }
//    dBuf[0] = tmp[0];
//}
/**
  * @brief  CRC16_MODBUS校验算法
  * @param  puchMsg  要进行校验的数据
	*					usDataLen  要进行校验的数据的长度
  * @note   
  * @retval CRC16的校验结果
	* @author free
	* @time   2018/07/06
  */
uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wChar = 0;
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(int32_t i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}
