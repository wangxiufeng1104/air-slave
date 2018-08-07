#line 1 "../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c"



























































































































 

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"


































 

 







 
#line 1 "../Inc/stm32f1xx_hal_conf.h"
































  

 







#line 1 "../Inc/main.h"





































 

 



 

 

 

 

#line 74 "../Inc/main.h"

 



 
 

 

 




void _Error_Handler(char *, int);








 
#line 44 "../Inc/stm32f1xx_hal_conf.h"
 
 

 


 
  

 
 
 
 
 
 
 

 
 

 
 
 

 
 
 
 
 
 
 
 
 
 
 
 
 
 


 
 

#line 93 "../Inc/stm32f1xx_hal_conf.h"

 




 












 






 









 









 

 


      





 



 
 

 

 

 
#line 168 "../Inc/stm32f1xx_hal_conf.h"

    





 

  

  

 





 



 
#line 202 "../Inc/stm32f1xx_hal_conf.h"




  
 





 


 

#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

































 

 







 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"


































 

 







 
#line 1 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"













































 



 



 
    






  


 



 






 

#line 96 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"



 
  
#line 109 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"



 
#line 121 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"



 



 

#line 1 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"











































 




 



 
    









 


 







 



 




 

  
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      
  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  USBWakeUp_IRQn              = 42,      
} IRQn_Type;



 

#line 1 "../Drivers/CMSIS/Include/core_cm3.h"
 




 

























 











#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "../Drivers/CMSIS/Include/core_cm3.h"

















 




 



 

 













#line 120 "../Drivers/CMSIS/Include/core_cm3.h"



 







#line 162 "../Drivers/CMSIS/Include/core_cm3.h"

#line 1 "../Drivers/CMSIS/Include/core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 297 "../Drivers/CMSIS/Include/cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "../Drivers/CMSIS/Include/cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 

#line 731 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 


#line 54 "../Drivers/CMSIS/Include/core_cmInstr.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmInstr.h"

   

#line 164 "../Drivers/CMSIS/Include/core_cm3.h"
#line 1 "../Drivers/CMSIS/Include/core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "../Drivers/CMSIS/Include/core_cmFunc.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmFunc.h"

 

#line 165 "../Drivers/CMSIS/Include/core_cm3.h"
















 
#line 203 "../Drivers/CMSIS/Include/core_cm3.h"

 






 
#line 219 "../Drivers/CMSIS/Include/core_cm3.h"

 




 












 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:27;               
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 


















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 



























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 




#line 500 "../Drivers/CMSIS/Include/core_cm3.h"

 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    

  volatile uint32_t ACTLR;                   



} SCnSCB_Type;

 



 










 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   


#line 1223 "../Drivers/CMSIS/Include/core_cm3.h"







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1372 "../Drivers/CMSIS/Include/core_cm3.h"

#line 1381 "../Drivers/CMSIS/Include/core_cm3.h"






 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 151 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
#line 1 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/system_stm32f1xx.h"



































 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           
extern const uint8_t  AHBPrescTable[16U];   
extern const uint8_t  APBPrescTable[8U];    



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 152 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
#line 153 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t SR;                
  volatile uint32_t CR1;               
  volatile uint32_t CR2;               
  uint32_t  RESERVED[16];
  volatile uint32_t DR;                
} ADC_Common_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint32_t DR1;
  volatile uint32_t DR2;
  volatile uint32_t DR3;
  volatile uint32_t DR4;
  volatile uint32_t DR5;
  volatile uint32_t DR6;
  volatile uint32_t DR7;
  volatile uint32_t DR8;
  volatile uint32_t DR9;
  volatile uint32_t DR10;
  volatile uint32_t RTCCR;
  volatile uint32_t CR;
  volatile uint32_t CSR;
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;            
  volatile uint8_t  IDR;           
  uint8_t       RESERVED0;     
  uint16_t      RESERVED1;       
  volatile uint32_t CR;             
} CRC_TypeDef;




 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;





 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;            
  volatile uint32_t PR;            
  volatile uint32_t RLR;           
  volatile uint32_t SR;            
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;


} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t CRH;
  volatile uint32_t CRL;
  volatile uint32_t PRLH;
  volatile uint32_t PRLL;
  volatile uint32_t DIVH;
  volatile uint32_t DIVL;
  volatile uint32_t CNTH;
  volatile uint32_t CNTL;
  volatile uint32_t ALRH;
  volatile uint32_t ALRL;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
} SPI_TypeDef;



 
typedef struct
{
  volatile uint32_t CR1;              
  volatile uint32_t CR2;              
  volatile uint32_t SMCR;             
  volatile uint32_t DIER;             
  volatile uint32_t SR;               
  volatile uint32_t EGR;              
  volatile uint32_t CCMR1;            
  volatile uint32_t CCMR2;            
  volatile uint32_t CCER;             
  volatile uint32_t CNT;              
  volatile uint32_t PSC;              
  volatile uint32_t ARR;              
  volatile uint32_t RCR;              
  volatile uint32_t CCR1;             
  volatile uint32_t CCR2;             
  volatile uint32_t CCR3;             
  volatile uint32_t CCR4;             
  volatile uint32_t BDTR;             
  volatile uint32_t DCR;              
  volatile uint32_t DMAR;             
  volatile uint32_t OR;               
}TIM_TypeDef;




 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 
  
typedef struct
{
  volatile uint16_t EP0R;                   
  volatile uint16_t RESERVED0;                  
  volatile uint16_t EP1R;                  
  volatile uint16_t RESERVED1;                    
  volatile uint16_t EP2R;                  
  volatile uint16_t RESERVED2;                    
  volatile uint16_t EP3R;                   
  volatile uint16_t RESERVED3;                    
  volatile uint16_t EP4R;                  
  volatile uint16_t RESERVED4;                    
  volatile uint16_t EP5R;                  
  volatile uint16_t RESERVED5;                    
  volatile uint16_t EP6R;                  
  volatile uint16_t RESERVED6;                    
  volatile uint16_t EP7R;                  
  volatile uint16_t RESERVED7[17];              
  volatile uint16_t CNTR;                  
  volatile uint16_t RESERVED8;                    
  volatile uint16_t ISTR;                  
  volatile uint16_t RESERVED9;                    
  volatile uint16_t FNR;                   
  volatile uint16_t RESERVEDA;                    
  volatile uint16_t DADDR;                 
  volatile uint16_t RESERVEDB;                    
  volatile uint16_t BTABLE;                
  volatile uint16_t RESERVEDC;                    
} USB_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  


 











 




#line 659 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



#line 672 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"










 






 
  


   

#line 737 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"




 



 
  
  

 
    
 
 
 

 
 
 
 
 

 




 




 




 
 
 
 
 

 
#line 798 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 805 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 815 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 825 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"






 
#line 844 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 

 




 




 




 




 




 




 




 




 




 






 
#line 916 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 924 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 941 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 

 
#line 979 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
 










 










 
#line 1012 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 1022 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1030 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 
#line 1044 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 



















 
#line 1079 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"




#line 1129 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1137 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







  
#line 1154 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1207 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
#line 1228 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 1242 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"









 
#line 1267 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 1278 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 1291 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"










 
#line 1314 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"




 
#line 1337 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 1351 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"









 
#line 1376 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 1387 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 1400 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"










 
#line 1420 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 





#line 1439 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 1468 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
 
 
 
 
 

 








































































































 








































































































 
#line 1736 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1786 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1836 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 1885 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1935 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 1988 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 

 
#line 1999 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2047 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 2054 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2069 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 
#line 2087 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 
#line 2102 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 
#line 2117 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 
#line 2135 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 
#line 2150 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"











 
#line 2169 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 
#line 2181 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 2192 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
#line 2207 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2228 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2249 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 2270 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2291 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2305 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2326 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2347 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 2368 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2389 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2403 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2424 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2445 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 2466 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2487 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2501 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2522 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2543 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 2564 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2585 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 



 
 
 
 
 

 
#line 2654 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2676 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
 
 
#line 2735 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2756 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2815 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2836 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2895 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2916 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2975 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 2996 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3055 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3076 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 

 
#line 3168 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3254 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3280 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"























 




 




 




 
 
 
 
 



 


 
#line 3345 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 



 
#line 3359 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3384 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3391 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3399 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3406 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 



 
#line 3430 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3437 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 3448 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3461 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3469 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3476 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3483 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3490 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3497 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3504 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3511 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3518 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3526 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3533 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3540 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3547 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3554 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3561 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3568 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3575 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3582 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3589 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 




 




 




 




 
#line 3629 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3638 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3647 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3656 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3664 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3674 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3683 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3692 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3701 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3710 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3719 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3729 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3738 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3747 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3756 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3765 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3774 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3784 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3793 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3802 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3811 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 




 




 




 




 
#line 3845 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
 
 
 
 
 
#line 3868 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















 
#line 3895 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3902 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3927 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 3935 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 3942 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





#line 3954 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 3967 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4014 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4052 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4078 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 






#line 4092 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4099 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"











#line 4116 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4123 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 







#line 4143 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 4157 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 






#line 4171 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4178 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"











#line 4195 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4202 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 







#line 4222 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 4236 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4280 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 




 




 




 




 




 




 
#line 4333 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 4358 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4368 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4377 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
 
 
 
 

 
#line 4399 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4419 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 




 




 




 




 




 




 
 
 
 
 

 




 
#line 4478 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
#line 4491 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 

 
#line 4509 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4518 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 
#line 4534 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4543 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







 







 





 
 
 
 
 

 






 
#line 4590 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 4603 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 










#line 4641 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 




 




 




 




 




 




 
#line 4695 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4703 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 4716 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
#line 4795 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4836 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 4910 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 
 
 
 
 

 
#line 4936 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

  
#line 4968 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 

                                                                            
#line 4980 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


                                                                                
#line 4990 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
                                                                                
#line 4998 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5019 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
                                                                           
















#line 5042 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 



                                                                          






#line 5063 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"










                                                                           






#line 5086 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5107 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5130 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5151 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5174 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5195 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5218 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5239 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5262 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5283 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5306 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 










#line 5327 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

















#line 5350 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
#line 5392 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 5424 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 5441 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 5467 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





     




 
 




 




 




 




 




 




 




 




 

 




 




 




 




 




 




 




 




 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 




 




 




 




 




 




 




 




 

 




#line 5667 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5685 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5703 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5721 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5739 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5757 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5775 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 




#line 5793 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 

 


#line 5809 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5821 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5833 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5845 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5857 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5869 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5881 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5893 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5905 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5917 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5929 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5941 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5953 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5965 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5977 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 


#line 5989 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



 
 
 
 
 

 
 
#line 6030 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6059 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6109 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 6122 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 6135 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6149 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6163 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6207 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6218 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 6225 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 6232 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6261 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
#line 6279 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6290 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6304 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6318 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6335 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6346 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6360 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6374 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6391 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

   
#line 6402 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6416 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6430 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6444 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6455 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6469 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6483 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6497 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6508 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6522 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6536 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
#line 6545 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6592 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6639 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6686 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6733 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6831 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 6929 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7027 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7125 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7223 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7321 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7419 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7517 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7615 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7713 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7811 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 7909 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8007 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8105 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8203 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8301 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8399 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8497 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8595 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8693 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8791 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8889 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 8987 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9085 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9183 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9281 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9379 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9477 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 

 
#line 9494 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 9501 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 9532 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9552 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9578 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 




 




 




 





 
 
 
 
 

 
#line 9654 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9665 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 9681 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 



#line 9716 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 
#line 9728 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
#line 9777 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9803 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9814 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
 
 
 
 

 
#line 9857 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
#line 9870 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9914 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9937 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"











 
#line 9982 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 9995 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 
 
 
 
 

 




#line 10030 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10044 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"







#line 10078 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
 
 
 
 
 
#line 10091 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 10101 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




#line 10116 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 







 
#line 10138 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10170 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 
#line 10183 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

#line 10202 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 




 

 
#line 10217 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10225 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10233 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10241 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10249 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10257 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10265 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"

 
#line 10273 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"





 



  



 

 









     


 


 

 
#line 10314 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"
  
 






 


 


 



 


 


 



 
 































































































#line 10463 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"



























    





    















 


                                            




 




 




 




                                     




 




 




 




 




 


 


 











  
 
 
 
 
  
 
 

 
#line 10604 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"


 
#line 10623 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h"




 



 





  

  
  
  
   
#line 150 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#line 161 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"



 



   
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 

#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"


































 

 
#line 364 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

 
#line 219 "../Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"










 



 
  



 
#line 47 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"
#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 51 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"

 



 
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

 



























 


#line 120 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"







#line 135 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"


 
#line 157 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"




 









 


#line 190 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"



 



 


#line 207 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h"







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 



 

 



 



 
typedef struct
{
  uint32_t PLLState;      
 

  uint32_t PLLSource;     
           

  uint32_t PLLMUL;        
 
} RCC_PLLInitTypeDef;
   


 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 
} RCC_ClkInitTypeDef;



 

 


 



 






 



 







 



 





 



 






 



 







 



 





 



 






 



 







 



 






 



 






 



 
#line 229 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 
  


 








 



 






 




 





 



 




 



 
#line 287 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"


  
  








 
 




 
#line 313 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

 




 



 

 



 







 
#line 345 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 353 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 361 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 369 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"








 







 

#line 395 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 







 
#line 414 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 422 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 430 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 438 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 446 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 454 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 462 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"












 







 

#line 498 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 







 
#line 517 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 525 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 533 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 541 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 549 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 557 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 565 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 573 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 581 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 588 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"







 
  






 

#line 623 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 




 
#line 638 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"




#line 648 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"






 




 
#line 667 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"





#line 679 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"







 



 










 









   





 



 





 






 




 



 






















 
#line 789 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 



 

















 
#line 837 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 



 






 




 































 








 




 



 








 









 




 



  

#line 954 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"











 








 

  

 






















 

                                                   






 




 




 





 



 




 




 
















 

















 


















 



















 





 
























 






 



 

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



  



 

#line 72 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"






 



 






#line 103 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 125 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"










#line 189 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"






#line 202 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"










 

  



 

#line 240 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 
typedef struct
{
  uint32_t OscillatorType;       
 






  uint32_t HSEState;              
 
                          
  uint32_t HSEPredivValue;       
 

  uint32_t LSEState;              
 
                                          
  uint32_t HSIState;              
 

  uint32_t HSICalibrationValue;   
 
                               
  uint32_t LSIState;              
 

  RCC_PLLInitTypeDef PLL;                




} RCC_OscInitTypeDef;

#line 295 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 
typedef struct
{
  uint32_t PeriphClockSelection;      
 

  uint32_t RTCClockSelection;         
 

  uint32_t AdcClockSelection;         
 

#line 324 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




  uint32_t UsbClockSelection;         
 


} RCC_PeriphCLKInitTypeDef;



 

 



 



 
#line 359 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



 







 

#line 403 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"






 





 




#line 449 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 462 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



#line 489 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 

#line 550 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 

#line 577 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



 
#line 596 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"


 

#line 624 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 

 


 







 

#line 655 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 668 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 681 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 694 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 742 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 







 

#line 782 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 







 

#line 804 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




#line 819 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 827 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 835 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 843 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"







#line 859 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




#line 929 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 968 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1034 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1046 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1076 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 







 

#line 1167 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 







 

#line 1190 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




#line 1223 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1235 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




#line 1260 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1281 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1302 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1332 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 







 

#line 1393 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 

#line 1419 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




 








#line 1440 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"













#line 1471 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1483 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1501 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"







#line 1517 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 




 









#line 1544 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"










#line 1562 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1570 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1578 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"

#line 1588 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



  

#line 1609 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"







 





#line 1630 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"


 






 

#line 1683 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"




   








 







 




#line 1728 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"








 









 




 

#line 1862 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"


 

 


 



 

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



 

#line 1904 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"



 



 
  


 
  






 

#line 1160 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

 


 



 

 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);



 



 

 
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
void              HAL_RCC_DisableCSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void              HAL_RCC_NMI_IRQHandler(void);

 
void              HAL_RCC_CSSCallback(void);



 



 



 



  
  
 

 
#line 1225 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 
  


 
#line 1239 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 




 






 
 


 


 


 



 
 



 



 
 



 



 



 





 
  
 


 


 


 








 



 


 






 

#line 1348 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"

#line 1373 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"



 



 



 
  






 

#line 220 "../Inc/stm32f1xx_hal_conf.h"


#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"



 



 

 


 



 
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 
} GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;


 

 



 



 
#line 116 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"




 










 
#line 138 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"













 




 






 




 





 



 

 


 






 







 







 







 







 



 

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"



 



 
 
 



 




 



 

#line 86 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 103 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"


 



 














 



 




 





 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 







 







 







 







 







 








 






 






 











 









 


#line 363 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 381 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 400 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 418 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"





 






 






 






 


#line 463 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 480 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"





 






 






 







 


#line 528 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 547 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 566 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 583 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 600 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 616 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 633 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 650 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 667 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 684 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 701 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 718 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 735 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 752 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 769 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 788 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"

#line 815 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"



 



 



 
#line 844 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"













#line 863 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"









 

 
 



 



 
void HAL_GPIOEx_ConfigEventout(uint32_t GPIO_PortSource, uint32_t GPIO_PinSource);
void HAL_GPIOEx_EnableEventout(void);
void HAL_GPIOEx_DisableEventout(void);



 



 



 



 







 
#line 230 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



 



 
 
 
 


 



 

 


 
#line 297 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"


 

 


 



 



 



 







 
#line 224 "../Inc/stm32f1xx_hal_conf.h"

   
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h"



 



 

 



 



 
typedef struct
{
  uint32_t Direction;                 

 

  uint32_t PeriphInc;                 
 

  uint32_t MemInc;                    
 

  uint32_t PeriphDataAlignment;       
 

  uint32_t MemDataAlignment;          
 

  uint32_t Mode;                      


 

  uint32_t Priority;                  
 
} DMA_InitTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U    
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER           = 0x00U,     
  HAL_DMA_HALF_TRANSFER           = 0x01U      
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,      
  HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,      
  HAL_DMA_XFER_ALL_CB_ID           = 0x04U       
    
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef   *Instance;                        
  
  DMA_InitTypeDef       Init;                              
  
  HAL_LockTypeDef       Lock;                               
  
  HAL_DMA_StateTypeDef  State;                            
  
  void                  *Parent;                                                         
  
  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);      
  
  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);  
  
  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);     

  void                  (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);       
  
  volatile uint32_t         ErrorCode;                                                     

  DMA_TypeDef            *DmaBaseAddress;                                              
  
  uint32_t               ChannelIndex;                                                   

} DMA_HandleTypeDef;    


 

 



 



 







 



 






 



 




 



 




 



 





 



 





 



 




 



 






 




 





 



 
#line 287 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h"


 



 


 


 




 






 






 



 










 











 











 






 




 

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"



 



 

  
 
 


 
 
#line 179 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"


 





 
#line 196 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"





 
#line 210 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"





 
#line 224 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"





 
#line 238 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h"












 














 




 


  


 



 



 




        



 
#line 373 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h"

 


 



 
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);


 



 
 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, uint32_t CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



 



 
 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


 



 

 


 































  

 



 



 







 
#line 228 "../Inc/stm32f1xx_hal_conf.h"

   



   








#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"

































  

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"



 



  
 


 

#line 92 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"



 

 



 



 
#line 116 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"


 



 





 

#line 261 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"



 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);








 



 

 
 
 
 


 















#line 404 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"



 

 



  



 
  





 

 
#line 244 "../Inc/stm32f1xx_hal_conf.h"














#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h"
   


 



 
  


 



 



 















   

  


   



 
typedef enum 
{
  FLASH_PROC_NONE              = 0U, 
  FLASH_PROC_PAGEERASE         = 1U,
  FLASH_PROC_MASSERASE         = 2U,
  FLASH_PROC_PROGRAMHALFWORD   = 3U,
  FLASH_PROC_PROGRAMWORD       = 4U,
  FLASH_PROC_PROGRAMDOUBLEWORD = 5U
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;  
  
  volatile uint32_t               DataRemaining;     

  volatile uint32_t               Address;           

  volatile uint64_t               Data;              

  HAL_LockTypeDef             Lock;              

  volatile uint32_t               ErrorCode;        
 
} FLASH_ProcessTypeDef;



 

 


   



 








 



  






 




 






 

#line 176 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h"


   
  
 




 
 



 






 







 




 





  
  





  







  




 





    



  





 




 
  


  

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"



 



  



 







   



 





















 





 
#line 104 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"

 






 





 








#line 134 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"

 






 








 
#line 158 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"

 






 
#line 173 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"



   

  


   



 
typedef struct
{
  uint32_t TypeErase;   
 
  
  uint32_t Banks;       
     
  
  uint32_t PageAddress; 

 
  
  uint32_t NbPages;     
 
                                                          
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;  
 

  uint32_t WRPState;    
 

  uint32_t WRPPage;     
 

  uint32_t Banks;        
  
    
  uint8_t RDPLevel;     
 

#line 229 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"
  uint8_t USERConfig;   


 


  uint32_t DATAAddress; 
 
  
  uint8_t DATAData;     
 
} FLASH_OBProgramInitTypeDef;



 

 


   



  



  



        




         
        



 



  





 



 
#line 294 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"


 



 



  



 







 



  





 



 
 
#line 344 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"
        
       
 
#line 373 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"


 
#line 414 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"
         
        


 
 




 
#line 431 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"
       
 
#line 439 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"

 
#line 447 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"
      
 
#line 455 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"



 



 




 
  


  




 



  




  



  




 

#line 506 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"



 




 



 



  




 
#line 545 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"










 
  



 
#line 572 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"







   



 
  



 

 


 




  

#line 691 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"







  









  












 












 
#line 746 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h"





 



 

 


 



 
 
HAL_StatusTypeDef  HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef  HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);



 



 
 
HAL_StatusTypeDef  HAL_FLASHEx_OBErase(void);
HAL_StatusTypeDef  HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void               HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
uint32_t           HAL_FLASHEx_OBGetUserData(uint32_t DATAAdress);


 



 



 



 






 
#line 265 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h"

 


 
  


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

 
void       HAL_FLASH_IRQHandler(void);
  
void       HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void       HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);



 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
void HAL_FLASH_OB_Launch(void);



 



 
 
uint32_t HAL_FLASH_GetError(void);



 



 

 


 
HAL_StatusTypeDef       FLASH_WaitForLastOperation(uint32_t Timeout);






 



 



 







 

#line 260 "../Inc/stm32f1xx_hal_conf.h"


















#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Reload;     
 

} IWDG_InitTypeDef;



 
typedef struct
{
  IWDG_TypeDef                 *Instance;   

  IWDG_InitTypeDef             Init;        

} IWDG_HandleTypeDef;



 

 


 



 
#line 103 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h"


 



 


 


 





 







 




 

 


 



 
 
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);


 



 
 
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


 



 

 


 



 







 

 


 





 






 






 
#line 211 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h"





 




 



 



 








 
#line 280 "../Inc/stm32f1xx_hal_conf.h"


#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h"



 



 

 



  



 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;




 


 



  





 

 
 



  



 
#line 109 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h"
                                                          


 



 
#line 124 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h"



 




 





 



 





 



 





 



 





 



 







 



 

 


 















 







 





 





 





 





 






 






 






 






 





 






 







 





 





 



 

 


 






















 



 



 
  


 

 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);



 



 

 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
 
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);



void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);


 



 



 



 








 
#line 284 "../Inc/stm32f1xx_hal_conf.h"


























#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"



 



 

 


 


 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
									 
  uint32_t AutoReloadPreload;  
 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;




 
typedef struct
{
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;




 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
 
}TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
 
}TIM_ClearInputConfigTypeDef;



 
typedef struct {
  uint32_t  SlaveMode;      
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

}TIM_SlaveConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
}HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
}HAL_TIM_ActiveChannel;



 
typedef struct
{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7U];     
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          
}TIM_HandleTypeDef;



 

 


 



 





 



 




 



 






 



 







 



 





 



 




 



 
#line 379 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 




 



 







 



 






 



 




 



 





 



 
#line 520 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 





 



 
#line 544 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 
#line 559 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 
#line 578 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 
#line 595 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 







 



 






 



 




 



 




 



 






 



 




 



 




 



 






 



 




 



 




 


 




 



 
#line 717 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 







 



 




 



 
#line 754 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 







 



 






 



 




 



 
#line 812 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 
#line 837 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 
#line 851 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"


 



 






 



 

 


 


 





 

 


 
















                              
#line 911 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"




















































#line 973 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"















































#line 1028 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"










#line 1046 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"























#line 1088 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"

#line 1107 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"








 










 












 










 








 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);


 

 


 




 






 






 






 
#line 1216 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"

 





 
#line 1234 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"






 
















 
















 















 















 




















 




















 







 







 








 







 














 













 








 






 









 










 













 
#line 1468 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"








 




















 




















 













 













 


















 








 

 
#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim_ex.h"



 



  

  


 




 

typedef struct
{

  uint32_t IC1Polarity;            
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
   
  uint32_t Commutation_Delay;  
 
} TIM_HallSensor_InitTypeDef;








  
typedef struct
{
  uint32_t OffStateRunMode;       
 
  uint32_t OffStateIDLEMode;      
 
  uint32_t LockLevel;             
                              
  uint32_t DeadTime;              
 
  uint32_t BreakState;            
 
  uint32_t BreakPolarity;         
 
  uint32_t AutomaticOutput;       
            
} TIM_BreakDeadTimeConfigTypeDef;


        
        



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
  
  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;



  

 





 
    


 



 



 

        
        

 










 
















 






 


 



 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 







 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 

        
        



 
 



HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);

        
        
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef * sMasterConfig);


 



 
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);


 






 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);


 

        
        



  
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


  
 



  



 
  







 
#line 1578 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"

 


 



 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 



 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);


 



 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);



 



 



 



 







 
#line 312 "../Inc/stm32f1xx_hal_conf.h"


#line 1 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"

































 

 







 
#line 46 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"



 



 

  


 



 
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 
 

  uint32_t OverSampling;              

 
}UART_InitTypeDef;







































 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    
 
  HAL_UART_STATE_READY             = 0x20U,    
 
  HAL_UART_STATE_BUSY              = 0x24U,    
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,    
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,    
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_UART_STATE_ERROR             = 0xE0U     
 
}HAL_UART_StateTypeDef;



 
typedef struct
{
  USART_TypeDef                 *Instance;         

  UART_InitTypeDef              Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  volatile uint16_t                 TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  volatile uint16_t                 RxXferCount;       

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 
  
  volatile HAL_UART_StateTypeDef    RxState;          
 

  volatile uint32_t                 ErrorCode;         
}UART_HandleTypeDef;



 

 


 



 
#line 209 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"


 



 




 



 




 



 





 



 






 



  





 



 




 




 






 




   




 


 




 





 
#line 316 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"


 









 













 



 

 


 





 









 



















 






















 






 
#line 425 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"





 






 






 






 
















 


















 


















 















 


















 


















 


















 






#line 598 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"



 




 



 
 


 



 
 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback (UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback (UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);


 



 
 
 
 


 


 







 

 


 
#line 740 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"





 








 





 

 


 



 



  



 







 
#line 316 "../Inc/stm32f1xx_hal_conf.h"

























   

 
#line 359 "../Inc/stm32f1xx_hal_conf.h"







 
#line 47 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"



 



 

 
 



 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 
 


 











 

 


 





 






 




#line 125 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 133 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 141 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 149 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 157 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 165 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"



 





 





 






 







 




#line 207 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

 



 




#line 224 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 232 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 240 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 248 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"


#line 257 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 265 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"

#line 273 "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"



 



 





 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_GetUID(uint32_t *UID);


 



 
 
 


 


 
 


 


 
 
 


 



 







 
#line 128 "../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c"



 




 



 
 
 
 
 


 
static void TIM_OC1_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_OC3_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_OC4_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_TI1_ConfigInputStage(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICFilter);
static void TIM_TI2_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter);
static void TIM_TI2_ConfigInputStage(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICFilter);
static void TIM_TI3_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter);
static void TIM_TI4_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter);
static void TIM_ETR_SetConfig(TIM_TypeDef* TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);
static void TIM_ITRx_SetConfig(TIM_TypeDef* TIMx, uint16_t InputTriggerSource);
static void TIM_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma);
static void TIM_DMATriggerCplt(DMA_HandleTypeDef *hdma);
static void TIM_SlaveTimer_SetConfig(TIM_HandleTypeDef *htim,
                                     TIM_SlaveConfigTypeDef * sSlaveConfig);



 

 



 





















 





 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim)
{
   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_Base_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_Base_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}






 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  htim->State= HAL_TIM_STATE_READY;

   
  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State= HAL_TIM_STATE_READY;

   
  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

    
   ((htim)->Instance ->DIER |= (((0x1U << (0U)))));

    
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);
   
  ((htim)->Instance ->DIER &= ~(((0x1U << (0U)))));

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}







 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length)
{
   
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((pData == 0U) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
   
  htim->hdma[((uint16_t)0x0)]->XferCpltCallback = TIM_DMAPeriodElapsedCplt;

   
  htim->hdma[((uint16_t)0x0)]->XferErrorCallback = TIM_DMAError ;

   
  HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x0)], (uint32_t)pData, (uint32_t)&htim->Instance->ARR, Length);

   
  ((htim)->Instance ->DIER |= (((0x1U << (8U)))));

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (8U)))));

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}



 





















 





 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef* htim)
{
   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_OC_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  TIM_Base_SetConfig(htim->Instance,  &htim->Init);

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

   htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_OC_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}











 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
   
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if(((uint32_t)pData == 0U) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  switch (Channel)
  {
    case 0x00000000U:
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      htim->hdma[((uint16_t)0x3)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x3)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x3)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3,Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
      
      htim->hdma[((uint16_t)0x4)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x4)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x4)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR4, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}



 





















 





 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim)
{
   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_PWM_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_PWM_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}











 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT (TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
   
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if(((uint32_t)pData == 0U) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  switch (Channel)
  {
    case 0x00000000U:
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      htim->hdma[((uint16_t)0x3)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x3)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x3)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3,Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
      
      htim->hdma[((uint16_t)0x4)]->XferCpltCallback = TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x4)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x4)], (uint32_t)pData, (uint32_t)&htim->Instance->CCR4, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}



 





















 





 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim)
{
   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_IC_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_IC_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}











 
HAL_StatusTypeDef HAL_TIM_IC_Start (TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT (TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER |= (((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }
   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (3U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (4U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
   
  ((void)0U);
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((pData == 0U) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)&htim->Instance->CCR1, (uint32_t)pData, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)&htim->Instance->CCR2, (uint32_t)pData, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      htim->hdma[((uint16_t)0x3)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x3)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x3)], (uint32_t)&htim->Instance->CCR3, (uint32_t)pData, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      htim->hdma[((uint16_t)0x4)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x4)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x4)], (uint32_t)&htim->Instance->CCR4, (uint32_t)pData, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000001U);

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}











 
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);
  ((void)0U);

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (9U)))));
    }
    break;

    case 0x00000004U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (10U)))));
    }
    break;

    case 0x00000008U:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (11U)))));
    }
    break;

    case 0x0000000CU:
    {
       
      ((htim)->Instance ->DIER &= ~(((0x1U << (12U)))));
    }
    break;

    default:
    break;
  }

   
  TIM_CCxChannelCmd(htim->Instance, Channel, 0x00000000U);

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}


 





















 









 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode)
{
   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_OnePulse_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

   
  htim->Instance->CR1 &= ~(0x1U << (3U));

   
  htim->Instance->CR1 |= OnePulseMode;

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_OnePulse_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}









 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel)
{
   
  (void)OutputChannel;

  






 

  TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
  TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  return HAL_OK;
}









 
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel)
{
   
  (void)OutputChannel;

  



 

  TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
  TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}









 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel)
{
   
  (void)OutputChannel;

  






 

   
  ((htim)->Instance ->DIER |= (((0x1U << (1U)))));

   
  ((htim)->Instance ->DIER |= (((0x1U << (2U)))));

  TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
  TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    ((htim)->Instance ->BDTR|=((0x1U << (15U))));
  }

   
  return HAL_OK;
}









 
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel)
{
   
  (void)OutputChannel;

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));

  



 
  TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
  TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

  if(((htim->Instance) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) != RESET)
  {
     
    do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->BDTR &= ~((0x1U << (15U))); } } } while(0U);
  }

   
   do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}



 





















 





 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig)
{
  uint32_t tmpsmcr = 0U;
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  if(htim == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if(htim->State == HAL_TIM_STATE_RESET)
  {
     
    htim->Lock = HAL_UNLOCKED;
    
     
    HAL_TIM_Encoder_MspInit(htim);
  }

   
  htim->State= HAL_TIM_STATE_BUSY;

   
  htim->Instance->SMCR &= ~(0x7U << (0U));

   
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

   
  tmpsmcr = htim->Instance->SMCR;

   
  tmpccmr1 = htim->Instance->CCMR1;

   
  tmpccer = htim->Instance->CCER;

   
  tmpsmcr |= sConfig->EncoderMode;

   
  tmpccmr1 &= ~((0x3U << (0U)) | (0x3U << (8U)));
  tmpccmr1 |= (sConfig->IC1Selection | (sConfig->IC2Selection << 8U));

   
  tmpccmr1 &= ~((0x3U << (2U)) | (0x3U << (10U)));
  tmpccmr1 &= ~((0xFU << (4U)) | (0xFU << (12U)));
  tmpccmr1 |= sConfig->IC1Prescaler | (sConfig->IC2Prescaler << 8U);
  tmpccmr1 |= (sConfig->IC1Filter << 4U) | (sConfig->IC2Filter << 12U);

   
  tmpccer &= ~((0x1U << (1U)) | (0x1U << (5U)));
  tmpccer &= ~((0x1U << (3U)) | (0x1U << (7U)));
  tmpccer |= sConfig->IC1Polarity | (sConfig->IC2Polarity << 4U);

   
  htim->Instance->SMCR = tmpsmcr;

   
  htim->Instance->CCMR1 = tmpccmr1;

   
  htim->Instance->CCER = tmpccer;

   
  htim->State= HAL_TIM_STATE_READY;

  return HAL_OK;
}






 
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim)
{
   
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  HAL_TIM_Encoder_MspDeInit(htim);

   
  htim->State = HAL_TIM_STATE_RESET;

   
  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}





 
__weak void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}










 
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
  switch (Channel)
  {
    case 0x00000000U:
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
      break;
  }
    case 0x00000004U:
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);
      break;
  }
    default :
  {
     TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
     TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);
     break;
    }
  }
   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}










 
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
    ((void)0U);

   
 
  switch (Channel)
  {
    case 0x00000000U:
  {
     TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
      break;
  }
    case 0x00000004U:
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);
      break;
  }
    default :
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);
     break;
    }
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  return HAL_OK;
}










 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

   
   
  switch (Channel)
  {
    case 0x00000000U:
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
    ((htim)->Instance ->DIER |= (((0x1U << (1U)))));
      break;
  }
    case 0x00000004U:
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);
    ((htim)->Instance ->DIER |= (((0x1U << (2U)))));
      break;
  }
    default :
  {
     TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
     TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);
     ((htim)->Instance ->DIER |= (((0x1U << (1U)))));
     ((htim)->Instance ->DIER |= (((0x1U << (2U)))));
     break;
    }
  }

   
  ((htim)->Instance ->CR1|=((0x1U << (0U))));

   
  return HAL_OK;
}










 
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  
 
  if(Channel == 0x00000000U)
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);

     
  ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));
  }
  else if(Channel == 0x00000004U)
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

     
  ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));
  }
  else
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

     
    ((htim)->Instance ->DIER &= ~(((0x1U << (1U)))));
    ((htim)->Instance ->DIER &= ~(((0x1U << (2U)))));
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length)
{
   
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((((pData1 == 0U) || (pData2 == 0U) )) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }

  switch (Channel)
  {
    case 0x00000000U:
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)&htim->Instance->CCR1, (uint32_t )pData1, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (9U)))));

       
      ((htim)->Instance ->CR1|=((0x1U << (0U))));

       
      TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
    }
    break;

    case 0x00000004U:
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError;
       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)&htim->Instance->CCR2, (uint32_t)pData2, Length);

       
      ((htim)->Instance ->DIER |= (((0x1U << (10U)))));

       
      ((htim)->Instance ->CR1|=((0x1U << (0U))));

       
      TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);
    }
    break;

    case 0x00000018U:
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)&htim->Instance->CCR1, (uint32_t)pData1, Length);

       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback = TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)&htim->Instance->CCR2, (uint32_t)pData2, Length);

      
      ((htim)->Instance ->CR1|=((0x1U << (0U))));

       
      TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000001U);
      TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000001U);

       
      ((htim)->Instance ->DIER |= (((0x1U << (9U)))));
       
      ((htim)->Instance ->DIER |= (((0x1U << (10U)))));
    }
    break;

    default:
    break;
  }
   
  return HAL_OK;
}










 
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   
  ((void)0U);

  
 
  if(Channel == 0x00000000U)
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);

     
    ((htim)->Instance ->DIER &= ~(((0x1U << (9U)))));
  }
  else if(Channel == 0x00000004U)
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

     
    ((htim)->Instance ->DIER &= ~(((0x1U << (10U)))));
  }
  else
  {
    TIM_CCxChannelCmd(htim->Instance, 0x00000000U, 0x00000000U);
    TIM_CCxChannelCmd(htim->Instance, 0x00000004U, 0x00000000U);

     
    ((htim)->Instance ->DIER &= ~(((0x1U << (9U)))));
    ((htim)->Instance ->DIER &= ~(((0x1U << (10U)))));
  }

   
  do { if (((htim)->Instance ->CCER & ((uint32_t)((0x1U << (0U)) | (0x1U << (4U)) | (0x1U << (8U)) | (0x1U << (12U))))) == 0U) { if(((htim)->Instance ->CCER & ((uint32_t)((0x1U << (2U)) | (0x1U << (6U)) | (0x1U << (10U))))) == 0U) { (htim)->Instance ->CR1 &= ~((0x1U << (0U))); } } } while(0U);

   
  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}



 












 




 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
   
  if((((htim)->Instance ->SR &(((0x1U << (1U))))) == (((0x1U << (1U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (1U))))) == (((0x1U << (1U))))) ? SET : RESET) !=RESET)
    {
      {
        ((htim)->Instance ->SR = ~(((0x1U << (1U)))));
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;

         
        if((htim->Instance->CCMR1 & (0x3U << (0U))) != 0x00U)
        {
          HAL_TIM_IC_CaptureCallback(htim);
        }
         
        else
        {
          HAL_TIM_OC_DelayElapsedCallback(htim);
          HAL_TIM_PWM_PulseFinishedCallback(htim);
        }
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
      }
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (2U))))) == (((0x1U << (2U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (2U))))) == (((0x1U << (2U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (2U)))));
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
       
      if((htim->Instance->CCMR1 & (0x3U << (8U))) != 0x00U)
      {
        HAL_TIM_IC_CaptureCallback(htim);
      }
       
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (3U))))) == (((0x1U << (3U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (3U))))) == (((0x1U << (3U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (3U)))));
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
       
      if((htim->Instance->CCMR2 & (0x3U << (0U))) != 0x00U)
      {
        HAL_TIM_IC_CaptureCallback(htim);
      }
       
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (4U))))) == (((0x1U << (4U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (4U))))) == (((0x1U << (4U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (4U)))));
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
       
      if((htim->Instance->CCMR2 & (0x3U << (8U))) != 0x00U)
      {
        HAL_TIM_IC_CaptureCallback(htim);
      }
       
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (0U))))) == (((0x1U << (0U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (0U))))) == (((0x1U << (0U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (0U)))));
      HAL_TIM_PeriodElapsedCallback(htim);
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (7U))))) == (((0x1U << (7U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (7U))))) == (((0x1U << (7U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (7U)))));
      HAL_TIMEx_BreakCallback(htim);
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (6U))))) == (((0x1U << (6U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (6U))))) == (((0x1U << (6U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (6U)))));
      HAL_TIM_TriggerCallback(htim);
    }
  }
   
  if((((htim)->Instance ->SR &(((0x1U << (5U))))) == (((0x1U << (5U))))) != RESET)
  {
    if(((((htim)->Instance ->DIER & (((0x1U << (5U))))) == (((0x1U << (5U))))) ? SET : RESET) !=RESET)
    {
      ((htim)->Instance ->SR = ~(((0x1U << (5U)))));
      HAL_TIMEx_CommutationCallback(htim);
    }
  }
}



 


















 













 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel)
{
   
  ((void)0U);
  ((void)0U);
  ((void)0U);

   
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

  switch (Channel)
  {
    case 0x00000000U:
    {
      ((void)0U);
       
      TIM_OC1_SetConfig(htim->Instance, sConfig);
    }
    break;

    case 0x00000004U:
    {
      ((void)0U);
       
      TIM_OC2_SetConfig(htim->Instance, sConfig);
    }
    break;

    case 0x00000008U:
    {
       ((void)0U);
       
      TIM_OC3_SetConfig(htim->Instance, sConfig);
    }
    break;

    case 0x0000000CU:
    {
      ((void)0U);
        
       TIM_OC4_SetConfig(htim->Instance, sConfig);
    }
    break;

    default:
    break;
  }
  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel)
{
   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

  if (Channel == 0x00000000U)
  {
     
    TIM_TI1_SetConfig(htim->Instance,
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);

     
    htim->Instance->CCMR1 &= ~(0x3U << (2U));

     
    htim->Instance->CCMR1 |= sConfig->ICPrescaler;
  }
  else if (Channel == 0x00000004U)
  {
     
    ((void)0U);

    TIM_TI2_SetConfig(htim->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

     
    htim->Instance->CCMR1 &= ~(0x3U << (10U));

     
    htim->Instance->CCMR1 |= (sConfig->ICPrescaler << 8U);
  }
  else if (Channel == 0x00000008U)
  {
     
    ((void)0U);

    TIM_TI3_SetConfig(htim->Instance,
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);

     
    htim->Instance->CCMR2 &= ~(0x3U << (2U));

     
    htim->Instance->CCMR2 |= sConfig->ICPrescaler;
  }
  else
  {
     
    ((void)0U);

    TIM_TI4_SetConfig(htim->Instance,
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);

     
    htim->Instance->CCMR2 &= ~(0x3U << (10U));

     
    htim->Instance->CCMR2 |= (sConfig->ICPrescaler << 8U);
  }

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}













 
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel)
{
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  htim->State = HAL_TIM_STATE_BUSY;

  switch (Channel)
  {
    case 0x00000000U:
    {
      ((void)0U);
       
      TIM_OC1_SetConfig(htim->Instance, sConfig);

       
      htim->Instance->CCMR1 |= (0x1U << (3U));

       
      htim->Instance->CCMR1 &= ~(0x1U << (2U));
      htim->Instance->CCMR1 |= sConfig->OCFastMode;
    }
    break;

    case 0x00000004U:
    {
      ((void)0U);
       
      TIM_OC2_SetConfig(htim->Instance, sConfig);

       
      htim->Instance->CCMR1 |= (0x1U << (11U));

       
      htim->Instance->CCMR1 &= ~(0x1U << (10U));
      htim->Instance->CCMR1 |= sConfig->OCFastMode << 8;
    }
    break;

    case 0x00000008U:
    {
      ((void)0U);
       
      TIM_OC3_SetConfig(htim->Instance, sConfig);

       
      htim->Instance->CCMR2 |= (0x1U << (3U));

      
      htim->Instance->CCMR2 &= ~(0x1U << (2U));
      htim->Instance->CCMR2 |= sConfig->OCFastMode;
    }
    break;

    case 0x0000000CU:
    {
      ((void)0U);
       
      TIM_OC4_SetConfig(htim->Instance, sConfig);

       
      htim->Instance->CCMR2 |= (0x1U << (11U));

      
      htim->Instance->CCMR2 &= ~(0x1U << (10U));
      htim->Instance->CCMR2 |= sConfig->OCFastMode << 8;
    }
    break;

    default:
    break;
  }

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}















 
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim,  TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel)
{
  TIM_OC_InitTypeDef temp1;

   
  ((void)0U);
  ((void)0U);

  if(OutputChannel != InputChannel)
  {
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  temp1.OCMode = sConfig->OCMode;
  temp1.Pulse = sConfig->Pulse;
  temp1.OCPolarity = sConfig->OCPolarity;
  temp1.OCNPolarity = sConfig->OCNPolarity;
  temp1.OCIdleState = sConfig->OCIdleState;
  temp1.OCNIdleState = sConfig->OCNIdleState;

    switch (OutputChannel)
  {
    case 0x00000000U:
    {
        ((void)0U);

      TIM_OC1_SetConfig(htim->Instance, &temp1);
    }
    break;
    case 0x00000004U:
    {
        ((void)0U);

      TIM_OC2_SetConfig(htim->Instance, &temp1);
    }
    break;
    default:
    break;
  }
  switch (InputChannel)
  {
    case 0x00000000U:
    {
      ((void)0U);

      TIM_TI1_SetConfig(htim->Instance, sConfig->ICPolarity,
                        sConfig->ICSelection, sConfig->ICFilter);

       
    htim->Instance->CCMR1 &= ~(0x3U << (2U));

       
        htim->Instance->SMCR &= ~(0x7U << (4U));
      htim->Instance->SMCR |= 0x00000050U;

       
        htim->Instance->SMCR &= ~(0x7U << (0U));
      htim->Instance->SMCR |= 0x00000006U;
    }
    break;
    case 0x00000004U:
    {
      ((void)0U);

      TIM_TI2_SetConfig(htim->Instance, sConfig->ICPolarity,
                 sConfig->ICSelection, sConfig->ICFilter);

       
        htim->Instance->CCMR1 &= ~(0x3U << (10U));

       
        htim->Instance->SMCR &= ~(0x7U << (4U));
      htim->Instance->SMCR |= 0x00000060U;

       
        htim->Instance->SMCR &= ~(0x7U << (0U));
      htim->Instance->SMCR |= 0x00000006U;
    }
    break;

    default:
    break;
  }

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}
  else
  {
    return HAL_ERROR;
  }
}






































 
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,
                                              uint32_t* BurstBuffer, uint32_t  BurstLength)
{
   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((BurstBuffer == 0U) && (BurstLength > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  switch(BurstRequestSrc)
  {
    case ((0x1U << (8U))):
    {
       
      htim->hdma[((uint16_t)0x0)]->XferCpltCallback = TIM_DMAPeriodElapsedCplt;

       
      htim->hdma[((uint16_t)0x0)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x0)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (9U))):
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback =  TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (10U))):
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback =  TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (11U))):
    {
       
      htim->hdma[((uint16_t)0x3)]->XferCpltCallback =  TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x3)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x3)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (12U))):
    {
       
      htim->hdma[((uint16_t)0x4)]->XferCpltCallback =  TIM_DMADelayPulseCplt;

       
      htim->hdma[((uint16_t)0x4)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x4)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (13U))):
    {
       
      htim->hdma[((uint16_t)0x5)]->XferCpltCallback =  TIMEx_DMACommutationCplt;

       
      htim->hdma[((uint16_t)0x5)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x5)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (14U))):
    {
       
      htim->hdma[((uint16_t)0x6)]->XferCpltCallback = TIM_DMATriggerCplt;

       
      htim->hdma[((uint16_t)0x6)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x6)], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, ((BurstLength) >> 8U) + 1U);
    }
    break;
    default:
    break;
  }
    
   htim->Instance->DCR = BurstBaseAddress | BurstLength;

    
   ((htim)->Instance ->DIER |= (BurstRequestSrc));

   htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}






 
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc)
{
   
  ((void)0U);

   
  switch(BurstRequestSrc)
  {
    case ((0x1U << (8U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x0)]);
    }
    break;
    case ((0x1U << (9U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x1)]);
    }
    break;
    case ((0x1U << (10U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x2)]);
    }
    break;
    case ((0x1U << (11U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x3)]);
    }
    break;
    case ((0x1U << (12U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x4)]);
    }
    break;
    case ((0x1U << (13U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x5)]);
    }
    break;
    case ((0x1U << (14U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x6)]);
    }
    break;
    default:
    break;
  }

   
  ((htim)->Instance ->DIER &= ~(BurstRequestSrc));

   
  return HAL_OK;
}






































 
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,
                                             uint32_t  *BurstBuffer, uint32_t  BurstLength)
{
   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

  if((htim->State == HAL_TIM_STATE_BUSY))
  {
     return HAL_BUSY;
  }
  else if((htim->State == HAL_TIM_STATE_READY))
  {
    if((BurstBuffer == 0U) && (BurstLength > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  switch(BurstRequestSrc)
  {
    case ((0x1U << (8U))):
    {
       
      htim->hdma[((uint16_t)0x0)]->XferCpltCallback = TIM_DMAPeriodElapsedCplt;

       
      htim->hdma[((uint16_t)0x0)]->XferErrorCallback = TIM_DMAError ;

       
       HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x0)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (9U))):
    {
       
      htim->hdma[((uint16_t)0x1)]->XferCpltCallback =  TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x1)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x1)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (10U))):
    {
       
      htim->hdma[((uint16_t)0x2)]->XferCpltCallback =  TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x2)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x2)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (11U))):
    {
       
      htim->hdma[((uint16_t)0x3)]->XferCpltCallback =  TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x3)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x3)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (12U))):
    {
       
      htim->hdma[((uint16_t)0x4)]->XferCpltCallback =  TIM_DMACaptureCplt;

       
      htim->hdma[((uint16_t)0x4)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x4)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (13U))):
    {
       
      htim->hdma[((uint16_t)0x5)]->XferCpltCallback =  TIMEx_DMACommutationCplt;

       
      htim->hdma[((uint16_t)0x5)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x5)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    case ((0x1U << (14U))):
    {
       
      htim->hdma[((uint16_t)0x6)]->XferCpltCallback = TIM_DMATriggerCplt;

       
      htim->hdma[((uint16_t)0x6)]->XferErrorCallback = TIM_DMAError ;

       
      HAL_DMA_Start_IT(htim->hdma[((uint16_t)0x6)], (uint32_t)&htim->Instance->DMAR, (uint32_t)BurstBuffer, ((BurstLength) >> 8U) + 1U);
    }
    break;
    default:
    break;
  }

   
  htim->Instance->DCR = BurstBaseAddress | BurstLength;

   
  ((htim)->Instance ->DIER |= (BurstRequestSrc));

  htim->State = HAL_TIM_STATE_READY;

   
  return HAL_OK;
}






 
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc)
{
   
  ((void)0U);

   
  switch(BurstRequestSrc)
  {
    case ((0x1U << (8U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x0)]);
    }
    break;
    case ((0x1U << (9U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x1)]);
    }
    break;
    case ((0x1U << (10U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x2)]);
    }
    break;
    case ((0x1U << (11U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x3)]);
    }
    break;
    case ((0x1U << (12U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x4)]);
    }
    break;
    case ((0x1U << (13U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x5)]);
    }
    break;
    case ((0x1U << (14U))):
    {
      HAL_DMA_Abort(htim->hdma[((uint16_t)0x6)]);
    }
    break;
    default:
    break;
  }

   
  ((htim)->Instance ->DIER &= ~(BurstRequestSrc));

   
  return HAL_OK;
}

















 

HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource)
{
   
  ((void)0U);
  ((void)0U);

   
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

   
  htim->State = HAL_TIM_STATE_BUSY;

   
  htim->Instance->EGR = EventSource;

   
  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

   
  return HAL_OK;
}













  
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel)
{
  uint32_t tmpsmcr = 0U;

   
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);
  ((void)0U);

   
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

  switch (sClearInputConfig->ClearInputSource)
  {
    case 0x00000000U:
    {

       
      tmpsmcr &= ~((0xFU << (8U)) | (0x3U << (12U)) | (0x1U << (14U)) | (0x1U << (15U)));

       
      htim->Instance->SMCR = tmpsmcr;
   }
    break;

    case 0x00000001U:
    {
      TIM_ETR_SetConfig(htim->Instance,
                        sClearInputConfig->ClearInputPrescaler,
                        sClearInputConfig->ClearInputPolarity,
                        sClearInputConfig->ClearInputFilter);

    }
    break;
    default:
    break;
  }

  switch (Channel)
  {
    case 0x00000000U:
      {
        if(sClearInputConfig->ClearInputState != RESET)
        {
           
          htim->Instance->CCMR1 |= (0x1U << (7U));
        }
        else
        {
           
        htim->Instance->CCMR1 &= ~(0x1U << (7U));
        }
      }
      break;
    case 0x00000004U:
      {
        ((void)0U);
        if(sClearInputConfig->ClearInputState != RESET)
        {
           
          htim->Instance->CCMR1 |= (0x1U << (15U));
        }
        else
        {
           
          htim->Instance->CCMR1 &= ~(0x1U << (15U));
        }
      }
    break;
    case 0x00000008U:
      {
        ((void)0U);
        if(sClearInputConfig->ClearInputState != RESET)
        {
           
          htim->Instance->CCMR2 |= (0x1U << (7U));
        }
        else
        {
           
        htim->Instance->CCMR2 &= ~(0x1U << (7U));
        }
      }
    break;
    case 0x0000000CU:
      {
        ((void)0U);
        if(sClearInputConfig->ClearInputState != RESET)
        {
           
          htim->Instance->CCMR2 |= (0x1U << (15U));
        }
        else
        {
           
        htim->Instance->CCMR2 &= ~(0x1U << (15U));
        }
      }
    break;
    default:
    break;
  }

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}







  
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig)
{
  uint32_t tmpsmcr = 0U;

   
  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

   
  ((void)0U);

   
  tmpsmcr = htim->Instance->SMCR;
  tmpsmcr &= ~((0x7U << (0U)) | (0x7U << (4U)));
  tmpsmcr &= ~((0xFU << (8U)) | (0x3U << (12U)) | (0x1U << (14U)) | (0x1U << (15U)));
  htim->Instance->SMCR = tmpsmcr;

  switch (sClockSourceConfig->ClockSource)
  {
  case ((0x1U << (12U))):
    {
      ((void)0U);
       
      htim->Instance->SMCR &= ~(0x7U << (0U));
    }
    break;

  case ((0x7U << (4U))):
    {
       
      ((void)0U);

       
      ((void)0U);
      ((void)0U);
      ((void)0U);
      
       
      TIM_ETR_SetConfig(htim->Instance,
                        sClockSourceConfig->ClockPrescaler,
                        sClockSourceConfig->ClockPolarity,
                        sClockSourceConfig->ClockFilter);
       
      tmpsmcr = htim->Instance->SMCR;
       
      tmpsmcr &= ~((0x7U << (0U)) | (0x7U << (4U)));
       
      tmpsmcr |= (0x00000007U | ((0x7U << (4U))));
       
      htim->Instance->SMCR = tmpsmcr;
    }
    break;

  case ((0x2U << (12U))):
    {
       
      ((void)0U);

       
      ((void)0U);
      ((void)0U);
      ((void)0U);
      
       
      TIM_ETR_SetConfig(htim->Instance,
                        sClockSourceConfig->ClockPrescaler,
                        sClockSourceConfig->ClockPolarity,
                        sClockSourceConfig->ClockFilter);
       
      htim->Instance->SMCR |= (0x1U << (14U));
    }
    break;

  case ((0x1U << (4U)) | (0x4U << (4U))):
    {
       
      ((void)0U);

       
      ((void)0U);
      ((void)0U);
      
      TIM_TI1_ConfigInputStage(htim->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TIM_ITRx_SetConfig(htim->Instance, ((0x1U << (4U)) | (0x4U << (4U))));
    }
    break;
  case ((0x2U << (4U)) | (0x4U << (4U))):
    {
       
      ((void)0U);

        
      ((void)0U);
      ((void)0U);

      TIM_TI2_ConfigInputStage(htim->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TIM_ITRx_SetConfig(htim->Instance, ((0x2U << (4U)) | (0x4U << (4U))));
    }
    break;
  case ((0x4U << (4U))):
    {
       
      ((void)0U);

       
      ((void)0U);
      ((void)0U);

      TIM_TI1_ConfigInputStage(htim->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TIM_ITRx_SetConfig(htim->Instance, ((0x4U << (4U))));
    }
    break;
  case ((uint32_t)0x0000):
    {
       
      ((void)0U);

      TIM_ITRx_SetConfig(htim->Instance, ((uint32_t)0x0000));
    }
    break;
  case ((0x1U << (4U))):
    {
       
      ((void)0U);

      TIM_ITRx_SetConfig(htim->Instance, ((0x1U << (4U))));
    }
    break;
  case ((0x2U << (4U))):
    {
       
      ((void)0U);

      TIM_ITRx_SetConfig(htim->Instance, ((0x2U << (4U))));
    }
    break;
  case ((0x1U << (4U)) | (0x2U << (4U))):
    {
       
      ((void)0U);

      TIM_ITRx_SetConfig(htim->Instance, ((0x1U << (4U)) | (0x2U << (4U))));
    }
    break;

  default:
    break;
  }
  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}












 
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection)
{
  uint32_t tmpcr2 = 0U;

   
  ((void)0U);
  ((void)0U);

   
  tmpcr2 = htim->Instance->CR2;

   
  tmpcr2 &= ~(0x1U << (7U));

   
  tmpcr2 |= TI1_Selection;

   
  htim->Instance->CR2 = tmpcr2;

  return HAL_OK;
}









 
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig)
{
   
  ((void)0U);
  ((void)0U);
  ((void)0U);

  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

  TIM_SlaveTimer_SetConfig(htim, sSlaveConfig);

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (6U)))));

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (14U)))));

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
    }









 
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim,
                                                        TIM_SlaveConfigTypeDef * sSlaveConfig)
    {
       
  ((void)0U);
  ((void)0U);
  ((void)0U);

  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  htim->State = HAL_TIM_STATE_BUSY;

  TIM_SlaveTimer_SetConfig(htim, sSlaveConfig);

   
  ((htim)->Instance ->DIER |= (((0x1U << (6U)))));

   
  ((htim)->Instance ->DIER &= ~(((0x1U << (14U)))));

  htim->State = HAL_TIM_STATE_READY;

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);

  return HAL_OK;
}











 
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  uint32_t tmpreg = 0U;

  do{ if((htim)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (htim)->Lock = HAL_LOCKED; } }while (0U);

  switch (Channel)
  {
  case 0x00000000U:
    {
       
      ((void)0U);

       
      tmpreg =  htim->Instance->CCR1;

      break;
    }
  case 0x00000004U:
    {
       
      ((void)0U);

       
      tmpreg =   htim->Instance->CCR2;

      break;
    }

  case 0x00000008U:
    {
       
      ((void)0U);

       
      tmpreg =   htim->Instance->CCR3;

      break;
    }

  case 0x0000000CU:
    {
       
      ((void)0U);

       
      tmpreg =   htim->Instance->CCR4;

      break;
    }

  default:
    break;
  }

  do{ (htim)->Lock = HAL_UNLOCKED; }while (0U);
  return tmpreg;
}



 


















 





 
__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 

}




 
__weak void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}




 
__weak void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}





 
__weak void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
   
  (void)htim;
  

 
}



 














 





 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}





 
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}





 
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}





 
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}





 
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}





 
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim)
{
  return htim->State;
}



 



 



 





 
void TIM_DMAError(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  htim->State= HAL_TIM_STATE_READY;

  HAL_TIM_ErrorCallback(htim);
}





 
void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  htim->State= HAL_TIM_STATE_READY;

  if (hdma == htim->hdma[((uint16_t)0x1)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
  }
  else if (hdma == htim->hdma[((uint16_t)0x2)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
  }
  else if (hdma == htim->hdma[((uint16_t)0x3)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
  }
  else if (hdma == htim->hdma[((uint16_t)0x4)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
  }

  HAL_TIM_PWM_PulseFinishedCallback(htim);

  htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}




 
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  htim->State= HAL_TIM_STATE_READY;

  if (hdma == htim->hdma[((uint16_t)0x1)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
  }
  else if (hdma == htim->hdma[((uint16_t)0x2)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
  }
  else if (hdma == htim->hdma[((uint16_t)0x3)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
  }
  else if (hdma == htim->hdma[((uint16_t)0x4)])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
  }

  HAL_TIM_IC_CaptureCallback(htim);

  htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}





 
static void TIM_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  htim->State= HAL_TIM_STATE_READY;

  HAL_TIM_PeriodElapsedCallback(htim);
}





 
static void TIM_DMATriggerCplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  htim->State= HAL_TIM_STATE_READY;

  HAL_TIM_TriggerCallback(htim);
}






 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure)
{
  uint32_t tmpcr1 = 0U;
  tmpcr1 = TIMx->CR1;

   
  if ((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000000U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000400U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000800U)))))
  {
     
    tmpcr1 &= ~((0x1U << (4U)) | (0x3U << (5U)));
    tmpcr1 |= Structure->CounterMode;
  }

  if((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000000U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000400U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000800U)))))
  {
     
    tmpcr1 &= ~(0x3U << (8U));
    tmpcr1 |= (uint32_t)Structure->ClockDivision;
  }

   
  tmpcr1 &= ~(0x1U << (7U));
  tmpcr1 |= (uint32_t)Structure->AutoReloadPreload;

  TIMx->CR1 = tmpcr1;

   
  TIMx->ARR = (uint32_t)Structure->Period ;

   
  TIMx->PSC = (uint32_t)Structure->Prescaler;

  if (((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))))
  {
     
    TIMx->RCR = Structure->RepetitionCounter;
  }

  
 
  TIMx->EGR = (0x1U << (0U));
}






 
static void TIM_OC1_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx = 0U;
  uint32_t tmpccer = 0U;
  uint32_t tmpcr2 = 0U;

    
  TIMx->CCER &= ~(0x1U << (0U));

   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;

   
  tmpccmrx = TIMx->CCMR1;

   
  tmpccmrx &= ~(0x7U << (4U));
  tmpccmrx &= ~(0x3U << (0U));
   
  tmpccmrx |= OC_Config->OCMode;

   
  tmpccer &= ~(0x1U << (1U));
   
  tmpccer |= OC_Config->OCPolarity;

  if((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) && (((0x00000000U) == 0x00000000U) || ((0x00000000U) == 0x00000004U) || ((0x00000000U) == 0x00000008U))))
  {
     
    ((void)0U);

     
    tmpccer &= ~(0x1U << (3U));
     
    tmpccer |= OC_Config->OCNPolarity;
     
    tmpccer &= ~(0x1U << (2U));
  }

  if(((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))))
  {
     
    ((void)0U);
    ((void)0U);

     
    tmpcr2 &= ~(0x1U << (8U));
    tmpcr2 &= ~(0x1U << (9U));
     
    tmpcr2 |= OC_Config->OCIdleState;
     
    tmpcr2 |= OC_Config->OCNIdleState;
  }
   
  TIMx->CR2 = tmpcr2;

   
  TIMx->CCMR1 = tmpccmrx;

   
  TIMx->CCR1 = OC_Config->Pulse;

   
  TIMx->CCER = tmpccer;
}






 
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx = 0U;
  uint32_t tmpccer = 0U;
  uint32_t tmpcr2 = 0U;

   
  TIMx->CCER &= ~(0x1U << (4U));

   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;

   
  tmpccmrx = TIMx->CCMR1;

   
  tmpccmrx &= ~(0x7U << (12U));
  tmpccmrx &= ~(0x3U << (8U));

   
  tmpccmrx |= (OC_Config->OCMode << 8U);

   
  tmpccer &= ~(0x1U << (5U));
   
  tmpccer |= (OC_Config->OCPolarity << 4U);

  if((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) && (((0x00000004U) == 0x00000000U) || ((0x00000004U) == 0x00000004U) || ((0x00000004U) == 0x00000008U))))
  {
    ((void)0U);

     
    tmpccer &= ~(0x1U << (7U));
     
    tmpccer |= (OC_Config->OCNPolarity << 4U);
     
    tmpccer &= ~(0x1U << (6U));

  }

  if(((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))))
  {
     
    ((void)0U);
    ((void)0U);

     
    tmpcr2 &= ~(0x1U << (10U));
    tmpcr2 &= ~(0x1U << (11U));
     
    tmpcr2 |= (OC_Config->OCIdleState << 2);
     
    tmpcr2 |= (OC_Config->OCNIdleState << 2);
  }

   
  TIMx->CR2 = tmpcr2;

   
  TIMx->CCMR1 = tmpccmrx;

   
  TIMx->CCR2 = OC_Config->Pulse;

   
  TIMx->CCER = tmpccer;
}






 
static void TIM_OC3_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx = 0U;
  uint32_t tmpccer = 0U;
  uint32_t tmpcr2 = 0U;

   
  TIMx->CCER &= ~(0x1U << (8U));

   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;

   
  tmpccmrx = TIMx->CCMR2;

   
  tmpccmrx &= ~(0x7U << (4U));
  tmpccmrx &= ~(0x3U << (0U));
   
  tmpccmrx |= OC_Config->OCMode;

   
  tmpccer &= ~(0x1U << (9U));
   
  tmpccer |= (OC_Config->OCPolarity << 8U);

  if((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) && (((0x00000008U) == 0x00000000U) || ((0x00000008U) == 0x00000004U) || ((0x00000008U) == 0x00000008U))))
  {
    ((void)0U);

     
    tmpccer &= ~(0x1U << (11U));
     
    tmpccer |= (OC_Config->OCNPolarity << 8U);
     
    tmpccer &= ~(0x1U << (10U));
  }

  if(((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))))
  {
     
    ((void)0U);
    ((void)0U);

     
    tmpcr2 &= ~(0x1U << (12U));
    tmpcr2 &= ~(0x1U << (13U));
     
    tmpcr2 |= (OC_Config->OCIdleState << 4U);
     
    tmpcr2 |= (OC_Config->OCNIdleState << 4U);
  }

   
  TIMx->CR2 = tmpcr2;

   
  TIMx->CCMR2 = tmpccmrx;

   
  TIMx->CCR3 = OC_Config->Pulse;

   
  TIMx->CCER = tmpccer;
}






 
static void TIM_OC4_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx = 0U;
  uint32_t tmpccer = 0U;
  uint32_t tmpcr2 = 0U;

   
  TIMx->CCER &= ~(0x1U << (12U));

   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;

   
  tmpccmrx = TIMx->CCMR2;

   
  tmpccmrx &= ~(0x7U << (12U));
  tmpccmrx &= ~(0x3U << (8U));

   
  tmpccmrx |= (OC_Config->OCMode << 8U);

   
  tmpccer &= ~(0x1U << (13U));
   
  tmpccer |= (OC_Config->OCPolarity << 12U);

  if(((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))))
  {
    ((void)0U);

    
    tmpcr2 &= ~(0x1U << (14U));
     
    tmpcr2 |= (OC_Config->OCIdleState << 6);
  }

   
  TIMx->CR2 = tmpcr2;

   
  TIMx->CCMR2 = tmpccmrx;

   
  TIMx->CCR4 = OC_Config->Pulse;

   
  TIMx->CCER = tmpccer;
}








 
static void TIM_SlaveTimer_SetConfig(TIM_HandleTypeDef *htim,
                              TIM_SlaveConfigTypeDef * sSlaveConfig)
{
  uint32_t tmpsmcr = 0U;
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  tmpsmcr = htim->Instance->SMCR;

   
  tmpsmcr &= ~(0x7U << (4U));
   
  tmpsmcr |= sSlaveConfig->InputTrigger;

   
  tmpsmcr &= ~(0x7U << (0U));
   
  tmpsmcr |= sSlaveConfig->SlaveMode;

   
  htim->Instance->SMCR = tmpsmcr;

   
  switch (sSlaveConfig->InputTrigger)
  {
  case 0x00000070U:
    {
       
      ((void)0U);
      ((void)0U);
      ((void)0U);
      ((void)0U);
       
      TIM_ETR_SetConfig(htim->Instance,
                        sSlaveConfig->TriggerPrescaler,
                        sSlaveConfig->TriggerPolarity,
                        sSlaveConfig->TriggerFilter);
    }
    break;

  case 0x00000040U:
    {
       
      ((void)0U);
      ((void)0U);

       
      tmpccer = htim->Instance->CCER;
      htim->Instance->CCER &= ~(0x1U << (0U));
      tmpccmr1 = htim->Instance->CCMR1;

       
      tmpccmr1 &= ~(0xFU << (4U));
      tmpccmr1 |= ((sSlaveConfig->TriggerFilter) << 4U);

       
      htim->Instance->CCMR1 = tmpccmr1;
      htim->Instance->CCER = tmpccer;

    }
    break;

  case 0x00000050U:
    {
       
      ((void)0U);
      ((void)0U);
      ((void)0U);

       
      TIM_TI1_ConfigInputStage(htim->Instance,
                               sSlaveConfig->TriggerPolarity,
                               sSlaveConfig->TriggerFilter);
    }
    break;

  case 0x00000060U:
    {
       
      ((void)0U);
      ((void)0U);
      ((void)0U);

       
      TIM_TI2_ConfigInputStage(htim->Instance,
                                sSlaveConfig->TriggerPolarity,
                                sSlaveConfig->TriggerFilter);
    }
    break;

  case 0x00000000U:
    {
       
      ((void)0U);
    }
    break;

  case 0x00000010U:
    {
       
      ((void)0U);
    }
    break;

  case 0x00000020U:
    {
       
      ((void)0U);
    }
    break;

  case 0x00000030U:
    {
       
      ((void)0U);
    }
    break;

  default:
    break;
  }
}



















 
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  TIMx->CCER &= ~(0x1U << (0U));
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;

   
  if((((TIMx) == ((TIM_TypeDef *)((0x40000000U + 0x00010000U) + 0x00002C00U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000000U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000400U))) || ((TIMx) == ((TIM_TypeDef *)(0x40000000U + 0x00000800U)))) != RESET)
  {
    tmpccmr1 &= ~(0x3U << (0U));
    tmpccmr1 |= TIM_ICSelection;
  }
  else
  {
    tmpccmr1 |= (0x1U << (0U));
  }

   
  tmpccmr1 &= ~(0xFU << (4U));
  tmpccmr1 |= ((TIM_ICFilter << 4U) & (0xFU << (4U)));

   
  tmpccer &= ~((0x1U << (1U)) | (0x1U << (3U)));
  tmpccer |= (TIM_ICPolarity & ((0x1U << (1U)) | (0x1U << (3U))));

   
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}











 
static void TIM_TI1_ConfigInputStage(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  tmpccer = TIMx->CCER;
  TIMx->CCER &= ~(0x1U << (0U));
  tmpccmr1 = TIMx->CCMR1;

   
  tmpccmr1 &= ~(0xFU << (4U));
  tmpccmr1 |= (TIM_ICFilter << 4U);

   
  tmpccer &= ~((0x1U << (1U)) | (0x1U << (3U)));
  tmpccer |= TIM_ICPolarity;

   
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}



















 
static void TIM_TI2_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  TIMx->CCER &= ~(0x1U << (4U));
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;

   
  tmpccmr1 &= ~(0x3U << (8U));
  tmpccmr1 |= (TIM_ICSelection << 8U);

   
  tmpccmr1 &= ~(0xFU << (12U));
  tmpccmr1 |= ((TIM_ICFilter << 12U) & (0xFU << (12U)));

   
  tmpccer &= ~((0x1U << (5U)) | (0x1U << (7U)));
  tmpccer |= ((TIM_ICPolarity << 4U) & ((0x1U << (5U)) | (0x1U << (7U))));

   
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}











 
static void TIM_TI2_ConfigInputStage(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr1 = 0U;
  uint32_t tmpccer = 0U;

   
  TIMx->CCER &= ~(0x1U << (4U));
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;

   
  tmpccmr1 &= ~(0xFU << (12U));
  tmpccmr1 |= (TIM_ICFilter << 12U);

   
  tmpccer &= ~((0x1U << (5U)) | (0x1U << (7U)));
  tmpccer |= (TIM_ICPolarity << 4U);

   
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}



















 
static void TIM_TI3_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr2 = 0U;
  uint32_t tmpccer = 0U;

   
  TIMx->CCER &= ~(0x1U << (8U));
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;

   
  tmpccmr2 &= ~(0x3U << (0U));
  tmpccmr2 |= TIM_ICSelection;

   
  tmpccmr2 &= ~(0xFU << (4U));
  tmpccmr2 |= ((TIM_ICFilter << 4U) & (0xFU << (4U)));

   
  tmpccer &= ~((0x1U << (9U)) | (0x1U << (11U)));
  tmpccer |= ((TIM_ICPolarity << 8U) & ((0x1U << (9U)) | (0x1U << (11U))));

   
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}



















 
static void TIM_TI4_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                       uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr2 = 0U;
  uint32_t tmpccer = 0U;

   
  TIMx->CCER &= ~(0x1U << (12U));
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;

   
  tmpccmr2 &= ~(0x3U << (8U));
  tmpccmr2 |= (TIM_ICSelection << 8U);

   
  tmpccmr2 &= ~(0xFU << (12U));
  tmpccmr2 |= ((TIM_ICFilter << 12U) & (0xFU << (12U)));

   
  tmpccer &= ~(0x1U << (13U));
  tmpccer |= ((TIM_ICPolarity << 12U) & (0x1U << (13U)));

   
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer ;
}















 
static void TIM_ITRx_SetConfig(TIM_TypeDef *TIMx, uint16_t InputTriggerSource)
{
  uint32_t tmpsmcr = 0U;

    
   tmpsmcr = TIMx->SMCR;
    
   tmpsmcr &= ~(0x7U << (4U));
    
   tmpsmcr |= InputTriggerSource | 0x00000007U;
    
   TIMx->SMCR = tmpsmcr;
}
















 
static void TIM_ETR_SetConfig(TIM_TypeDef* TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter)
{
  uint32_t tmpsmcr = 0U;

  tmpsmcr = TIMx->SMCR;

   
  tmpsmcr &= ~((0xFU << (8U)) | (0x3U << (12U)) | (0x1U << (14U)) | (0x1U << (15U)));

   
  tmpsmcr |= (uint32_t)(TIM_ExtTRGPrescaler | (TIM_ExtTRGPolarity | (ExtTRGFilter << 8U)));

   
  TIMx->SMCR = tmpsmcr;
}













 
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState)
{
  uint32_t tmp = 0U;

   
  ((void)0U);
  ((void)0U);

  tmp = (0x1U << (0U)) << Channel;

   
  TIMx->CCER &= ~tmp;

   
  TIMx->CCER |=  (uint32_t)(ChannelState << Channel);
}



 




 



 
 
