#ifndef __depth_H
#define __depth_H

#include "mb.h"

#define FW_VERSION   1
//-----------------------------------------------------------------------------
#define REG_HOLDING_START   0
#define REG_HOLDING_NREGS   20
extern volatile USHORT   usRegHoldingStart;
extern volatile USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

#define REG_INPUT_START 0
#define REG_INPUT_NREGS 20
extern volatile USHORT   usRegInputStart;
extern volatile USHORT   usRegInputBuf[REG_INPUT_NREGS];
#define gFWwVersion         usRegInputBuf[0]
#define gInternalTemp       usRegInputBuf[1]
#define gAdcVref            usRegInputBuf[2]
#define gSysTick            (*((uint32_t*)(usRegInputBuf+3)))
#define gPIR                usRegInputBuf[5]
#define gDISTANCE           usRegInputBuf[6]
#define gSOUND              usRegInputBuf[7]
#define gLedEnableTime      (*((uint32_t*)(usRegInputBuf+8)))
#define gDimming            usRegInputBuf[10]
#define gPrevTimHandler     (*((uint32_t*)(usRegInputBuf+11)))

//-----------------------------------------------------------------------------
typedef enum{
  Off=0
 ,Auto
 ,On
} Mode;
//-----------------------------------------------------------------------------
typedef enum{
  sensPIR_ON=0
 ,sensPIR_OFF
 ,sensSOUND_ON
 ,sensSOUND_OFF
 ,sensDISTANCE_ON
 ,sensDISTANCE_OFF
} SensorEvent;
//-----------------------------------------------------------------------------
typedef enum{
  DimmingDown=0
 ,DimmingUp
} Dimming;
//-----------------------------------------------------------------------------
typedef struct
{
    uint16_t mPendingSaveCfg;
    uint16_t mWDTResets;
    uint16_t mMode;
    uint32_t mLedTimeout;
    uint16_t mStoredVal_4;
} __attribute__((aligned(1),packed)) Cfg;
//-----------------------------------------------------------------------------
typedef struct
{
    Cfg mCfg;

    uint8_t mReserved[ 128 -12 - REG_HOLDING_NREGS * 2 ];

    uint32_t mFW;
    uint32_t mGeneration;
    uint32_t mCrc;

} __attribute__((aligned(1),packed)) StorageCfg;
//-----------------------------------------------------------------------------
extern void ModbusInit();
extern void ModbusEventHandler();

extern void ConfigInit();
extern void ConfigEventHandler();

extern void SysTickHandler();
void WDTCheck();
void WDTEventHandler();

void PIR_Handler();
void Sound_Handler();
void Distance_Handler();
void Timer_Handler();


#define INTERNAL_LED_ON     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)
#define INTERNAL_LED_OFF    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET)
#define INTERNAL_LED_TOGGLE HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)



#define VDD_APPLI       ((uint32_t) 3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS    ((uint32_t) 4095)   /* Max value with a full range of 12 bits */

#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */                                                               /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

#define COMPUTATION_TEMPERATURE_STD_PARAMS(TS_ADC_DATA)                        \
  ((((int32_t)(INTERNAL_TEMPSENSOR_V25 - (((TS_ADC_DATA) * VDD_APPLI) / RANGE_12BITS)   \
     ) * 1000                                                                  \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )

#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)                        \
  ( (ADC_DATA) * VDD_APPLI / RANGE_12BITS)




#endif /* __***_H */
