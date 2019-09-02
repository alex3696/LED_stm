#include "main.h"
#include "depth.h"
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "gpio.h"
#include "iwdg.h"
#include <string.h>
#include "flash_map.h"
#include "crc.h"
#include "stdlib.h"

volatile USHORT   usRegInputBuf[REG_INPUT_NREGS] = {FW_VERSION};
volatile InputReg* gInReg = (InputReg*)&usRegInputBuf;


volatile USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
volatile HoldingReg* gCfg = (HoldingReg*)&usRegHoldingBuf;


static uint8_t gPendingApplyCfg=0;

//-----------------------------------------------------------------------------

void ModbusInit()
{
    eMBErrorCode    eStatus=MB_ENOERR;
    eStatus = eMBInit( MB_RTU, gCfg->mMBAddress, 0, 460800, MB_PAR_NONE );

    assert_param(MB_ENOERR == eStatus);
    // Enable the Modbus Protocol Stack.
    eStatus = eMBEnable(  );
    assert_param(MB_ENOERR == eStatus);
}

//-----------------------------------------------------------------------------
void ModbusEventHandler()
{
    eMBErrorCode err=eMBPoll();
    switch(err)
    {
    case MB_ENOERR: break;
    default:
        //eMBDisable();
        //eMBEnable();
        break;

    }
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress
                           , USHORT usNRegs )
{
    //remember usAddress is 1 based
    #if REG_INPUT_START>0
        if(usAddress < REG_INPUT_START)
        return MB_ENOREG;
    #endif
    --usAddress; // 0 based start address
    usNRegs += usAddress;// 0 based end address

    if( usNRegs > REG_INPUT_START + REG_INPUT_NREGS )
        return MB_ENOREG;

    USHORT* cellBuf = (USHORT*)pucRegBuffer;
    volatile USHORT* reg = usRegInputBuf +usAddress - REG_INPUT_START;

    while( usAddress++ < usNRegs )
    {
        *cellBuf++ = __builtin_bswap16(*reg++);
    }

    return MB_ENOERR;
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress
                             , USHORT usNRegs, eMBRegisterMode eMode )
{
    #if REG_HOLDING_START>0
    if(usAddress < REG_HOLDING_START)
        return MB_ENOREG;
    #endif
    --usAddress; // 0 based start address
    usNRegs += usAddress;// 0 based end address
    if( usNRegs > REG_HOLDING_START + REG_HOLDING_NREGS )
        return MB_ENOREG;
    USHORT* cellBuf = (USHORT*)pucRegBuffer;
    volatile USHORT* reg = usRegHoldingBuf + usAddress - REG_HOLDING_START;
    switch(eMode)
    {
    default: return MB_ENOREG;
    case MB_REG_READ:
        while( usAddress++ < usNRegs )
            *cellBuf++ = __builtin_bswap16(*reg++);
        break;
    case MB_REG_WRITE:
        while( usAddress++ < usNRegs )
            *reg++ = __builtin_bswap16(*cellBuf++);
        gPendingApplyCfg=1;
        break;
    }//switch(eMode)
    return MB_ENOERR;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_60  /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE   /* End @ of user Flash area */

void SaveCfg();
const StorageCfg* LoadCfg();
void SetCfg(const StorageCfg* const );
//-----------------------------------------------------------------------------
void SysTickHandler()
{
    gInReg->mSysTick = HAL_GetTick();
}
//-----------------------------------------------------------------------------
void InitInputReg()
{
    memset((void*)usRegInputBuf, 0, REG_INPUT_NREGS*2 );

    gInReg->mFWVersion = FW_VERSION;
    gInReg->mInternalTemp = -1;
    gInReg->mAdcVref = -1;
    gInReg->mSysTick = 0;
    gInReg->mEnableTime=0;
    gInReg->mDecTime=0;
    gInReg->mPrevTimHandler = 0;
    gInReg->mLedTimeout = 2000;
    gInReg->mActivity=0;
    gInReg->mDimming = DimmingDown;
}
//-----------------------------------------------------------------------------
void InitHoldingReg()
{
    const StorageCfg* cfg = LoadCfg();
    if(cfg)
        SetCfg(cfg);
    else
    {
        memset((void*)usRegHoldingBuf, 0, REG_HOLDING_NREGS*2 );
        // default setup
        gCfg->mMBAddress=1;
        gCfg->mPendingSaveCfg = 0;
        gCfg->mWDTResets = 0;
        gCfg->mMode = Auto;

        gCfg->mBlinkQty=6;
        gCfg->mDimmUp=1;
        gCfg->mDimmDown=1;

        gCfg->mMaxTimeout=128000;
        gCfg->mMinTimeout=2000;
        gCfg->mTimHandler=10;
        // default setup
    }
    gPendingApplyCfg=1;
}
//-----------------------------------------------------------------------------
void ConfigInit()
{
    InitHoldingReg();
    InitInputReg();
}
//-----------------------------------------------------------------------------
void ConfigEventHandler()
{
    if(!gPendingApplyCfg)
        return;
    ENTER_CRITICAL_SECTION();
    gPendingApplyCfg=0;
    if(gCfg->mPendingSaveCfg)
        SaveCfg();

    EXIT_CRITICAL_SECTION( );
}
//-----------------------------------------------------------------------------
void SetCfg(const StorageCfg* const cfg)
{
    *gCfg = cfg->mCfg;
}
//-----------------------------------------------------------------------------
const StorageCfg* LoadCfg()
{
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t curr_cfg_address=0;
    uint32_t max_generation=0;
    const StorageCfg* pcfg;
    while (address < FLASH_USER_END_ADDR)
    {
        pcfg = (const StorageCfg*)address;

        const uint32_t crc_size= sizeof(StorageCfg)/sizeof(uint32_t)-1;
        const uint32_t calculateCRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)address, crc_size) ;

        if(   pcfg->mFW == FW_VERSION
           && pcfg->mGeneration >= max_generation
           && pcfg->mCrc == calculateCRC
           )
        {
            max_generation = pcfg->mGeneration;
            curr_cfg_address=address;
        }
        address += sizeof(StorageCfg);
    }//while

    if(curr_cfg_address)
        return (const StorageCfg*)curr_cfg_address;
    return 0;
}
//-----------------------------------------------------------------------------
uint32_t EraseUserFlash(uint32_t address)
{
    ENTER_CRITICAL_SECTION();
    uint32_t page_no = (address-ADDR_FLASH_PAGE_0)/FLASH_PAGE_SIZE;
    uint32_t curr_address = page_no*FLASH_PAGE_SIZE + ADDR_FLASH_PAGE_0;
    curr_address += FLASH_PAGE_SIZE;
    if(curr_address >= FLASH_USER_END_ADDR)
        curr_address=FLASH_USER_START_ADDR;

    HAL_FLASH_Unlock();
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = curr_address;
    EraseInitStruct.NbPages     = 1;
    assert_param(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) == HAL_OK);

    HAL_FLASH_Lock();
    EXIT_CRITICAL_SECTION();
    return curr_address;
}
//-----------------------------------------------------------------------------
uint32_t FindFreeArea(const uint32_t start_addr)
{
    uint32_t* curr = (uint32_t*)start_addr;
    while((uint32_t)curr < FLASH_USER_END_ADDR)
    {
        //int filled_area = memcmp((const void*)curr,(const void*)empty_buf,sizeof(StorageCfg));
        int filled_area = 0;

        const size_t len = sizeof(StorageCfg)/sizeof(uint32_t);
        for(size_t i=0; i<len; i++)
            if(0xFFFFFFFF != curr[i] )
            {
                filled_area++;
                break;
            }

        if(!filled_area)
            return (uint32_t)curr;
        curr += sizeof(StorageCfg)/sizeof(uint32_t);
    }
    return EraseUserFlash(start_addr);
}
//-----------------------------------------------------------------------------
void SaveCfg()
{
    if( 2 > gCfg->mPendingSaveCfg)
        return;
    gCfg->mPendingSaveCfg=0;

    const StorageCfg* old_cfg = LoadCfg();
    StorageCfg cfg;
    memset(&cfg,0x00,sizeof(StorageCfg));
    cfg.mFW = FW_VERSION;

    uint32_t free_area=0;

    if(old_cfg)
    {
        if( 0 == memcmp( old_cfg, (void*)usRegHoldingBuf, REG_HOLDING_NREGS*2) )
            return;
        if(UINT32_MAX==old_cfg->mGeneration)
        {
            cfg.mGeneration=0;
            uint32_t curr = FLASH_USER_START_ADDR;
            while(curr < FLASH_USER_END_ADDR)
            {
                free_area = EraseUserFlash(curr);
                curr += FLASH_PAGE_SIZE;
            }
        }//if(-1==cfg.mGeneration)
        else
        {
            cfg.mGeneration = old_cfg->mGeneration+1;
            free_area = FindFreeArea((uint32_t)old_cfg);
        }
    }//if(old_cfg)
    else
    {
        free_area = FindFreeArea(FLASH_USER_START_ADDR);
    }

    cfg.mCfg = *gCfg;

    const uint32_t crc_size= sizeof(StorageCfg)/sizeof(uint32_t)-1;
    cfg.mCrc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&cfg, crc_size);

    ENTER_CRITICAL_SECTION();
    HAL_FLASH_Unlock();

    int err=0;
    size_t cfg_qty = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR)/sizeof(StorageCfg);
    for(uint32_t i=0;i<cfg_qty; i++ )
    {
        uint32_t address=free_area;
        uint32_t n= sizeof(StorageCfg)/sizeof(uint32_t);
        for(uint32_t i=0;i<n; i++ )
        {
            err += HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, ((int32_t*)&cfg)[i]);
            address += sizeof(uint32_t);
        }
        const uint32_t check_frame =HAL_CRC_Calculate(&hcrc, (uint32_t*)free_area, crc_size);
        if (!err && check_frame==cfg.mCrc)
            break;
        else
        {
            err = 1;
            free_area = FindFreeArea(address);;
        }

    }

    assert_param(0==err);

    HAL_FLASH_Lock();
    EXIT_CRITICAL_SECTION();
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void WDTCheck()
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        gCfg->mWDTResets++;
    }
}
//-----------------------------------------------------------------------------
void WDTEventHandler()
{
    assert_param(HAL_OK==HAL_IWDG_Refresh(&hiwdg));
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ClrActivity()
{
    gInReg->mActivity=0;
    dbg_printf("%lu ClrActivity\n", gInReg->mSysTick);
}
//-----------------------------------------------------------------------------
void IncActivity()
{
    if(INT16_MAX < gInReg->mActivity)
        return;
    // �� �������������� ��������� ���������� �� 3/4 ������� ���������
    if(DimmingUp==gInReg->mDimming
       && ((htim2.Init.Period*3)/4 > htim2.Instance->CCR2) )
        return;
    gInReg->mActivity++;
    dbg_printf("%lu IncActivity %d\n", gInReg->mSysTick, gInReg->mActivity);
}

//-----------------------------------------------------------------------------
void IncrementTimeout()
{
    if(gCfg->mMaxTimeout <= gInReg->mLedTimeout)
        return;
    gInReg->mLedTimeout*=2;
    //if(gInReg->mLedTimeout>gCfg->mMaxTimeout)
    //    gInReg->mLedTimeout=gCfg->mMaxTimeout;
    dbg_printf("%lu IncTimeout %lu\n",gInReg->mSysTick, gInReg->mLedTimeout);
}
//-----------------------------------------------------------------------------
void DecrementTimeout()
{
    if(gInReg->mLedTimeout<= gCfg->mMinTimeout)
        return;

    if(gInReg->mSysTick - gInReg->mDecTime < gInReg->mLedTimeout )
        return;

    gInReg->mDecTime = gInReg->mSysTick;
    gInReg->mLedTimeout/=2;
    if(gInReg->mLedTimeout<gCfg->mMinTimeout)
        gInReg->mLedTimeout=gCfg->mMinTimeout;
    dbg_printf("%lu DecTimeout %lu\n",gInReg->mSysTick, gInReg->mLedTimeout);
}
//-----------------------------------------------------------------------------
void EnableLed()
{
    if(Off==gCfg->mMode)
        return;

    gInReg->mEnableTime = gInReg->mSysTick;
    INTERNAL_LED_ON;
    gInReg->mDimming = DimmingUp;
    dbg_printf("%lu EnableLED Time=%lu Timeout=%lu \n"
               ,gInReg->mSysTick, gInReg->mEnableTime, gInReg->mLedTimeout);
}
//-----------------------------------------------------------------------------
void DisableLed()
{
    if(On==gCfg->mMode || DimmingDown==gInReg->mDimming)
        return;
    INTERNAL_LED_OFF;
    gInReg->mDimming = DimmingDown;
    dbg_printf("%lu DisableLED \n"
               ,gInReg->mSysTick);
}
//-----------------------------------------------------------------------------
void ModeHandler()
{
    gCfg->mMode++;
    if(Auto < gCfg->mMode)
        gCfg->mMode=Off;

    switch(gCfg->mMode)
    {
        default: //case On:  EnableLed();
        case Auto:
            EnableLed();
        break;
        case Off:
            gInReg->mLedTimeout = gCfg->mMinTimeout;
            DisableLed();
            htim2.Instance->CCR2 = 1;
        break;
    }
}
//-----------------------------------------------------------------------------
void EventHandler(SensorEvent evt)
{
    switch(evt)
    {
    default: case sensPIR_OFF: case sensSOUND_OFF: case sensDISTANCE_OFF: break;

    case sensDISTANCE_ON:
        ModeHandler();
        break;
    case sensSOUND_ON:
        if(htim2.Instance->CCR2)
            IncActivity();
        break;
    case sensPIR_ON:
        if(htim2.Instance->CCR2)
            IncActivity();
        else
            ClrActivity();
        EnableLed();
        break;
    }//switch(evt)
}
//-----------------------------------------------------------------------------
static uint16_t blinkOffQty=0;
//-----------------------------------------------------------------------------
void StepDimmUp()
{
    if(htim2.Instance->CCR2< htim2.Init.Period)
    {
        htim2.Instance->CCR2 += gCfg->mDimmUp;
        if(htim2.Instance->CCR2 > htim2.Init.Period)
            htim2.Instance->CCR2=htim2.Init.Period;
    }
}
//-----------------------------------------------------------------------------
void StepDimmDown()
{
    if( 0 < htim2.Instance->CCR2 ) //__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2) )
    {
        if(htim2.Instance->CCR2 >= gCfg->mDimmDown)
            htim2.Instance->CCR2 -= gCfg->mDimmDown;
        else
        {
            htim2.Instance->CCR2 = 0;
            blinkOffQty=0;
        }
    }
}
//-----------------------------------------------------------------------------
void Timer_Handler()
{
    if(gInReg->mSysTick-gInReg->mPrevTimHandler < gCfg->mTimHandler)
        return;

    if(DimmingDown==gInReg->mDimming)
    {
        DecrementTimeout();
        if( 0 == htim2.Instance->CCR2)
            return;
        if(!blinkOffQty)
        {
            StepDimmDown();
            return;
        }


        if(blinkOffQty % 2)
        {
            StepDimmDown();
            if(htim2.Init.Period*1/10 > htim2.Instance->CCR2)
                blinkOffQty--;
        }
        else
        {
            StepDimmUp();
            if(htim2.Init.Period*4/10 < htim2.Instance->CCR2)
                blinkOffQty--;

        }

    }
    else
    {
        if( gInReg->mSysTick - gInReg->mEnableTime > gInReg->mLedTimeout )
        {
            if(gInReg->mActivity)
            {
                const uint16_t need_act = (gInReg->mLedTimeout/gCfg->mMinTimeout);
                if( need_act <= gInReg->mActivity )
                    IncrementTimeout();
                EnableLed();
            }
            else
            {
                blinkOffQty=gCfg->mBlinkQty;
                DisableLed();
            }
            ClrActivity();
        }

        StepDimmUp();
    }//if(DimmingDown==gInReg->mDimming)
    gInReg->mPrevTimHandler=gInReg->mSysTick;

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static uint32_t startPIR;
static uint32_t startSOUND;
static uint32_t startDISTANCE;
//-----------------------------------------------------------------------------
void PIR_Handler()
{
    const GPIO_PinState pir_state = HAL_GPIO_ReadPin (PIR_GPIO_Port, PIR_Pin);
    if( GPIO_PIN_SET == pir_state)
    {
        if(startPIR)
        {
            if(gInReg->mSysTick - startPIR > 200)
            {
                //INTERNAL_LED_ON;
                EventHandler(sensPIR_ON);
            }
        }
        else
            startPIR = gInReg->mSysTick;
    }
    else
    {
        startPIR = 0;
        //INTERNAL_LED_OFF;
        EventHandler(sensPIR_OFF);
    }


}
//-----------------------------------------------------------------------------
void Sound_Handler()
{
    const GPIO_PinState sound_state = HAL_GPIO_ReadPin (SOUND_GPIO_Port, SOUND_Pin);
    if( GPIO_PIN_RESET == sound_state)
    {
        if(startSOUND)
        {
            //INTERNAL_LED_ON;
            EventHandler(sensSOUND_ON);
        }
        else
            startSOUND = gInReg->mSysTick;
    }
    else
    {
        startSOUND = 0;
        //INTERNAL_LED_OFF;
        EventHandler(sensSOUND_OFF);
    }
}
//-----------------------------------------------------------------------------
BOOL ON_sent=FALSE;
BOOL OFF_sent=FALSE;
void Distance_Handler()
{
    const GPIO_PinState distance_state = HAL_GPIO_ReadPin (DISTANCE_GPIO_Port, DISTANCE_Pin);
    if( GPIO_PIN_RESET == distance_state)
    {
        if(startDISTANCE)
        {
            if(gInReg->mSysTick - startDISTANCE > 1000)
            {
                if(!ON_sent)
                {
                    EventHandler(sensDISTANCE_ON);
                    ON_sent=TRUE;
                    OFF_sent=FALSE;
                }
            }
        }
        else
            startDISTANCE = gInReg->mSysTick;
    }
    else
    {
        startDISTANCE = 0;
        //INTERNAL_LED_OFF;
        //HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        if(!OFF_sent)
        {
            EventHandler(sensDISTANCE_OFF);
            ON_sent=FALSE;
            OFF_sent=TRUE;
        }
    }
}
