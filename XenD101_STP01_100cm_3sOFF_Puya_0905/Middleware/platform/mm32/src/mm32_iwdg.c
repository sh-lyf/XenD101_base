/**
  ******************************************************************************
  * @file    mm32_iwdg.c
  * @author  ting.gao@iclegend.com
  * @brief   iwdg services
  ******************************************************************************
  */
#include "mm32_iwdg.h"

static void Write_Iwdg_ON(u16 IWDG_Prescaler, u16 Reload)
{
    //Start the internal low-speed clock and wait for the clock to be ready
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    //Setting Clock Pre-division Frequency
    PVU_CheckStatus();
    IWDG_WriteAccessCmd(0x5555);
    IWDG_SetPrescaler(IWDG_Prescaler);

    //Setting overload register values
    RVU_CheckStatus();
    IWDG_WriteAccessCmd(0x5555);
    IWDG_SetReload(Reload & 0xfff);

    //Loading and Enabling Counter
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void IWDG_Reload(void)
{
    IWDG_ReloadCounter();
}

void IWDG_Init(void)
{
    Write_Iwdg_ON(IWDG_Prescaler_32, IWDG_RELOAD_VALUE); 
}

