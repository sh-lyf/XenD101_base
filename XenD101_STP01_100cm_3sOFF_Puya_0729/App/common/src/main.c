/**
  ******************************************************************************
  * @file           : main.c
  * @author         : iclm team
  * @brief          : main program for radar dev
  ******************************************************************************
  */
#include <stdio.h>
#include "global_conf.h"
#include "platform.h"
#include "radar.h"
#include "dataprocess.h"
#include "cmdprocess.h"
#include "config.h"
#ifdef STM32_PLATFORM
#include "rtos.h"
#include "cmsis_os.h"
#endif
#ifdef SUPPORT_ABD
#include "abd.h"
#endif

int main(void)
{   
    Platform_Init();

    Radar_PreInit();

    Radar_Init();

    ABD_Init();

    DataProc_Init(); 

    while (1)
    {
        DataProc_Recv();   
    }
}

