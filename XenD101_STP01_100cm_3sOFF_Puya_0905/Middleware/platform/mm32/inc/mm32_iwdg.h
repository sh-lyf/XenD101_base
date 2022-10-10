/**
  ******************************************************************************
  * @file    mm32_iwdg.h
  * @author  ting.gao@iclegend.com
  * @brief   iwdg header file
  ******************************************************************************
  */

#ifndef __MM32_IWDG_H__
#define __MM32_IWDG_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#include "hal_conf.h"

#define IWDG_RELOAD_VALUE      (0x7ffU)   //reset time is about 1.6s.

void IWDG_Init(void);
void IWDG_Reload(void);

#ifdef __cplusplus
}
#endif

#endif

