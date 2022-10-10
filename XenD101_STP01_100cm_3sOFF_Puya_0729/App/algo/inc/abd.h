/**
  ******************************************************************************
  * @file           : abd.h
  * @author         : iclm team
  * @brief          : abd head file
  ******************************************************************************
  */

#ifndef __ABD_H__
#define __ABD_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>
typedef enum  
{
	RANGE_GEAR_1_5_M = 1,
    RANGE_GEAR_1_0_M,
    RANGE_GEAR_0_5_M
}rangeGearEnum;

typedef enum  
{
	SENSITIVITY_LEVEL_1 = 1,
    SENSITIVITY_LEVEL_2,
    SENSITIVITY_LEVEL_3,
    SENSITIVITY_LEVEL_MAX
}sensitivityEnum;

typedef enum  
{
	ABD_RANGE_GEAR = 0,
    ABD_SENSITIVITY_LEVEL,
    ABD_LAG_TIME
}abdParaEnum;

typedef struct ABD_PARA
{
    uint8_t roiMax;
    uint8_t pad;
    uint16_t validRange;
    uint16_t exitThd;
    uint16_t holdTicks;
}ABD_PARA_T;

#ifdef FFT_ZERO_FILL
#define DFFT_PEAK_OUTPUT_NUM            (1)
#define RANGE_RESOLUTION                (10.0f)
#else
#define DFFT_PEAK_OUTPUT_NUM            (9)
#define RANGE_RESOLUTION                (70.3f)
#endif

#define RANGE_BUF_SIZE                  (10)
#define TAR_OFF_BUF_SIZE                (100)
#define TAR_OFF_TH                      (90)
#define MOV_TAR_TH_NUM                  (32)

#define IGNORE_RANGE_DIFF               (4)
#define IGNORE_NUM                      (3)

#define FRAME_ACC_NUM                   (5)//连续采5帧，peak累计
#define V_STATIC                        (16)//
#define HOLD_TICKS                      (50)

#ifdef XenD101_HPS01_1G
#define ENTER_THD                       (10000)
#define SLIDE_WINDOW_SIZE               (6)

#define ROI_MAX_1_5_M                   (10)
#define VALID_RANGE_1_5_M               (155)

#define ROI_MAX_1_0_M                   (7)
#define VALID_RANGE_1_0_M               (105)

#define ROI_MAX_0_6_M                   (4)
#define VALID_RANGE_0_6_M               (60)
#else
#define ENTER_THD                       (10000)
#define SLIDE_WINDOW_SIZE               (10)

#define ROI_MAX_1_5_M                   (150)
#define VALID_RANGE_1_5_M               (160)

#define ROI_MAX_1_4_M                   (140)
#define VALID_RANGE_1_4_M               (150)

#define ROI_MAX_1_3_M                   (130)
#define VALID_RANGE_1_3_M               (140)

#define ROI_MAX_1_2_M                   (120)
#define VALID_RANGE_1_2_M               (130)

#define ROI_MAX_1_1_M                   (110)
#define VALID_RANGE_1_1_M               (120)

#define ROI_MAX_1_0_M                   (100)
#define VALID_RANGE_1_0_M               (110)


#define ROI_MAX_0_5_M                   (50)
#define VALID_RANGE_0_5_M               (55)
#endif

/***---1.5m---***/
#define ROI_MAX                     ROI_MAX_1_4_M        //ROI_MAX_0_5_M     
#define VALID_RANGE                 VALID_RANGE_1_4_M    //VALID_RANGE_0_5_M 

#define SENSITIVITY_LEVEL_1_THD         (100)
#define SENSITIVITY_LEVEL_2_THD         (25)
#define SENSITIVITY_LEVEL_3_THD         (10)

#define EXIT_THD                        SENSITIVITY_LEVEL_2_THD
#define FRAME_FREQ                      (10) //1000/100

typedef enum
{
    TAR_MOVING = 0,
    TAR_STOP = 1,
    TAR_OFF = 2
}TargetStatus_t;
    
typedef  struct
{
    uint8_t r_idx[DFFT_PEAK_OUTPUT_NUM];
    uint8_t v_idx[DFFT_PEAK_OUTPUT_NUM];
    uint32_t mag [DFFT_PEAK_OUTPUT_NUM];
}DfftPeakData_t;

void ABD_DetectByPeakData(uint8_t* dataBuf, uint16_t dataLen);
void ABD_Init(void);
void ABD_PrintStatus(void);
int8_t ABD_ParaUpdate(uint16_t type, int32_t val);


#ifdef __cplusplus
}
#endif

#endif


