/**
  ******************************************************************************
  * @file           : abd.c
  * @author         : iclm team
  * @brief          : algorithm for area body detect
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "global_conf.h"
#include "platform.h"
#include "abd.h"

#include "utilities.h"

static uint16_t targetRangeBuf[RANGE_BUF_SIZE] = {0};
static DfftPeakData_t dfftPeakData = {0};
static uint8_t targetOffBuf[TAR_OFF_BUF_SIZE];
const ABD_PARA_T abdPara __ALIGN(4) =
{
    ROI_MAX,
    0,
    VALID_RANGE,
    EXIT_THD,
    HOLD_TICKS
};

const uint16_t sensitivityMap[SENSITIVITY_LEVEL_MAX] = 
{
    0,
    SENSITIVITY_LEVEL_1_THD,
    SENSITIVITY_LEVEL_2_THD,
    SENSITIVITY_LEVEL_3_THD
};

const uint16_t MovingTargetTh[MOV_TAR_TH_NUM] = 
{
#ifdef FFT_ZERO_FILL
    100,/* idx 0 */
    100,/* idx 1 */
    100,/* idx 2 */
    100,/* idx 3 */
    100,/* idx 4 */
    100,/* idx 5 */
    100,/* idx 6 */
    100,/* idx 7 */
    100,/* idx 8 */
    100,/* idx 9 */
    100,/* idx 10 */
    100,/* idx 11 */
    100,/* idx 12 */
    100,/* idx 13 */
    100,/* idx 14 */
    0,  /* idx 15 */
    0,  /* idx 16 */
    0,  /* idx 17 */
    0,  /* idx 18 */
    0,  /* idx 19 */
    0,  /* idx 20 */
    0,  /* idx 21 */
    0,  /* idx 22 */
    0,  /* idx 23 */
    0,  /* idx 24 */
    0,  /* idx 25 */
    0,  /* idx 26 */
    0,  /* idx 27 */
    0,  /* idx 28 */
    0,  /* idx 29 */
    0,  /* idx 30 */
    0   /* idx 31 */    
#else
    2000,/* idx 0 */
    2000,/* idx 1 */
    2000,/* idx 2 */
    1500,/* idx 3 */
    1300,/* idx 4 */
    1200,/* idx 5 */
    1200,/* idx 6 */
    1600,/* idx 7 */
    1600,/* idx 8 */
    1600,/* idx 9 */
    1300,/* idx 10 */
    500,/* idx 11 */
    400,/* idx 12 */
    350,/* idx 13 */
    350,/* idx 14 */
    730,/* idx 15 */ 
    730,/* idx 16 */ 
    730,/* idx 17 */ 
    730,/* idx 18 */ 
    730,/* idx 19 */ 
    730,/* idx 20 */ 
    730,/* idx 21 */ 
    730,/* idx 22 */ 
    730,/* idx 23 */ 
    730,/* idx 24 */ 
    730,/* idx 25 */ 
    730,/* idx 26 */ 
    730,/* idx 27 */ 
    730,/* idx 28 */ 
    730,/* idx 29 */ 
    730,/* idx 30 */ 
    730 /* idx 31 */  
#endif
 };


static void GetDfftPeakData(uint8_t *data_buf, DfftPeakData_t *pData)
{
   for(uint16_t i = 0; i < DFFT_PEAK_OUTPUT_NUM; i++)
   {
       pData->v_idx[i] = data_buf[12 * i + 0]; 
       pData->r_idx[i] = data_buf[12 * i + 1];
       pData->mag[i] = (uint32_t)((data_buf[12 * i + 4] << 24) | 
                                  (data_buf[12 * i + 5] << 16) |
                                  (data_buf[12 * i + 6] << 8) |
                                  (data_buf[12 * i + 7]));
   }
}

static TargetStatus_t GetTargetStatus(DfftPeakData_t *pData)
{      
    if(pData->mag[0] > MovingTargetTh[pData->r_idx[0]]) /* 能量值很高，很有可能是运动目标 */
    {
        return TAR_MOVING;
    }
	
    /* 能量值不高，但是有目标，此时目标可能处于静止状态 */
	#ifdef PY32_PLATFORM
    if(HAL_GPIO_ReadPin(TSPI_IO_PORT, TSPI_IO_PIN))
	#else
    if(GPIO_ReadInputDataBit(TSPI_IO_PORT, TSPI_IO_PIN))
    #endif
    {
        return TAR_STOP;
    }
    /* SOC无法检测到目标，判断为无人 */
    return TAR_OFF;
 }

void PrintTargetInfo(DfftPeakData_t *pData)
{   
    static uint8_t print_cnt = 0;
    
    if(print_cnt++ % 5)
    {
        return;
    } 
    
    for(uint8_t i = 0; i < DFFT_PEAK_OUTPUT_NUM; i++)
    {
        printf("idx:%u r:%u v:%u val:%u\r\n",
                i, pData->r_idx[i], pData->v_idx[i], pData->mag[i] );
    }  
 }

uint16_t GetTargetRange(DfftPeakData_t *pData)
{      
#ifdef CENTER_GRAVITY
    int32_t left_level = 0;
    int32_t right_level = 0;
    int32_t level = 0;
  
    //uint8_t v_idx = 0;
    uint8_t  r_idx = 0;
    uint8_t idx_tmp = 0;
     
    uint8_t flag_tmp = 0;
    float   distance = 0;
    float   d = 0;
    
    //v_idx = pData->v_idx[0];//速度索引
    r_idx = pData->r_idx[0];//距离索引
    level = (int32_t)(pData->mag[0]);//能量值
    
	
	for(uint8_t i = 1; i < DFFT_PEAK_OUTPUT_NUM; i++)
    {
        idx_tmp = pData->r_idx[i];    
        if( idx_tmp == r_idx - 1 )//取左侧距离门的能量值
        { 
           flag_tmp |= 0x01;
           left_level = (int32_t)(pData->mag[i]);//能量值  
           break;
        }
    }
		
	for(uint8_t i = 1; i < DFFT_PEAK_OUTPUT_NUM; i++)
    {
        idx_tmp = pData->r_idx[i];
     
        if(idx_tmp == r_idx + 1)//取右侧距离门的能量值
        { 
           flag_tmp |= 0x02;
           right_level = (int32_t)(pData->mag[i]);//能量值  
           break;
        }
    }
    
    if((flag_tmp & 0x01) == 0x00)
    {
        left_level = 0;
    }
    
    if((flag_tmp & 0x02) == 0x00)
    {
        right_level = 0;
    }
    
    d =     (float)(right_level - left_level) 
          / (float)(2 * (2 * level - left_level - right_level));
    distance = (((float)r_idx + d) * (float)RANGE_RESOLUTION);
    	
    return distance;
#else
    return ((pData->r_idx[0]) * RANGE_RESOLUTION );
#endif
}

static uint8_t IsConditonMeet(uint8_t *contentBuf, uint16_t bufSize, uint16_t condition)
{
    uint16_t i = 0;
    uint16_t sum = 0;
    
    if (NULL == contentBuf)
    {
        return 0;
    }

    for (i = 0; i < bufSize; i++)
    {
        if (contentBuf[i])
        {
            sum++;
        }
    }

    if (sum >= condition)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

#define OUTRANGE_MAX_CNT	100

uint8_t outrange_cnt = 0;

void ABD_DetectByPeakData(uint8_t* dataBuf, uint16_t dataLen)
{
    static uint8_t k = 0;
    static uint8_t pos = 0;
    static uint16_t lastRange = 0;
    static uint8_t tOffIdx = 0;
    uint8_t tarOff = 0;
    uint16_t thisRange = 0;
    TargetStatus_t targetStatus = TAR_OFF;
    
    GetDfftPeakData(dataBuf, &dfftPeakData);
#if 1
    PrintTargetInfo(&dfftPeakData);    
//    return;
#endif

    targetStatus = GetTargetStatus(&dfftPeakData);
	
    if (targetStatus == TAR_OFF)
    {
        targetOffBuf[tOffIdx] = 1;
    }
    else
    {
        targetOffBuf[tOffIdx] = 0;
    }
    tOffIdx = (++tOffIdx) % TAR_OFF_BUF_SIZE;

    if(TAR_MOVING == targetStatus)
    {
        memset(targetOffBuf, 0, sizeof(targetOffBuf));
    }
    
    tarOff = IsConditonMeet(targetOffBuf, TAR_OFF_BUF_SIZE, TAR_OFF_TH);
		
    if (tarOff)
    {	 
		IoOutOff();
        printf("---OFF---\r\n");
        k = 0;
        pos = 0;
        lastRange = 0;
        memset(targetRangeBuf, 0, sizeof(targetRangeBuf));

        return; 
    }
    
    if(TAR_MOVING == targetStatus)
    {
        targetRangeBuf[k] = GetTargetRange(&dfftPeakData);
        k = (++k) % RANGE_BUF_SIZE;
                       
        pos = (pos >= RANGE_BUF_SIZE ? RANGE_BUF_SIZE : ++pos);
			
        for(uint8_t i = 0; i < pos; i++)
        {
            thisRange += targetRangeBuf[i];
        }  
        thisRange /= pos;
        
        lastRange = thisRange;     
    }
    else
    {
        thisRange = lastRange ; 
    }

	if(thisRange > abdPara.validRange)
	{
		if(outrange_cnt <= OUTRANGE_MAX_CNT)
		{
			outrange_cnt ++;
		}
	}
	else
	{
		outrange_cnt = 0;
	}
    if (thisRange == 0)
    {				
        printf("---ON---\r\n");			
    }
	else if(outrange_cnt > OUTRANGE_MAX_CNT)
	{
		IoOutOff();
		printf("---OFF---\r\n");
	}
    else
    {
		if(thisRange > abdPara.roiMax)
		{
			thisRange = abdPara.roiMax;
		}		
		IoOutOn();//Jackie
		printf("---ON---(R = %d)\r\n", thisRange);
    }
 }

void ABD_Init(void)
{
    IoOutInit();
    IoOutOff();
    TspiIoInit();
}

void ABD_PrintStatus(void)
{
    static uint16_t tickCnt = 0;
    bool status = 0;
	
	#ifdef PY32_PLATFORM
    status = HAL_GPIO_ReadPin(TSPI_IO_PORT, TSPI_IO_PIN);
	#else
    status = GPIO_ReadInputDataBit(TSPI_IO_PORT, TSPI_IO_PIN);
    #endif

    if (status)
    {
        IoOutOn();
    }
    else
    {
        IoOutOff();
    }

    tickCnt++;
    if (tickCnt % ABD_CHECK_INTERVAL == 0)
    {
        if (status)
        {
            printf("ON\r\n");
        }
        else
        {
            printf("OFF\r\n");
        }
    }
}


