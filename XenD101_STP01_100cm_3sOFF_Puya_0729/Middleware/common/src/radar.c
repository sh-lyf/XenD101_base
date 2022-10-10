/**
  ******************************************************************************
  * @file           : radar.c
  * @author         : iclm team
  * @brief          : radar driver
  ******************************************************************************
  */
#include <stdio.h>
#include "platform.h"
#include "radar.h"
#include "config.h"
#include "radar_para.h"
#include "utilities.h"

uint8_t g_ChannelCount = CHANNEL_MAX;
uint8_t g_maxChannelCount = CHANNEL_MAX;
uint8_t g_TxCount = 1;

static const uint16_t rawPointMap[RAW_MAP_NUM] = 
{
	RAW_POINT_64,
    RAW_POINT_128,
    RAW_POINT_256,
    RAW_POINT_512,
    RAW_POINT_1024
};

#ifndef XenD101_1815_ULP_BOARD
static const RADAR_REG_T RegListPD[] =
{
    {0x70, 0x1020},
    {0x6C, 0x8880},
    {0x6D, 0x8800},
    {0x72, 0x0650},
    {0x67, 0x0000},
    {0x66, 0xF0F0},
    {0x6E, 0x03FC},    
    {0x41, 0xC864},	 
    {0x00, 0x0000} /*must be last, do not delete!!!*/
};
#endif

const RADAR_REG_T RegListNormal[] =
{
    {0x41, 0xC874}, 
    {0x72, 0x0613},
    {0x6C, 0x9990},
    {0x6D, 0x99C0},
    {0x70, 0x2EA0},
    {0x6E, 0x8BFC},
    {0x66, 0xF8F0},
    {0x67, 0x1040},  
    {0x69, 0x0006},
    {0x00, 0x0000} /*must be last, do not delete!!!*/
};

#ifdef SUPPORT_DFFT_PEAK_CONFIG
const RADAR_REG_T RegListDfftPeak[DFFT_PEAK_REG_NUM] ={
	{
		RADAR_2DFFT_PEAK_RANGE_IDX,
		(MAX_RANGE_IDX << 8) | MIN_RANGE_IDX,
	},
	{
		RADAR_2DFFT_PEAK_VELOCITY_IDX,
		(MAX_VELOCITY_IDX << 8) | MIN_VELOCITY_IDX,
	},
	{
		RADAR_2DFFT_PEAK_TAR_FRAME_NUM,
		(ACTIVE_FRAME_NUM << 8) | INACTIVE_FRAME_NUM,
	},
	
	{
		RANGE_THRESHOLD_BASE + 0,
		RANGR_THRESHOLD_0,
	},

	{
		RANGE_THRESHOLD_BASE + 1,
		RANGR_THRESHOLD_1,
	},

	{
		RANGE_THRESHOLD_BASE + 2,
		RANGR_THRESHOLD_2,
	},
	
	{
		RANGE_THRESHOLD_BASE + 3,
		RANGR_THRESHOLD_3,
	},

	{
		RANGE_THRESHOLD_BASE + 4,
		RANGR_THRESHOLD_4,
	},

	{
		RANGE_THRESHOLD_BASE + 5,
		RANGR_THRESHOLD_5,
	},

	{
		RANGE_THRESHOLD_BASE + 6,
		RANGR_THRESHOLD_6,
	},

	{
		RANGE_THRESHOLD_BASE + 7,
		RANGR_THRESHOLD_7,
	},

	{
		RANGE_THRESHOLD_BASE + 8,
		RANGR_THRESHOLD_8,
	},
	{
		RANGE_THRESHOLD_BASE + 9,
		RANGR_THRESHOLD_9,
	},	

	{
		RANGE_THRESHOLD_BASE + 10,
		RANGR_THRESHOLD_10,
	},
	{
		RANGE_THRESHOLD_BASE + 11,
		RANGR_THRESHOLD_11,
	},	
	{
		RANGE_THRESHOLD_BASE + 12,
		RANGR_THRESHOLD_12,
	},
	{
		RANGE_THRESHOLD_BASE + 13,
		RANGR_THRESHOLD_13,
	},	
	{
		RANGE_THRESHOLD_BASE + 14,
		RANGR_THRESHOLD_14,
	},
	{
		RANGE_THRESHOLD_BASE + 15,
		RANGR_THRESHOLD_15,
	},	
	{
		0,
		0,
	},
};


#endif




uint16_t Radar_GetOneFrameChirpNum(void)
{
	uint16_t val = 0;
    I2C_Read(I2C_ADDR_RADAR_Chip0, RADAR_PAT_CHIRP_NUM, &val);
	return val;
}

uint16_t Radar_GetDfftDataNum(void)
{
    uint16_t val = 0;
    
    I2C_Read(I2C_ADDR_RADAR_Chip0, RADAR_DIG_DFFT_DATA_NUM, &val);

    return (val >> RADAR_DFFT_DATA_NUM_POS);
}

uint16_t Radar_GetDfftPeakSize(void)
{
    uint16_t val = 0;
    
    I2C_Read(I2C_ADDR_RADAR_Chip0, RADAR_DIG_RAW_PEAK_NUM, &val);

    return ((val & RADAR_PEAK_MASK) * 4); /*4--word length*/
}

uint16_t Radar_GetDfftChirpNum(void)
{
    uint16_t val = 0;
    
    I2C_Read(I2C_ADDR_RADAR_Chip0, RADAR_DIG_DFFT_CHIRP_NUM, &val);

    return (val >> RADAR_DFFT_CHIRP_NUM_POS);
}

uint8_t Radar_GetDataType(void)
{
    uint8_t dataType = 0;
    uint16_t val = 0;
    
    I2C_Read(I2C_ADDR_RADAR_Chip0, RADAR_DIG_FUN_SWITCH, &val);

    if (val & RADAR_DFFT_DATA)
    {
        dataType = DATA_TYPE_DFFT;
    }
    else if (val & RADAR_FFT_DATA)
    {
        dataType = DATA_TYPE_FFT;
    }
    else if (val & RADAR_DFFT_PEAK_DATA)
    {
        dataType = DATA_TYPE_DFFT_PEAK;
    }
	else if (val & RADAR_DSRAW_DATA)
    {
        dataType = DATA_TYPE_DSRAW;
    }
    else
    {
        dataType = DATA_TYPE_MAX;
    } 

    return dataType;
}


#ifdef SUPPORT_DFFT_PEAK_CONFIG
void ConfigDfftPeakPara(void)
{
    uint16_t loop = 0;
  
    while(RegListDfftPeak[loop].addr) 
    {
        I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(RegListDfftPeak[loop].addr), RegListDfftPeak[loop].val);
        loop++;
    }
}


#endif

#ifdef CONFIG_DEBUG
static void DumpFlashRegValue(void)
{
    uint16_t loop = 0;
    
    printf("radar flash value:\r\n");
    while(InitRegList[loop].addr) 
    {
        printf("%02X=%04X\r\n", InitRegList[loop].addr, InitRegList[loop].val);
        loop++;
    } 
}

static void DumpChipRegValue(void)
{
    uint16_t loop = 0;
    uint16_t val = 0;
    
    printf("radar ic value:\r\n");
    while(InitRegList[loop].addr) 
    {
        I2C_Read(I2C_ADDR_RADAR_Chip0, InitRegList[loop].addr, &val);
        printf("%02X=%04X\r\n", InitRegList[loop].addr, val);
        loop++;
    } 

#ifdef SUPPORT_DFFT_PEAK_CONFIG
    loop = 0;
    while(RegListDfftPeak[loop].addr) 
    {
        I2C_Read(I2C_ADDR_RADAR_Chip0, RegListDfftPeak[loop].addr, &val);
        printf("%02X=%04X\r\n", RegListDfftPeak[loop].addr, val);
        loop++;
    } 
#endif
}
#endif




void ReplaceSpecialRegs(void)
{
    uint16_t loopPd = 0;
    uint16_t loopInit = 0;

	while (InitRegList[loopInit].addr) 
    {
		if(loopInit < REG_FIX_NUM)
		{
			I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(InitRegList[loopInit].addr), InitRegList[loopInit].val);
		}

        else
		{
			loopPd = 0;
			while (RegListPD[loopPd].addr)
			{
				if (InitRegList[loopInit].addr == RegListPD[loopPd].addr)
				{
					I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(RegListPD[loopPd].addr), RegListPD[loopPd].val);
					break;
				}
				loopPd ++;
			}
			if(RegListPD[loopPd].addr == 0)
			{
				I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(InitRegList[loopInit].addr), InitRegList[loopInit].val);
			}
		}
        loopInit++;
    } 
}




void Radar_Init(void)
{
    uint16_t loop = 0;

    ReplaceSpecialRegs();

    DumpFlashRegValue();

    Delay(RADAR_PD2NORMAL_DELAY);
    Radar_EnterNormalMode();

    ConfigDfftPeakPara();


    DumpChipRegValue();

}

void Radar_PreInit(void)
{
    uint16_t loop = 0;

    while (InitRegList[loop].addr) 
    {
        I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(InitRegList[loop].addr), InitRegList[loop].val);
        loop++;
    }   
}


void* Radar_GetRadarParaAddr(void)
{
    return (void*)&InitRegList;
}

uint32_t Radar_GetRadarParaLen(void)
{
    return sizeof(InitRegList);
}

//#ifdef SUPPORT_LOW_POWER
void Radar_EnterPDMode(void)
{
#ifdef XenD101_1815_ULP_BOARD
    GPIO_PowerCtlOff();
#else
    uint16_t loop = 0;
    
    while(RegListPD[loop].addr) 
    {
        I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(RegListPD[loop].addr), RegListPD[loop].val);
        loop++;
    }
#endif
    TIM3_Enable();
}
//#endif

void Radar_EnterNormalMode(void)
{
#ifdef XenD101_1815_ULP_BOARD
    GPIO_PowerCtlOn();
#else
    uint16_t loop = 0;
    
    while(RegListNormal[loop].addr) 
    {
        I2C_Write(I2C_ADDR_RADAR_Chip0, (uint8_t)(RegListNormal[loop].addr), RegListNormal[loop].val);
        loop++;
	}
#endif
}


