/**
  ******************************************************************************
  * @file           : dataprocess.c
  * @author         : iclm team
  * @brief          : data process module
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "global_conf.h"
#include "utilities.h"
#include "dataprocess.h"
#include "radar.h"
#include "cmdprocess.h"
#include "system.h"
#include "abd.h"


static RADAR_DATA_PARSE_T RadarDataParse[CHANNEL_MAX];
RADAR_PARA_T RadarPara;


static void DataStateIdParse(uint8_t data, uint8_t channel)
{
    uint8_t flag = 0;
    
    switch (RadarPara.dataType)
    {     
        case DATA_TYPE_DFFT_PEAK:
            if ((data & ID_MASK) == DFFT_PEAK_ID)
            {
                flag = 1;
			}
            break;
        
        default:
            break;
    }

    if (flag)
    {
        RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
        RadarDataParse[channel].state = DATA_STATE_INDEX1;
    }
    else
    {
        RadarDataParse[channel].state = DATA_STATE_HEAD;
    }
}

static void DataStateIndex1Parse(uint8_t data, uint8_t channel)
{
    switch (RadarPara.dataType)
    {   
        case DATA_TYPE_DFFT_PEAK:
            break;
        
        default:
            break;
    }

    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    RadarDataParse[channel].state = DATA_STATE_INDEX2;
}

static void DataStateIndex2Parse(uint8_t data, uint8_t channel)
{
    switch (RadarPara.dataType)
    {
        
        case DATA_TYPE_DFFT_PEAK:
            break;
        
        default:
            break;
    }
    
    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    RadarDataParse[channel].state = DATA_STATE_DATA;
}

static uint8_t DataStateTail3Parse(uint8_t data, uint8_t channel)
{
    RadarDataParse[channel].state = DATA_STATE_HEAD;
    if (data != DATA_TAIL) 
    {
        return 0;
    }
    
    switch (RadarPara.dataType)
    {
        
        case DATA_TYPE_DFFT_PEAK:
            break;
        
        default:
            break;
    }
    
    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    
    return 1;
}

static void DataCopy(uint8_t* buf, uint16_t len, uint8_t channel, uint16_t *i)
{
    uint16_t copyLen = 0;

    if (NULL == buf || NULL == i)
    {
        return;
    }
    
	copyLen = (RadarDataParse[channel].needCopyLen > (len - *i))?
            (len - *i) : RadarDataParse[channel].needCopyLen;
    memcpy(&RadarDataParse[channel].buf[RadarDataParse[channel].curIndex], &buf[*i], copyLen);

    RadarDataParse[channel].curIndex += copyLen;
    *i += (copyLen - 1);
    RadarDataParse[channel].needCopyLen -= copyLen;

    if (!RadarDataParse[channel].needCopyLen)
    {
    	RadarDataParse[channel].state = DATA_STATE_TAIL0;
        RadarDataParse[channel].needCopyLen = RadarPara.dataLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
    }
}

static uint8_t DataParse(uint8_t* buf, uint16_t len, uint8_t channel)
{
    uint16_t i = 0;
    uint8_t parseFinish = 0;

    if (NULL == buf || 0 == len || channel >= CHANNEL_MAX)
    {
        return 0;
    }

	for (i = 0; i < len; i++) 
    {
		switch(RadarDataParse[channel].state)
		{                    
			case DATA_STATE_HEAD:
				if (buf[i] == DATA_HEAD) 
                {
                    RadarDataParse[channel].curIndex = 0;
                    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
					RadarDataParse[channel].state = DATA_STATE_ID;
				}
				break;
                
			case DATA_STATE_ID:
                DataStateIdParse(buf[i], channel);
				break;
                
			case DATA_STATE_INDEX1:
                DataStateIndex1Parse(buf[i], channel);
				break;
                
			case DATA_STATE_INDEX2:
                DataStateIndex2Parse(buf[i], channel);
				break;
            
			case DATA_STATE_DATA:				
                DataCopy(buf, len, channel, &i);
				break;

            case DATA_STATE_TAIL0:
                RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
				RadarDataParse[channel].state = DATA_STATE_TAIL1;
				break;

            case DATA_STATE_TAIL1:
                RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
				RadarDataParse[channel].state = DATA_STATE_TAIL2;
				break;

            case DATA_STATE_TAIL2:
                RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
				RadarDataParse[channel].state = DATA_STATE_TAIL3;
				break;
                
   		    case DATA_STATE_TAIL3:
                parseFinish = DataStateTail3Parse(buf[i], channel);
				break;
			
			default:
				RadarDataParse[channel].state = DATA_STATE_HEAD;
				break;

		}
	}

    return parseFinish;
}


void StartAlgorithm(uint8_t* dataBuf, uint16_t dataLen, uint8_t channel, uint16_t index)
{
  
#if defined(SUPPORT_RV_OUTPUT)
    ABD_DetectByPeakData(dataBuf, dataLen);
#elif defined(SUPPORT_ABD)
    ABD_PrintStatus();
#endif
}

static void DataDispatch(uint8_t* frameBuf, uint16_t bufLen, uint8_t channel, uint16_t index)
{
    uint8_t* dataBuf = NULL;
    uint16_t dataLen = 0;

    if (NULL == frameBuf || 0 == bufLen)
    {
        return;
    }
	

	dataBuf = frameBuf + SPI_FRAME_HLEN;
	dataLen = bufLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
	StartAlgorithm(dataBuf, dataLen, channel, index);

}

static void DataProcess(uint8_t channel, uint8_t dmaFlag, uint8_t *recvBuf, uint16_t bufLen)
{
    uint8_t parseFinish = 0;
    uint16_t index = 0;
    uint16_t threshold = INDICATOR_RECV_THRESHOLD;

#if defined(MM32_PLATFORM) || defined(PY32_PLATFORM) 
    IWDG_Reload();
#endif
    if (channel >= CHANNEL_MAX || dmaFlag >= DMA_RECV_FLAG_MAX || NULL == recvBuf)
    {
        printf("Error para!\r\n");
        return;
    }

    parseFinish = DataParse(recvBuf, bufLen, channel);
    g_dataRecvFlag[channel][dmaFlag] = 0;
    if (!parseFinish)
    {
        return;
    }

    switch (RadarPara.dataType)
    {        
        case DATA_TYPE_DFFT_PEAK:
            threshold >>= INDICATOR_RECV_THD_DPEAK_SHIFT;
            break;
        
        default:
            break;
    }
    
    Indicator_RadarDataReceived(threshold);
#ifdef SUPPORT_LOW_POWER
    Radar_EnterPDMode();
#endif
    DataDispatch(RadarDataParse[channel].buf, RadarPara.dataLen, channel, index);
}

static int8_t GetRadarPara(RADAR_PARA_T *radarPara)
{

    if (NULL == radarPara)
    {
        return FAIL;
    }

    radarPara->dataType = Radar_GetDataType();
#if defined(CONFIG_DEBUG)
    printf("dataType: %d\r\n", radarPara->dataType);
#endif
    switch (radarPara->dataType)
    {
        
        
        case DATA_TYPE_DFFT_PEAK:
            radarPara->dataLen = Radar_GetDfftPeakSize() + SPI_FRAME_HLEN + SPI_FRAME_TLEN;
            break;
        
        default:
            printf("Error: unsupport dataType\r\n");
            return FAIL;
    }

#if defined(CONFIG_DEBUG)
    printf("dataLen: %d\r\n", radarPara->dataLen);
#endif
    if (radarPara->dataLen > SPI_FRAME_LEN_MAX)
    {
        printf("Error: dataLen is too long\r\n");
        return FAIL;
    }

    radarPara->chirpNum = Radar_GetOneFrameChirpNum();
#if defined(CONFIG_DEBUG)
    printf("chirpNum: %d\r\n", radarPara->chirpNum);
#endif

    return OK;
}

void DataProc_Init(void)
{
    uint8_t channel = 0;
    
    memset(&RadarDataParse, 0 ,sizeof(RadarDataParse));
    memset(&RadarPara, 0 ,sizeof(RadarPara));

    if (FAIL == GetRadarPara(&RadarPara))
    {
//        RunFailed((uint8_t *)__FILE__, __LINE__);
    }

    for (channel = 0; channel < CHANNEL_MAX; channel++)
    {
        RadarDataParse[channel].needCopyLen = RadarPara.dataLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
    }


    SPI_Init(RadarPara.dataLen * 2); /*radar data received by spi dma, ping-pang buffer*/
}



uint8_t DataProc_GetRadarDataType(void)
{    
    return RadarPara.dataType;
}

uint16_t DataProc_GetRadarDataLen(void)
{    
    return RadarPara.dataLen;
}


void DataProc_Recv(void)
{
    uint8_t channel = 0;
    uint8_t dmaFlag = 0;
    uint16_t dataPos = 0;

    for (dmaFlag = 0; dmaFlag < DMA_RECV_FLAG_MAX; dmaFlag++)
    {
        for (channel = 0; channel < g_ChannelCount; channel++)
        {
            while (!g_dataRecvFlag[channel][dmaFlag]);

            if (DMA_RECV_FLAG_MEM_0 == dmaFlag)
            {
                dataPos = 0;
            }
            else
            {
                dataPos = RadarPara.dataLen;
            }
            
            DataProcess(channel, dmaFlag, &g_dataRecvBuf[channel][dataPos], RadarPara.dataLen);
        }
    }
}




