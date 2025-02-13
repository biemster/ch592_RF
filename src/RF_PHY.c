/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#include "HAL.h"
#include "CONFIG.h"
#include "RF_PHY.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

// #define RX_MODE
#define TX_MODE
#define TX_INTERVAL 1000 // * 0.625 ms

uint8_t taskID;
// uint8_t TX_DATA[] = {1, 2, 3, 42, 3, 2 ,1};
uint8_t TX_DATA[] = {0x66, 0x55, 0x44, 0x33, 0x22, 0xd1, // MAC (reversed)
                    0x1e, 0xff, 0x4c, 0x00, 0x12, 0x19, 0x00, // Apple FindMy stuff
                    0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xef, 0xfe, 0xdd,0xcc, // key
                    0xbb, 0xaa, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, // more key
                    0x00, 0x00}; // status byte and one more
rfConfig_t rf_Config;
uint8_t pktTxType = 0x02;

volatile uint8_t tx_end_flag = 0;

__HIGH_CODE
__attribute__((noinline))
void RF_Wait_Tx_End()
{
    uint32_t i=0;
    while(!tx_end_flag)
    {
        i++;
        __nop();
        __nop();
        // Ô¼5ms³¬Ê±
        if(i>(FREQ_SYS/1000))
        {
            tx_end_flag = TRUE;
        }
    }
}

void RF_2G4StatusCallBack(uint8_t sta, uint8_t crc, uint8_t *rxBuf)
{
    switch(sta)
    {
        case TX_MODE_TX_FINISH:
        case TX_MODE_TX_FAIL:
        {
            tx_end_flag = TRUE;
            break;
        }
        case TX_MODE_RX_DATA:
        {
            break;
        }
        case TX_MODE_RX_TIMEOUT: // Timeout is about 200us
        {
            break;
        }
        case RX_MODE_RX_DATA:
        {
            if (crc == 0) {
                uint8_t i;

                PRINT("rx recv, rssi: %d\r\n", (int8_t)rxBuf[0]);
                PRINT("(len=%d):", rxBuf[1]);
                
                for (i = 0; i < rxBuf[1]; i++) {
                    PRINT(" %x", rxBuf[i + 2]);
                }
                PRINT("\r\n");
                if(tmos_memcmp(rxBuf +2, TX_DATA, sizeof(TX_DATA))) {
                    GPIOA_InverseBits(GPIO_Pin_8);
                }
            } else {
                if (crc & (1<<0)) {
                    PRINT("crc error\r\n");
                }

                if (crc & (1<<1)) {
                    PRINT("match type error\r\n");
                }
            }
            tmos_set_event(taskID, SBP_RF_RF_RX_EVT);
            break;
        }
        case RX_MODE_TX_FINISH:
        {
            break;
        }
        case RX_MODE_TX_FAIL:
        {
            break;
        }
    }
}

uint16_t RF_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(task_id)) != NULL)
        {
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    if(events & SBP_RF_START_DEVICE_EVT)
    {
        tmos_start_task(taskID, SBP_RF_PERIODIC_EVT, TX_INTERVAL);
        return events ^ SBP_RF_START_DEVICE_EVT;
    }
    if(events & SBP_RF_PERIODIC_EVT)
    {
        uint16_t ret = 0;
        RF_Config(&rf_Config);
        RF_Shut();
        tx_end_flag = FALSE;

        if(!RF_Tx(TX_DATA, sizeof(TX_DATA), pktTxType, 0xFF))
        {
            RF_Wait_Tx_End();
        }

        if(rf_Config.Channel == 37 || rf_Config.Channel == 38) {
            rf_Config.Channel++;
            ret = events;
        }
        else {
            HAL_Init();
            GPIOA_ResetBits(GPIO_Pin_8);
            CH59x_LowPower(MS_TO_RTC(30));
            GPIOA_SetBits(GPIO_Pin_8);
            tmos_start_task(taskID, SBP_RF_PERIODIC_EVT, TX_INTERVAL);
            rf_Config.Channel = 37;
            ret = events ^ SBP_RF_PERIODIC_EVT;
        }
        return ret;
    }
    if(events & SBP_RF_RF_RX_EVT)
    {
        uint8_t state;
        RF_Shut();
        state = RF_Rx(TX_DATA, sizeof(TX_DATA), 0xFF, 0xFF);
        PRINT("RX mode.state = %x\r\n", state);
        return events ^ SBP_RF_RF_RX_EVT;
    }
    return 0;
}

void RF_Init(void)
{
    uint8_t    state;

    tmos_memset(&rf_Config, 0, sizeof(rfConfig_t));
    taskID = TMOS_ProcessEventRegister(RF_ProcessEvent);
    rf_Config.accessAddress = 0x8E89BED6;
    rf_Config.CRCInit = 0x555555;
    rf_Config.Channel = 37;
    rf_Config.LLEMode = LLE_MODE_BASIC;
    rf_Config.rfStatusCB = RF_2G4StatusCallBack;
    rf_Config.RxMaxlen = 251;
    state = RF_Config(&rf_Config);
    PRINT("rf 2.4g init: %x\r\n", state);

#ifdef RX_MODE
    { // RX mode
        state = RF_Rx(TX_DATA, sizeof(TX_DATA), 0xFF, 0xFF);
        PRINT("RX mode.state = %x\r\n", state);
    }
#endif
#ifdef TX_MODE
    { // TX mode
        tmos_set_event( taskID , SBP_RF_START_DEVICE_EVT );
    }
#endif
}
