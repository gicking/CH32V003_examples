/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/11/21
 * Description        : IAP
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "iap.h"
#include "string.h"
#include "flash.h"
#include "core_riscv.h"

/******************************************************************************/

#define FLASH_Base   0x08000000
#define USBD_DATA_SIZE               64
u32 Program_addr = FLASH_Base;
u32 Verity_addr = FLASH_Base;
u8 Verity_Star_flag = 0;
u8 Fast_Program_Buf[128];
u16 CodeLen = 0;
u8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE];
#define  isp_cmd_t   ((isp_cmd  *)EP2_Rx_Buffer)

/*********************************************************************
 * @fn      UART1_SendData
 *
 * @brief   USART1 send 1B data
 *
 * @param   pbuf - Packet to be sent
 *          num - Number of data sent
 *
 * @return  none
 */
void UART1_SendData(u8 data)
{
    while(!(USART1->STATR & USART_FLAG_TC));
    USART1->DATAR = (uint16_t) data;
}

/*********************************************************************
 * @fn      UART1_ReceiveData
 *
 * @brief   Uart1 receive 1B data
 *
 * @return  none
 */
u8 UART1_ReceiveData(void)
{
    while(!(USART1->STATR & USART_FLAG_RXNE));
    return (uint8_t) USART1->DATAR;
}

/*********************************************************************
 * @fn      RecData_Deal
 *
 * @brief   UART-USB
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SCUESS - SCUESS
 *          ERR_End - End
 */
u8 RecData_Deal(void)
{
    u8 i, s, Length;

    Length = isp_cmd_t->Len;

    switch ( isp_cmd_t->Cmd) {

        case CMD_IAP_ERASE:
            FLASH_Unlock_Fast();
            FLASH_EraseAllPages();
            s = ERR_SCUESS;
            break;

        case CMD_IAP_PROM:
            for (i = 0; i < Length; i++) {
                Fast_Program_Buf[CodeLen + i] = isp_cmd_t->data[i];
            }
            CodeLen += Length;
            if (CodeLen >= 64) {


                CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
                CodeLen -= 64;
                for (i = 0; i < CodeLen; i++) {
                    Fast_Program_Buf[i] = Fast_Program_Buf[64 + i];
                }

                Program_addr += 0x40;

            }
            s = ERR_SCUESS;
            break;

        case CMD_IAP_VERIFY:

            if (Verity_Star_flag == 0) {
                Verity_Star_flag = 1;

                for (i = 0; i < (64 - CodeLen); i++) {
                    Fast_Program_Buf[CodeLen + i] = 0xFF;
                }

                CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
                CodeLen = 0;
            }

            s = ERR_SCUESS;
            for (i = 0; i < Length; i++) {
                if (isp_cmd_t->data[i] != *(u8*) (Verity_addr + i)) {
                    s = ERR_ERROR;
                    break;
                }
            }

            Verity_addr += Length;

            break;

        case CMD_IAP_END:
            Verity_Star_flag = 0;
            End_Flag = 1;
            Program_addr = FLASH_Base;
            Verity_addr = FLASH_Base;

            s = ERR_End;
            break;

        default:
            s = ERR_ERROR;
            break;
    }

    return s;
}

/*********************************************************************
 * @fn      UART_Rx_Deal
 *
 * @brief   UART Rx data deal
 *
 * @return  none
 */
void UART_Rx_Deal(void)
{
    u8 i, s;
    u8 Data_add = 0;

    if (UART1_ReceiveData() == Uart_Sync_Head1)
    {
        if (UART1_ReceiveData() == Uart_Sync_Head2)
        {
            isp_cmd_t->Cmd = UART1_ReceiveData();
            Data_add += isp_cmd_t->Cmd;
            isp_cmd_t->Len = UART1_ReceiveData();
            Data_add += isp_cmd_t->Len;
            isp_cmd_t->Rev[0] = UART1_ReceiveData();
            Data_add += isp_cmd_t->Rev[0];
            isp_cmd_t->Rev[1] = UART1_ReceiveData();
            Data_add += isp_cmd_t->Rev[1];

            if ((isp_cmd_t->Cmd == CMD_IAP_PROM) || (isp_cmd_t->Cmd == CMD_IAP_VERIFY))
            {
                for (i = 0; i < isp_cmd_t->Len; i++) {
                    isp_cmd_t->data[i] = UART1_ReceiveData();
                    Data_add += isp_cmd_t->data[i];
                }
            }

            if (UART1_ReceiveData() == Data_add)
            {

                s = RecData_Deal();

                if (s != ERR_End)
                {
                    UART1_SendData(0x00);
                    if (s == ERR_ERROR)
                    {
                        UART1_SendData(0x01);
                    }
                    else
                    {
                        UART1_SendData(0x00);
                    }
                }
            }
        }
    }
}

