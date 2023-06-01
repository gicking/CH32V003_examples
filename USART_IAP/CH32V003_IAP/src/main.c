/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/11/21
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
IAP upgrade routine:
Support serial port for FLASH burning

1. Use the IAP download tool to realize the download PA0 floating (default pull-up input)
2. After downloading the APP, connect PC0 to ground (low level input), and press the
reset button to run the APP program.
3. use WCH-LinkUtility.exe or https://github.com/ch32-rs/wlink to download to BOOT(addr=0x1FFFF000)

*/

#include "debug.h"
#include "string.h"
#include "iap.h"

/*********************************************************************
 * @fn      IAP_2_APP
 *
 * @brief   IAP_2_APP program.
 *
 * @return  none
 */
void IAP_2_APP(void)
{
    RCC_ClearFlag();
    SystemReset_StartMode(Start_Mode_USER);
    NVIC_SystemReset();
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    u32 i=0;
    
    // Enable GPIOD, USART1 and GPIOC clock
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD| RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOC;
    
    // init required peripherals
    USART1_CFG(460800);    
    GPIO_CFG();


    // check bootmode pin only after reset
    if(Bootmode_Check() == 0)
    {
        IAP_2_APP();
        while(1);
    }

    // enter IAP bootloader
    while(1)
    {
        // indicate IAP mode via fast blinking PD0 (connect to LED)
        if (i++ == 1000000L)
        {
            i=0;
            GPIOD->OUTDR^=0x1;
        }

        // handle UART commands
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {
            UART_Rx_Deal();
        }
    }
}
