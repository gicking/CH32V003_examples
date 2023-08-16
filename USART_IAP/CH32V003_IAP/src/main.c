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
 Support flash upload via serial interface, and demonstrates how to jump from bootloader mode (=IAP) to user mode (=APP):
   - Upload IAP via debug interface to System Flash (addr=0x1FFFF000) using WCH-LinkUtility (GUI, Win) or https://github.com/ch32-rs/wlink (cmd, Cross-OS) 
   - Switching between IAP and APP mode is by a special key in flash 
   - Connect PD0 to a LED. IAP mode is indicated by a fast LED blink, and APP by a slow LED blink
   - To switch from IAP to APP, remove external pull-down on pin PC0 (configured as input pull-up)
   - To switch from APP (=application) to IAP (=bootloader), manually set pin PC0=LOW (configured as input pull-up)
   - Due to unknown reasons, sometimes a power-on reset is required after a mode switch
   - In IAP mode, a new APP software can be uploaded via USART1 using WCHISPTool (GUI, Win) or WCHISPTool_CMD (cmd, Cross-OS)
*/

#include "debug.h"
#include "string.h"
#include "iap.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    volatile uint32_t i=0;
    
    // Enable peripheral clocks for USART1, GPIOC and GPIOD
    RCC->APB2PCENR |= RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;
    
    // init USART1 to 460800Baud
    GPIOD->CFGLR  &= ~0x0FF00000;       // set PD5&6 default pin settings
    GPIOD->CFGLR  |= 0x08B00000;        // set PD6=Rx, PD5=Tx
    GPIOD->BCR     = GPIO_Pin_6;        // activate PD6 pull-up
    //USART1->CTLR2 |= USART_StopBits_1;                // 1 stop bit is reset setting 
    USART1->CTLR1  = 0x000C;            // set USART mode, length, parity, GPIO pin
    //USART1->CTLR3 |= USART_HardwareFlowControl_None;  // no flow control is reset setting
    USART1->BRR    = 0x34;              // set 460800 baudrate
    USART1->CTLR1 |= 0x2000;            // enables USART

    // configure GPIOC.0 as input with pull-up
    GPIOC->CFGLR &= ~0x0F;              // set PC0 default pin settings
    GPIOC->CFGLR |=  0x08;              // input with PU/PD
    GPIOC->BSHR   =  0x01;              // activate pull-up

    // configure GPIOD.0 (connected to LED) as output, push-pull
    GPIOD->CFGLR &= ~0x0F;              // set PD0 default pin settings               
    GPIOD->CFGLR |=  0x02;              // set output, push-pull


    // main loop
    while(1)
    {
        // indicate IAP mode by fast LED blink (connect PD0 to LED)
        if (i++ == 100000L)
        {
            i = 0;
            GPIOD->OUTDR ^= GPIO_Pin_0;
        }

        // return to APP mode if bootmode pin is high (=default)
        if (GPIOC->INDR & GPIO_Pin_0)
        {
            //RCC_ClearFlag();                            // clear reset cause (not required)
            SystemReset_StartMode(Start_Mode_USER);
            NVIC_SystemReset();
        }

        // execute UART commands
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        {
            UART_Rx_Deal();
        }

    } // main loop

} // main()
