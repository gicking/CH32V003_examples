/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 APP go to IAP routine:
 Demonstrates how to jump from userland to IAP.
 Note: The IAP program is solidified in the chip,
  you can refer to this routine to jump to the IAP to upgrade.
*/

#include "debug.h"

/*********************************************************************
 * @fn      GoToIAP
 *
 * @brief   Go to bootloader = IAP 
 *
 * @return  none
 */
void GoToIAP(void)
{
    RCC_ClearFlag();
    SystemReset_StartMode(Start_Mode_BOOT);
    NVIC_SystemReset();
}

/*********************************************************************
 * @fn      GPIO_INIT
 *
 * @brief   Initializes GPIO pins
 *
 * @return  none
 */
void GPIO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Initializes GPIOD.0 as push-pull output (connect to LED)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // initialize GPIOC.1 as input pull-up (switch to bootloader = IAP)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

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
    Delay_Init();
    GPIO_INIT();
    USART_Printf_Init(115200);
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    while(1)
    {
        // indicate APP mode via slow blinking PD0 (connect to LED)
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))
        {
            Delay_Ms(750);
            GPIO_WriteBit(GPIOD, GPIO_Pin_0, !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0));
        }
        
        // switch to IAP bootloader mode and trigger reset
        else
        {
            printf("Go to IAP...\r\n");
            Delay_Ms(10);
            GoToIAP();
            while(1);
        }
    }

}
