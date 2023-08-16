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
 Demonstrates how to jump from user mode (=APP) to bootloader mode (=IAP):
   - Upload IAP via debug interface to System Flash (addr=0x1FFFF000) using WCH-LinkUtility (GUI, Win) or https://github.com/ch32-rs/wlink (cmd, Cross-OS) 
   - Switching between IAP and APP mode is by a special key in flash 
   - Connect PD0 to a LED. IAP mode is indicated by a fast LED blink, and APP by a slow LED blink
   - To switch from IAP to APP, remove external pull-down on pin PC0 (configured as input pull-up)
   - To switch from APP (=application) to IAP (=bootloader), manually set pin PC0=LOW (configured as input pull-up)
   - Due to unknown reasons, sometimes a power-on reset is required after a mode switch
   - In IAP mode, a new APP software can be uploaded via USART1 using WCHISPTool (GUI, Win) or WCHISPTool_CMD (cmd, Cross-OS)
*/

#include "debug.h"

// SysTick ISR
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// 1ms SysTick counter
volatile uint32_t    countMs = 0;



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

    // initialize GPIOC.0 as input pull-up (switch to bootloader = IAP)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   ISR for 1ms SysTick
 *
 * @return  none
 */
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    countMs++;
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
    uint64_t    timeLastLED = 0;
    
    GPIO_INIT();
    USART_Printf_Init(115200);

    // init systic timer for 1ms
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = SystemCoreClock/1000L-1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;

    printf("\n\nSystemClk: %ldMHz\r\n", (SystemCoreClock / 1000000L));

    while(1)
    {
        // indicate APP mode via slow toggling PD0 (connect to LED)
        if (countMs - timeLastLED > 2000)
        {
            timeLastLED = countMs;
            GPIO_WriteBit(GPIOD, GPIO_Pin_0, !GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0));
            printf("time = %ds\r\n", (int) (countMs / 1000L));
        }
        
        // If PC2=LOW -> switch to IAP bootloader mode and trigger SW reset
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 0) {
            printf("Go to IAP...\r\n");
            Delay_Ms(50);
            GoToIAP();
            while(1);
        }
    }

}
