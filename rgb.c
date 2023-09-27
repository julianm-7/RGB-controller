/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

// Include Files
#include "LcdDrivermsp430/Crystalfontz128x128_ST7735.h"
#include "LcdDrivermsp430/HAL_MSP_EXP430FR5994_Crystalfontz128x128_ST7735.h"
#include "grlib.h"
#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>
#define timerperiod 66
#define YHIGH 2553
#define YLOW 1529
#define time 5000
uint8_t period=timerperiod;
Graphics_Context g_sContext;
uint16_t JoyStickX, JoyStickY;
volatile uint8_t PBS2,PBS1;
typedef enum {RED,GREEN,BLUE,TEAL,YELLOW,CITRUS,PURPLE,OFF} LedColors;
LedColors ledState(uint16_t);
//Function Headers
void LCD_init(void);
void ADC_init(void);
void joyStick_init();
void config_mkII(void);
void rgbDriver(uint16_t);
void initTimerB0(uint8_t RED_DutyCycle,uint8_t GREEN_DutyCycle,uint8_t BLUE_DutyCycle, uint8_t BUZZER_DutyCycle);
void TIMER_B_configureUpMode(void);
void TIMER_B_initCompareRegister_6(uint8_t);
void TIMER_B_initCompareRegister_5(uint8_t);
void TIMER_B_initCompareRegister_4(uint8_t);
void TIMER_B_initCompareRegister_3(uint8_t);
void RainbowSequence(void);
void main (void)
{
    LedColors ledColor;
    char buffer[100];
    WDT_A_hold(WDT_A_BASE);
    joyStick_init();
    config_mkII();
    ADC_init();
    LCD_init();
    ADC12_B_startConversion(ADC12_B_BASE,ADC12_B_START_AT_ADC12MEM0,ADC12_B_REPEATED_SEQOFCHANNELS);
    enum {on,off} onOffState;
    enum {on2,off2} nextBuzzerState;
    nextBuzzerState=off2;
    onOffState=off;
    GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3);

    while(1)
    {
        PBS1=GPIO_getInterruptStatus(GPIO_PORT_P4,GPIO_PIN3);
        if (PBS1==GPIO_PIN3)
        {
            if(onOffState==off) onOffState=on;
            else onOffState=off;
            GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3);
        }
        if (onOffState==off)
        {
            JoyStickY=0xffff;
            ledColor = ledState(JoyStickY);
//            sprintf(buffer,"Rainbow Sequence is on");
//            Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            RainbowSequence();
        }
        else
        {
            ADC12_B_startConversion(ADC12_B_BASE,ADC12_B_START_AT_ADC12MEM0,ADC12_B_SEQOFCHANNELS);
            while(ADC12_B_getInterruptStatus(ADC12_B_BASE,0,ADC12_B_IFG1) != ADC12_B_IFG1);
            ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);
            JoyStickY = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
            ledColor = ledState(JoyStickY);
            sprintf(buffer,"JoyStick Y is %c",ledColor);
            Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);
            rgbDriver(ledColor);
            if (nextBuzzerState==on2)
            {
                nextBuzzerState=off2;
            }
            else
            {
                Timer_B_setCompareValue (TIMER_B0_BASE,TIMER_B_CAPTURECOMPARE_REGISTER_6,0);
                nextBuzzerState=on2;
            }
            __delay_cycles(100000);
        }

    }
}

void RainbowSequence(void)
{
    uint8_t RED_DutyCycle;
    uint8_t GREEN_DutyCycle;
    uint8_t BLUE_DutyCycle;
    uint8_t BUZZER_DutyCycle;
    int i;
    for (i = 0; i <= 100 ; ++i) // a = 0 to 1 step 0.01
    {
        const double a = i / 100.0;
        RED_DutyCycle= period;
        GREEN_DutyCycle= (uint8_t) (a*(double)period);
        BLUE_DutyCycle= 0;
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);
    }
    for (i = 10; i <= 100 ; ++i) // a = 100 to 100 step 0.01
    {
        const double a = i / 10.0;
        RED_DutyCycle= (uint8_t) ((double)period/a);
        GREEN_DutyCycle= period;
        BLUE_DutyCycle= 0;
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);

    }
    for (i = 0; i <= 100 ; ++i) // a = 0 to 1 step 0.01
    {
        const double a = i / 100.0;
        RED_DutyCycle= 0;
        GREEN_DutyCycle= period;
        BLUE_DutyCycle= (uint8_t) (a*(double)period);
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);

    }
    for (i = 10; i <= 100 ; ++i) // a = 100 to 100 step 0.01
    {
        const double a = i / 10.0;
        RED_DutyCycle= 0;
        GREEN_DutyCycle= (uint8_t) ((double)period/a);
        BLUE_DutyCycle= period;
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);
    }
    for (i = 0; i <= 100 ; ++i) // a = 0 to 1 step 0.01
    {
        const double a = i / 100.0;
        RED_DutyCycle= (uint8_t) (a*(double)period);
        GREEN_DutyCycle= 0;
        BLUE_DutyCycle= period;
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);

    }
    for (i = 10; i <= 100 ; ++i) // a = 100 to 100 step 0.01
    {
        const double a = i / 10.0;
        RED_DutyCycle= period;
        GREEN_DutyCycle= 0;
        BLUE_DutyCycle= (uint8_t) ((double)period/a);
        BUZZER_DutyCycle=0;
        initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle);
        __delay_cycles(time);
    }
}

LedColors ledState(uint16_t JoyStickValue)
{
    PBS2=GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN2);
    switch (PBS2)
    {
        case GPIO_INPUT_PIN_HIGH:
        {
            if (JoyStickValue== 0xFFFF)
                return (BLUE);
            else if (JoyStickValue > YHIGH)
            {
                period=timerperiod/2;
                return (RED);
            }
            else if (JoyStickValue<YLOW )
            {
                period=timerperiod;
                return (GREEN);
            }
            else
                return (YELLOW);
        }
        case GPIO_INPUT_PIN_LOW:
        {
            if (JoyStickValue== 0xFFFF)
                return (BLUE);
            else if (JoyStickValue > YHIGH)
            {
                period=timerperiod/2;
                return (PURPLE);
            }
            else if (JoyStickValue<YLOW )
            {
                period=timerperiod;
                return (TEAL);
            }
            else
                return (CITRUS);
        }
    }
}

void rgbDriver(LedColors ledColor)
{
    // Value is the count which will give us our desired Duty Cycle
    uint8_t RED_DutyCycle;
    uint8_t GREEN_DutyCycle;
    uint8_t BLUE_DutyCycle;
    uint8_t BUZZER_DutyCycle;
    switch(ledColor)
    {
        case RED:
        {
            RED_DutyCycle= period;
            GREEN_DutyCycle=0;
            BLUE_DutyCycle= 0;
            BUZZER_DutyCycle=(period/2); // (50 duty cycle) instead of putting "50"
                                              // directly I put the timer period in half
            break;
        }
        case PURPLE:
        {
            RED_DutyCycle= (period/2);
            GREEN_DutyCycle=0;
            BLUE_DutyCycle= (period/2);
            BUZZER_DutyCycle=(period/2);
            break;
        }
        case GREEN:
        {
            RED_DutyCycle= 0;
            GREEN_DutyCycle=period;
            BLUE_DutyCycle= 0;
            BUZZER_DutyCycle=(period/2);
            break;
        }
        case TEAL:
        {
            RED_DutyCycle= 0;
            GREEN_DutyCycle=(period/2);
            BLUE_DutyCycle= (period/2);
            BUZZER_DutyCycle=(period/2);
            break;
        }
        case YELLOW:
        {
            RED_DutyCycle= period;
            GREEN_DutyCycle=period;
            BLUE_DutyCycle= 0;
            BUZZER_DutyCycle=0;
            break;
        }
        case CITRUS:
        {
            RED_DutyCycle= (period/2);
            GREEN_DutyCycle=(period/2);
            BLUE_DutyCycle= 0;
            BUZZER_DutyCycle=0;
            break;
        }
        case BLUE:
        {
            RED_DutyCycle= 0;
            GREEN_DutyCycle=0;
            BLUE_DutyCycle= period;
            BUZZER_DutyCycle=0;
            break;
        }
         case OFF:
        {
            RED_DutyCycle= 0;
            GREEN_DutyCycle=0;
            BLUE_DutyCycle= 0;
            BUZZER_DutyCycle=0;
            break;
        }
    }
    initTimerB0(RED_DutyCycle,GREEN_DutyCycle,BLUE_DutyCycle,BUZZER_DutyCycle); // needs one more parameter, which is the buzzer duty cycle
}

void initTimerB0(uint8_t RED_DutyCycle,uint8_t GREEN_DutyCycle,uint8_t BLUE_DutyCycle,uint8_t BUZZER_DutyCycle)
{
   TIMER_B_configureUpMode();
   TIMER_B_initCompareRegister_6(BUZZER_DutyCycle);
   TIMER_B_initCompareRegister_5(RED_DutyCycle);
   TIMER_B_initCompareRegister_4(GREEN_DutyCycle);
   TIMER_B_initCompareRegister_3(BLUE_DutyCycle);
   Timer_B_clearTimerInterrupt(TIMER_B0_BASE);
   Timer_B_startCounter(TIMER_B0_BASE,TIMER_B_UP_MODE);
}
void TIMER_B_configureUpMode(void)
{
    Timer_B_initUpModeParam MyTimerB0;
        MyTimerB0.clockSource=TIMER_B_CLOCKSOURCE_SMCLK;
        MyTimerB0.clockSourceDivider=TIMER_B_CLOCKSOURCE_DIVIDER_10;
        MyTimerB0.timerPeriod=period;
        MyTimerB0.timerInterruptEnable_TBIE=TIMER_B_CCIE_CCR0_INTERRUPT_DISABLE;
        MyTimerB0.timerClear= TIMER_B_DO_CLEAR;
        MyTimerB0.startTimer= false;
    Timer_B_initUpMode(TIMER_B0_BASE, &MyTimerB0);
}
void TIMER_B_initCompareRegister_6(uint8_t BUZZER_DutyCycle)
{
    Timer_B_initCompareModeParam MyTimerB0;
        MyTimerB0.compareRegister=TIMER_B_CAPTURECOMPARE_REGISTER_6;
        MyTimerB0.compareInterruptEnable=TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
        MyTimerB0.compareOutputMode=TIMER_B_OUTPUTMODE_RESET_SET;
        MyTimerB0.compareValue= BUZZER_DutyCycle;
    Timer_B_initCompareMode(TIMER_B0_BASE,&MyTimerB0);
}
void TIMER_B_initCompareRegister_5(uint8_t RED_DutyCycle)
{
    Timer_B_initCompareModeParam MyTimerB0;
        MyTimerB0.compareRegister=TIMER_B_CAPTURECOMPARE_REGISTER_5;
        MyTimerB0.compareInterruptEnable=TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
        MyTimerB0.compareOutputMode=TIMER_B_OUTPUTMODE_RESET_SET;
        MyTimerB0.compareValue= RED_DutyCycle;
    Timer_B_initCompareMode(TIMER_B0_BASE,&MyTimerB0);
}
void TIMER_B_initCompareRegister_4(uint8_t GREEN_DutyCycle)
{
    Timer_B_initCompareModeParam MyTimerB0;
        MyTimerB0.compareRegister=TIMER_B_CAPTURECOMPARE_REGISTER_4;
        MyTimerB0.compareInterruptEnable=TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
        MyTimerB0.compareOutputMode=TIMER_B_OUTPUTMODE_RESET_SET;
        MyTimerB0.compareValue= GREEN_DutyCycle;
    Timer_B_initCompareMode(TIMER_B0_BASE,&MyTimerB0);
}
void TIMER_B_initCompareRegister_3(uint8_t BLUE_DutyCyle)
{
    Timer_B_initCompareModeParam MyTimerB0;
        MyTimerB0.compareRegister=TIMER_B_CAPTURECOMPARE_REGISTER_3;
        MyTimerB0.compareInterruptEnable=TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
        MyTimerB0.compareOutputMode=TIMER_B_OUTPUTMODE_RESET_SET;
        MyTimerB0.compareValue= BLUE_DutyCyle;
    Timer_B_initCompareMode(TIMER_B0_BASE,&MyTimerB0);
}


void ADC_init(){
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);
    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_16_CYCLES,
                               ADC12_B_CYCLEHOLD_4_CYCLES,
                               ADC12_B_MULTIPLESAMPLESENABLE);
    ADC12_B_configureMemoryParam joyStickXParam = {0};
    joyStickXParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    joyStickXParam.inputSourceSelect = ADC12_B_INPUT_A2;
    joyStickXParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickXParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    joyStickXParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickXParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickXParam);
    //  JoyStickYParam Structure
    ADC12_B_configureMemoryParam joyStickYParam = {0};
    joyStickYParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    joyStickYParam.inputSourceSelect = ADC12_B_INPUT_A15;
    joyStickYParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickYParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    joyStickYParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickYParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickYParam);
    // Clear Interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);
}

void joyStick_init()
{
    // JoyStick X
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3, GPIO_TERNARY_MODULE_FUNCTION);
    // JoyStick Y
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2, GPIO_TERNARY_MODULE_FUNCTION);
}

// LCD_Init
// Configures mkII LCD display
// Inputs: none
// Returns: none
void LCD_init()
{
    /* Initializes display */
    Crystalfontz128x128_Init();
    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(0);
    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
}
// config_mkII
// Configures mkII RGB LED and PB S1 and S2
// Inputs: none
// Returns: none

void config_mkII(){

    PMM_unlockLPM5();
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN2);
    //  P3.7 BUZZER
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
    // P3.6 RED LED
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION);
    // P3.5 GREEN LED
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
    // P3.4 BLUE LED
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,GPIO_PIN4,GPIO_PRIMARY_MODULE_FUNCTION);
    LedColors x=OFF;
    rgbDriver(x);
}
