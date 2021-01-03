/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 23, 2020
 *
 * Use TA0.1-4 PWM output with led to demonstrate the intensity change
//             -----------------
//            |                 |
//            |                 |
//            |RST              |
//            |                 |
//            |             P2.2|--> LED
//            |                 |
//            |             P2.1|--> LED
//            |                 |
//            |             P2.0|-->LED
//
*/
#include "msp.h"
#define timerClk 3000000
void port2Setup(); //setup onboard led
void timerA1Setup(int freq, int timerFreq);
void pwmPins();
void timerA0PwmSetup(int freq, int percent, int timerFreq);
void TimerA1InHertz(int hz, int timerHz);
int tic = 0; //globl variable to count number of times enter the interrupt

void pwmPins();
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    P1->DIR |= BIT0; //set LED 1 as outpuyt
    P1->OUT |= BIT0; //set LED on high

    pwmPins(); //set the PWM pins

    timerA1Setup(60, timerClk); //set at 60hz @ timerClk (3e6)
    timerA0PwmSetup(60, 1, timerClk); //set at 60 hz @ timerClk (3e6) starting @ 1

    NVIC->ISER[0] = 1 << ((TA1_0_IRQn) & 31); //establish the NVIC setup for the interrupt TA1.0

    __enable_irq(); //enable global interrupts
    while(1){} //infinte loop
}
void TA1_0_IRQHandler(void)
{
    tic++; //increment tic
    if(tic >= 60/2) //when tic reaches 100, 3M/30k = 100, therefore every 100 tic will be a second, manipulate the tic by 2, it will double or half the speed
    {
        tic = 0; //reset the tic
        P1->OUT ^= BIT0; //output the led

        //increment each PWM pin by 1000
        TIMER_A0->CCR[4] += 1000;
        TIMER_A0->CCR[3] += 1000;
        TIMER_A0->CCR[2] += 1000;
        TIMER_A0->CCR[1] += 1000;
        //if value exceed 16 bits, reset to 500
        if(TIMER_A0->CCR[4] >= 50000) TIMER_A0->CCR[4] = 500;
        //if value exceed 16 bits, reset to 500
        if(TIMER_A0->CCR[3] >= 50000) TIMER_A0->CCR[3] = 500;
        //if value exceed 16 bits, reset to 500
        if(TIMER_A0->CCR[2] >= 50000) TIMER_A0->CCR[2] = 500;
        //if value exceed 16 bits, reset to 500
        if(TIMER_A0->CCR[1] >= 50000) TIMER_A0->CCR[1] = 500;
    }
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear the capture and compare flag of the timer
}
void pwmPins(){
    P2->SEL0 |= 0xF0; //set 1111 0000 bits
    P2->SEL1 &= ~0xF0; //bit clear the 1111 0000 bits
    P2->DIR |= 0xF0; //set it as high
    return;
}
void timerA1Setup(int freq, int timerFreq){
    int timerLimitCount = (int)timerFreq/freq;
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // capture and compare interrupt enable
    TIMER_A1->CCR[0] = timerLimitCount - 1; //set limit at 30000, 3Mhz/30000, every 100 tics will be a 1 sec
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP; //set SMCLK at 3mhz and upmode, upmode will count from 0->CCR[0]
    return;
}
void timerA0PwmSetup(int freq, int percent, int timerFreq){
    //set the percent as a float
    float percentInDec = (float)percent/100;
    //get the MAX timer count
    int timerLimitCount = (int)timerFreq/freq;
    //set the duty cycle
    int dutyCycle = (int)(timerLimitCount * percentInDec);
    //set SMCLK and UP Mode
    TIMER_A0->CTL =  TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP;

    //Set CCR0 with its max count
    TIMER_A0->CCR[0] = timerLimitCount - 1;
    //set the pins as PWM
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCTL[4] =  TIMER_A_CCTLN_OUTMOD_7;
    //set CCRx duty cycle
    TIMER_A0->CCR[1] = dutyCycle;
    TIMER_A0->CCR[2] = dutyCycle;
    TIMER_A0->CCR[3] = dutyCycle;
    TIMER_A0->CCR[4] = dutyCycle;
    }
void TimerInHertz(int hz, int timerHz)
{
    int frequency = (int)timerHz/hz;
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // capture and compare interrupt enable
    TIMER_A1->CCR[0] = frequency - 1; //set limit at 30000, 3Mhz/30000, every 100 tics will be a 1 sec
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP; //set SMCLK at 3mhz and upmode, upmode will count from 0->CCR[0]
    return;
}
