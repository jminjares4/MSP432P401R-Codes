/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 23, 2020
 *
 * BINARY COUNTER W/TimerA0_0 running at 10ms
//                MSP432P401x
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
void port2Setup(); //setup onboard led
void timerA0Setup(); //setup timer at 30khz/3Mhz
int tic = 0; //globl variable to count number of times enter the interrupt
int count = 0; //iterate over the onboard led
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	port2Setup(); //call function for onboard led
	timerA0Setup(); //call function of the TIMER_A0 setup
	NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31); //establish the NVIC setup for the interrupt TA0.0

	__enable_irq(); //enable global interrupts
	while(1){} //infinte loop
}
void TA0_0_IRQHandler(void)
{
    tic++; //increment tic
    if(tic >= 100) //when tic reaches 100, 3M/30k = 100, therefore every 100 tic will be a second, manipulate the tic by 2, it will double or half the speed
    {   count++; //increment the bit counter
        tic = 0; //reset the tic
        P2->OUT = count; //output the led
    }
    if(count > 7) //if overflows reset it
        count = 0; //clear count
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear the capture and compare flag of the timer

}
void port2Setup()
{
    P2->DIR |= 0x07; //set up onboard led, BIT0-2, 0000 0111
    P2->OUT &= ~(0x07); //bit clear the output
    P2->SEL0 &= ~(0x07); //clear out the output pins from the sel perpherial
    P2->SEL1 &= ~(0x07); //clear sel1 port
    return; //exit setup
}
void timerA0Setup()
{
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // capture and compare interrupt enable
    TIMER_A0->CCR[0] = 30000; //set limit at 30000, 3Mhz/30000, every 100 tics will be a 1 sec
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP; //set SMCLK at 3mhz and upmode, upmode will count from 0->CCR[0]
    return;
}
