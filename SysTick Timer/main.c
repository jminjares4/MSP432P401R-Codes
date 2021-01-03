/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 23, 2020
 *
 * BINARY COUNTER W/SysTick Timer running at 10ms
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
void port2Setup(); //led setup for the onboard led
void sysTickSetup(int load); //setup systick
void loop(); //infinte loop
int count = 0; //led counter
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	port2Setup(); //call the port setup function
	sysTickSetup(300000); //enable systick with a load of 300k, running at 10ms

	__enable_irq(); //enable global interrupt
	loop(); //infinte loop
}
void SysTick_Handler(void) //systick handler
{
    count++; //increment count every 10ms
    if(count > 7) //if count overflows reset it
        count = 0; //set count at 0
}

void port2Setup()
{
    P2->DIR |= 0x07; //set up onboard led, BIT0-2, 0000 0111
    P2->OUT &= ~(0x07); //bit clear the output
    P2->SEL0 &= ~(0x07); //clear out the output pins from the sel perpherial
    P2->SEL1 &= ~(0x07); //clear sel1 port
    return; //exit setup
}
void sysTickSetup(int load)
{
    SysTick->CTRL = 0x07; //enable timer, activate the clock source, and enable interrupt
    SysTick->LOAD = load; //set load with the parameter of the function
    SysTick->VAL = 0x01; //set current value at 1
    return; //exit setup
}
void loop()
{
    while(1){
        P2->OUT = count; //out the led
    }
}
