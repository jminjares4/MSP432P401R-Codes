/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 21, 2020
 *
 * BINARY COUNTER W/Interrupts
    PD -> PULL DOWN
    PU -> PULL UP
//                MSP432P401x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//CLEAR    >--|P1.4(PU)     P4.2|--> LED
//            |                 |
//INPUT   >---|P2.6(PU)     P4.1|--> LED
//            |                 |
//INPUT   >---|P3.0(PD)     P4.0|-->LED
//

*/

#include "msp.h"
void port4Setup(); // LED
void port1Interrupt(); //internal button configuration
void port2Interrupt(); //pull up configuration
void port3Interrupt(); // pulldown configuration
int counter = 0; //create a global variable
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	port4Setup(); //setup output leds
	port1Interrupt(); //call port 1 setup
	port2Interrupt(); //call port 2 setup
	port3Interrupt(); //call port 3 setup

	//set NVIC interrupts
	NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
	NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);
	NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);
	__enable_irq(); //enable global interrupts

	while(1){
	  	}

}
void PORT1_IRQHandler(void)
{
    counter = 0 ;//reset counter
    P1->IFG &= ~P1->IFG; //clear flag

    P4->OUT = counter; //outpit
}
void PORT2_IRQHandler(void)
{
    counter += 1; //increment counter
    if(counter > 7) //if overflows reset at 0
        counter = 0;
    P2->IFG &= ~P2->IFG; //clear flag
    P4->OUT = counter; //output
}
void PORT3_IRQHandler(void)
{
    counter -= 1; //decrement counter
    if(counter < 0) // if it overflow reset at 7
        counter = 7;
    P3->IFG &= ~P3->IFG; //clear flags
    P4->OUT = counter; // output
}

void port4Setup()
{

    P4->DIR |= 0x07; // set output BIT0-3 0000 0111
    P4->OUT &= ~(0x07); //clear output
    P4->SEL0 &= ~(0x07); //clear sels
    P4->SEL1 &= ~(0x07); //clear sels
}

void port3Interrupt()
{
    P3->IE |= BIT0; //enable bit3 as an interrupt
    P3->IES &= ~BIT0; // set pull down configuration
    P3->IFG &= ~(P3->IFG); //clear the interrupt flag
    return;
}
void port2Interrupt()
{
    P2->IE |= BIT6; //enable bit6 as an interrupt
    P2->IES |= BIT6; //set as a pull up configuration
    P2->REN |= BIT6; //enable bit for pull up
    P2->IFG &= ~(P2->IFG); //clear interrupt flag
    return;
}
void port1Interrupt()
{
    P1->IE |= BIT4; //enable interrupt pin
    P1->IES |= BIT4; //set as pull up configuration
    P1->REN |= BIT4; //enable bit for pull up
    P1->IFG &= ~(P1->IFG); //clear flag
    return; //exit
}
