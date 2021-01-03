/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 18, 2020
 *
 * Simple Blinking LED. RED LED will blink every
 * second to display GPIO outputs.
 *
                MSP432P401x
             -----------------
        /|\|                 |
         | |                 |
         --|RST              |
           |                 |
        >--|P1.4         P1.0|--> RED LED
            -----------------
*/
#include "msp.h"

void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	//Set port
	P1->DIR |= BIT0; //set BIT0 as OUTPUT
	P1->OUT &= ~BIT0; //set BIT0 as LOW
	P1->SEL0 &= ~BIT0; // clear SEL0
	P1->SEL1 &= ~BIT0; // clear SEL1

	while(1){
	    P1->OUT ^= BIT0; //use XOR to toggle LED
	    __delay_cycles(3000000); //delay for a second
	}
}
