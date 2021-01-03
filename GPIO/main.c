/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 20, 2020
 *
    PD -> PULL DOWN
    PU -> PULL UP
//                MSP432P401x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//CLEAR    >--|P1.4         P4.2|--> LED
//            |                 |
//INPUT   >---|P2.6(PU)     P4.1|--> LED
//            |                 |
//INPUT   >---|P3.0(PD)     P4.0|-->LED
//

*/
#include "msp.h"
void pullUp(); //pull up configuration
void pullDown(); //pull down configuration
void port4Setup(); //set output for the port 4
void onboardButton(); //set up internal button
int counter = 0; //create a global variable to count
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    onboardButton();
    pullUp(); // configure pullup function
    pullDown(); //configure pulldown function
    port4Setup(); // set port 4

    while(1){
        if(counter > 7 || (P1->IN & BIT4) == 0) //if counter over flows 3 bit, then clear OR if onboard button is pressed
            counter = 0; // reset value to 0
        if((P2->IN & BIT6) == 0) // PULL UP test
            counter++; //increment value
        if((P3->IN & BIT0)) // PULL DOWN test
            counter--; // decrement value
     _delay_cycles(350000); //delay
        P4->OUT = counter; //output the count
    }
}

void pullUp()
{
    P2->DIR &= ~(BIT6); //set as input BIT6 X0XX XXXX
    P2->SEL0 &= ~(BIT6); //clear the sels
    P2->SEL1 &= ~(BIT6); //clear the sels
    P2->REN |= BIT6; //set up resistor enable for pull up
}
void pullDown()
{
    P3->DIR &= ~(BIT0); // set as input BIT0 XXXX XXX0
    P3->SEL0 &= ~(BIT0); //clear sels
    P3->SEL1 &= ~(BIT0); //clear sels
    P3->REN &= ~BIT0; // clear BIT0 for REN as it will be pull-down
}
void port4Setup()
{

    P4->DIR |= 0x07; // set output BIT0-3 0000 0111
    P4->OUT &= ~(0x07); //clear output
    P4->SEL0 &= ~(0x07); //clear sels
    P4->SEL1 &= ~(0x07); //clear sels
}
void onboardButton()
{

    P1->DIR &= ~(BIT4); //set BIT4 as input
    P1->SEL0 &= ~BIT4; //clear sel
    P1->SEL1 &= ~BIT4; //clear sel
    P1->REN |= BIT4; //set up as pull up
}
