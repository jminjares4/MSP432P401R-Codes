/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 22, 2020
 *
 * BINARY COUNTER With Pull up and Pulldown configuration
    PD -> PULL DOWN
    PU -> PULL UP
//                MSP432P401x
//             -----------------
//            |                 |
//            |                 |
//0x00     >--|P1.1             |
//            |                 |
//0x07     >--|P1.4(PU)     P4.2|--> LED
//            |                 |
//INPUT   >---|P2.6(PU)     P4.1|--> LED
//            |                 |
//INPUT   >---|P2.5(PD)     P4.0|-->LED
//

*/
#include "msp.h"
void port1Setup(); //setup Por1 interrupt
void port2Setup(); //setup Port2 interrupt
void port4Setup(); //setup Port4 for output led
int count  = 0; //create a global variable to be alter with the interrupts
void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	//call function for the ports
	port1Setup();
	port2Setup();
	port4Setup();

	//intialize interrupt with NVIC perpherial
	NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
	NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);

	__enable_irq(); //enable global interrupts
	while(1){} //infinte loop
}
void PORT1_IRQHandler(void){ //port 1 interrupt routine
    if(P1->IFG & BIT1) //if flag equal Switch 1.1
        count = 0 ;//reset counter
    if(P1->IFG & BIT4) //if flag equal Switch 1.4
        count  = 7; //overflow 3 bit counter

    P4->OUT = count; //output
    P1->IFG &= ~(P1->IFG); //clear flag

}
void PORT2_IRQHandler(void){
    if(P2->IFG & BIT6){
        count += 1; //increment counter
        if(count > 7) //if overflows reset at 0
            count = 0;
    }
    if(P2->IFG & BIT5) //test if the BIT5 is pressed
    {
        count -= 1; //decrement counter
         if(count < 0) // if it overflow reset at 7
             count = 7;
    }
    P4->OUT = count; //output
    P2->IFG &= ~P2->IFG; //clear flag
}
void port1Setup(){
    P1->SEL0 &= ~(BIT1&BIT4);
    P1->SEL1 &= ~(BIT1 & BIT4);
    P1->IE |= BIT1 + BIT4; //enable interrupt pin
    P1->IES |= BIT1 + BIT4; //set as pull up configuration
    P1->REN |= BIT1 + BIT4; //enable bit for pull up
    P1->IFG &= ~(BIT1 + BIT4); //clear flag
    return; //exit
}
void port2Setup(){
    P2->IE |= BIT6 | BIT5; //enable BIT6 and BIT5 as interrupts
    P2->IES |= BIT6; //set BIT6 as PU and BIT5 as PD
    P2->REN |= BIT6; //enable pull up
    P2->IFG &= ~(P2->IFG); //clear interrupt flag
    return;
}
void port4Setup(){
    P4->DIR |= 0x07; // set output BIT0-3 0000 0111
    P4->OUT &= ~(0x07); //clear output
    P4->SEL0 &= ~(0x07); //clear sels
    P4->SEL1 &= ~(0x07); //clear sels
}
