/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 27, 2020
 *
 * PWM with interrupt button to change LED brightness
 *
 * Set P2.4 and P2.5 as PWM to display the brightness of LED. The LED
 * will be alter by the use of the onboard switch on the board. The switches
 * will be uses a interrupt buttons as pull up. Once a button is press it will
 * increment or decrement the brightness of the LED. The current readings will
 * be display via the serial port using UART0 (P1.1 and P1.2).
 *
//                MSP432P401x
//             -----------------
//   Input -->|P1.1         P1.4|<-- Input
//            |                 |
//            |RST              |
//            |                 |
//            |     TA0.1-> P2.4|--> LED
//            |            (PWM)|
//            |     TA0.2-> P2.5|--> LED
//            |_________________|
//
*/
#include "msp.h"
#include <string.h>
#include <stdio.h>
uint16_t pulse = 0; //value to hold pulse
char buffer[16];//buffer to store message
void percentage(); //display the percentage
void set3MhzClock(); //set SMCLK @ 3Mhz
void sendMessage(char *message); //send message via UART0
void pcEchoSetup(); //set UART0
void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	//set clock @ 3Mhz
	set3MhzClock();
	//set UART0
	pcEchoSetup();

	//Set P1.1 and P1.4 as pullup interrupt
	P1->IE |= BIT1 | BIT4; //set pin for interrupt
	P1->IES |= BIT1 | BIT4; // set as pull-up
	P1->REN |= BIT1|BIT4; //enable resistors
	P1->IFG &= ~(BIT1|BIT4); //clear flags

	//set PWM
	P2->SEL0 |= BIT4|BIT5; //set P2.4 and P2.5 as PWM
	P2->SEL1 &= BIT4|BIT5; //clear bits
	P2->DIR |= BIT4 | BIT5; //set as OUTPUT
	P2->OUT |= BIT4 | BIT5; // set as HIGH

	//Set TimerA0 for PWM
	TIMER_A0->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_0 | TIMER_A_CTL_TASSEL_2; // upMode, SMCLK, divide by 2^0
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; //set Reset/Set...always use for PWM
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; //set for RESET/SET
    TIMER_A0->CCR[0] = 60000 - 1; //set frequency at 20ms or 50hz
    TIMER_A0->CCR[1] = pulse; //set duty at 5%
    TIMER_A0->CCR[2] =pulse; //set at the first index of the pulse


    //set PORT 1 Interrupt bits
    NVIC->ISER[1] |= 1 << (PORT1_IRQn & 31);

    //enable global interrupts
    __enable_irq();

    while(1){}
}
void PORT1_IRQHandler(void){
    //check interrupt flag
    if((P1->IFG & BIT1) == BIT1){
        pulse+= 1000; //increment pulse
    }
    //check interrupt flag
    if((P1->IFG & BIT4) == BIT4){
        pulse -= 1000; //decrement pulse
    }
    //check boundaries
    if(pulse > 60000 || pulse < 1){
        pulse = 1; //set @ initial state
    }
    percentage();//display percentage
    //set pulse
    TIMER_A0->CCR[1] = pulse - 1;
    TIMER_A0->CCR[2] = pulse - 1;
    //clear interrupt flag
    P1->IFG &= ~(P1->IFG);
}
void pcEchoSetup(){
       P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
       P1->SEL1 &= ~(BIT2+BIT3);
       // Configure UART
       EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
       EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
       EUSCI_A0->BRW = 19;                     // 12000000/16/9600
       EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
       EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
       return;
}
void percentage(){
    //convert pulse to percentage
    float percent = (float)pulse/60000 * 100;
    sprintf(buffer,"%.2f\r\n", percent); //print to buffer
    sendMessage(buffer);//display buffer
    return;
}
void sendMessage(char *message){
    int i; //local variable for the loop
      for(i = 0; i < strlen(message); i++){ //iterate over the string length
          while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //busywait
                 EUSCI_A0->TXBUF = message[i]; //send character
      }
      return;
}
void set3MhzClock(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 |CS_CTL1_SELM_3; // Select ACLK = REFO// MCLK = DCO // SMCLK = DCO
   CS->KEY = 0; //lock CS module
}
