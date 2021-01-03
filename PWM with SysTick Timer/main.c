/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 27, 2020
 *
 * Change the brightness of an LED using PWM with altering rate by SysTick Timer.
 *
 * Set the PWM for P2.4 and P2.5 to display the use of PWM. The software will use SysTick to
 * alter the duty cycle of the LED every 10 ms. It will use UART0 (P1.1 and P1.2) to display message
 * via serial @ 9600 baud rate with 1 stop bit and no parity.
 *
//                MSP432P401x
//             -----------------
//            |                 |
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

#define MAX 30000 //macro for SysTick counter
uint16_t pulse = 0; //value to hold the current pulse

char buffer[16]; //buffer to be displayed
void percentage(); //display percentage of duty cycle
void set3MhzClock(); //set the SMCLK @ 3Mhz
void sendMessage(char *message); //send message via UART0
void pcEchoSetup(); //setup for UART0
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set3MhzClock(); //set clock at 3Mhz
    pcEchoSetup(); //set UART0

    //set P2.4 and P2.5 for PWM
    P2->SEL0 |= BIT4|BIT5;
    P2->SEL1 &= BIT4|BIT5;
    P2->DIR |= BIT4 | BIT5;
    P2->OUT |= BIT4 | BIT5;

    //Set the timerA0
    TIMER_A0->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_0 | TIMER_A_CTL_TASSEL_2; // upMode, SMCLK, divide by 2^0
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; //set Reset/Set...always use for PWM
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; //set for RESET/SET
    TIMER_A0->CCR[0] = 60000 - 1; //set frequency at 20ms or 50hz
    TIMER_A0->CCR[1] = pulse; //set duty at 5%
    TIMER_A0->CCR[2] = pulse; //set at the first index of the pulse

    //set Systick
    SysTick->CTRL |= 0x07; //turn on sysTick and enable interrupt
    SysTick->LOAD = MAX - 1; //set the max load
    SysTick->VAL = 1; // set intial value

    __enable_irq(); //enable global interrupts

    while(1){} //infinite loop

}
void SysTick_Handler(void){
    pulse += 100;//update pulse
    if(pulse > 60000) //check if pulse exceeds 16 bits
        pulse = 1; //set back to intiali state
    percentage(); //display current percentage
    //set the pusle to the P2.4 and P2.5
    TIMER_A0->CCR[1] = pulse - 1;
    TIMER_A0->CCR[2] = pulse - 1;
}
void pcEchoSetup(){
       P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
       P1->SEL1 &= ~(BIT2+BIT3);
           // Configure UART
       EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
       EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
       // Baud Rate calculation
       // 12000000/(16*9600) = 78.125
       // Fractional portion = 0.125
       // User's Guide Table 21-4: UCBRSx = 0x10
       // UCBRFx = int ( (78.125-78)*16) = 2
       EUSCI_A0->BRW = 19;                     // 12000000/16/9600
       EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
       EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
      // EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
     //  EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
       return;
}
void percentage(){
    //convert pulse to float
    float percent = (float)pulse/60000 * 100;
    sprintf(buffer,"%.2f\r\n", percent); //print to the buffer
    sendMessage(buffer);//send buffer to display via UART0
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
