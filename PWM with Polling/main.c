/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 27, 2020
 *
 * PWM with input button to change LED brightness
 *
 * Set P2.4 and P2.5 as PWM to display the brightness of LED. The LED
 * will be alter by the use of the onboard switch on the board. The switches
 * will be uses a GPIO buttons as pull up. Once a button is press it will
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
uint16_t pulse = 0; //value to store pulse

char buffer[16]; //buffer to hold message
void percentage(); //displays the percentage of the LED
void set3MhzClock(); //set the SMCLK at 3Mhz
void sendMessage(char *message); //send data via UART0 to be displayed
void pcEchoSetup(); //set UART0
void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set3MhzClock(); //set 3Mhz
    pcEchoSetup(); //set UART0

    //set Onboard switches
    P1->DIR &= ~(BIT1|BIT4);//set as input
    P1->OUT |= BIT1 | BIT4; //set as pull up
    P1->REN |= BIT1|BIT4; //turn on resistor

    //set PWM
    P2->SEL0 |= BIT4|BIT5; //set for PWM
    P2->SEL1 &= BIT4|BIT5; //clear SEL1
    P2->DIR |= BIT4 | BIT5; //set on HIGH
    P2->OUT |= BIT4 | BIT5; //set as OUTPUT

    //TimerA0 setup
    TIMER_A0->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_0 | TIMER_A_CTL_TASSEL_2; // upMode, SMCLK, divide by 2^0
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; //set Reset/Set...always use for PWM
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; //set for RESET/SET
    TIMER_A0->CCR[0] = 60000 - 1; //set frequency at 20ms or 50hz
    TIMER_A0->CCR[1] = pulse; //set duty at 5%
    TIMER_A0->CCR[2] =pulse; //set at the first index of the pulse

    while(1){
        //check  P1.1
        if((P1->IN & BIT1) == 0){
            pulse += 1000; //increment pulse
        }
        //check P1.4
        if((P1->IN & BIT4) == 0){
            pulse -=1000; //decrement pulse
        }
        //check for out of bound
        if(pulse > 60000 || pulse < 1){
            pulse = 1; //set on intial state
        }
        __delay_cycles(200000); //20ms delay
        percentage(); //display pulse
        //set pulse
        TIMER_A0->CCR[1] = pulse - 1;
        TIMER_A0->CCR[2] = pulse - 1;
    }
}
void pcEchoSetup(){
   P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
   P1->SEL1 &= ~(BIT2+BIT3);
   // Configure UART
   EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
   EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
   EUSCI_A0->BRW = 19;                     // 3000000/16/9600
   EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
   EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
   return;
}
void percentage(){
    //convert pulse to percentage
    float percent = (float)pulse/60000 * 100;
    sprintf(buffer,"%.2f\r\n", percent); //print to buffer
    sendMessage(buffer); //display buffer
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
