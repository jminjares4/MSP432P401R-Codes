/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * June 8, 2020
 *
 *  ADC Multiple Channels Readings
 *
 *  Use P4.0,2,4 for a multiple channel readings. The software was develop
 *  intend to read an 3-axis accelerometer. It will read each channel and display
 *  it data via serial using UART0.
 *
//                MSP432P401x
//             -----------------
//            |                 |
//            |                 |
//            |            P4.0 |<-- Input
//            |            P4.2 |<-- Input
//            |____________P4.4_|<-- Input
//
*/
#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void pcEchoSetup(); //set UART0 at 9600 bits/seconds
void set3MhzClock(); //set DCO clock at 3Mhz
void sendMessage(char *message); //send message through Putty
volatile uint16_t MotionSensor[3];
uint32_t data[3];
char buffer[24];
int count = 0;
int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |  WDT_A_CTL_HOLD;  // Stop watchdog timer

    set3MhzClock(); //set 3Mhz
    pcEchoSetup(); //set UART0

    // Configure GPIO
    P4->SEL1 |= BIT5 | BIT4 | BIT2 |BIT0;   // Enable A/D channel A0-A3
    P4->SEL0 |= BIT5 | BIT4 | BIT2 |BIT0;

    memset(data, 0,sizeof(data)); //clear data

    // Enable global interrupt
    __enable_irq();

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);// Enable ADC interrupt in NVIC module

    // Turn on ADC14, extend sampling time to avoid overflow of results
    ADC14->CTL0 = ADC14_CTL0_ON |
            ADC14_CTL0_MSC |
            ADC14_CTL0_SHT0__192 |
            ADC14_CTL0_SHP |
            ADC14_CTL0_CONSEQ_3;

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_13;   //P4.0
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_11;   //P4.2
    ADC14->MCTL[2] = ADC14_MCTLN_INCH_9| ADC14_MCTLN_EOS;;    //P4.4

    ADC14->IER0 = ADC14_IER0_IE2;           // Enable ADC14IFG.3

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();
    while(1){
        // Start conversion-software trigger
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
        __sleep();
    }
}
// ADC14 interrupt service routine
void ADC14_IRQHandler(void){
    //check flags
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG2){
        count++;
        data[0] += ADC14->MEM[0];   // Move A0 results, IFG is cleared
        data[1] += ADC14->MEM[1];   // Move A1 results, IFG is cleared
        data[2] += ADC14->MEM[2];   // Move A2 results, IFG is cleared
        if(count >= 10){ //generate a LP filter to clean signal
            sprintf(buffer,"%u %u %u \r\n",data[0]/count,data[1]/count,data[2]/count); //print to buffer
            sendMessage(buffer); //display buffer
            count = 0; //reset counter
            memset(data, 0,sizeof(data)); //clear data
        }
    }
}
void pcEchoSetup(){
    P1->SEL0 |= BIT2 | BIT3; // set 2-UART pin as secondary function
    P1->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK; // Configure eUSCI clock source for SMCLK
    EUSCI_A0->BRW = 19;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
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
