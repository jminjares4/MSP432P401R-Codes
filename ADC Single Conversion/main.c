/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * June 7, 2020
 *
 *  Set P5.4 as a analog reading with a sample trigger of 100hz. Turn Led when
 *  ADC14->MEM[0] exceeds (2^14)/2. Send message by PCEcho using UART with Putty Terminal
 *
//                MSP432P401x
//             -----------------
//            |                 |
//            |                 |
//            |          (P5.4) |--> 10k Pot
//            |                 |
//            |                 |
//            |            P2.2 |--> Blue Led
//            |                 |
//            |_________________|
//
*/
#include "msp.h"
#include <stdio.h> //need libray for sprintf functionality
#include <string.h> //need for strlen function
void adcSetup(); // set adc at a sampling rate of 100hz
unsigned adcRawReading; // create a global variable to save ADC readings
void port5Setup(); //setup port 5 for analog reading
void port2Setup(); //set P2.2 as led indicator
void set3MhzClock(); // configure clock at 3mhz
void pcEchoSetup(); // use UART0 to PCEcho adc values
void sendMessage(char *message); // send message at 9600 baudrate
char buffer[24]; // message string
int tic = 0;
void timerA2Setup();
void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	set3MhzClock(); //set clock at 3Mhz
	port2Setup(); // set P2.2 as output
	port5Setup(); // set P5.3 as analog reading
	adcSetup(); // set adc at 100 hz with sample trigger
	pcEchoSetup(); // set UART at 9600
	timerA2Setup();
	NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31); //enable adc
	NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31); // enable uart
	NVIC->ISER[0] = 1 << ((TA2_0_IRQn) & 31);
	__enable_irq(); // enable global interrupts

	sendMessage("Testing...\r\n");
	while(1){} //infinite loop
}
void TA2_0_IRQHandler(){
    tic++;
    //check if the time has elapse
    if(tic == 25){
        ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC); //trigger ADC
    }
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear flag
}
void timerA2Setup(){
    TIMER_A2->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_TASSEL_2; //set @ SMCLK and UP mode
    TIMER_A2->CCTL[0] = TIMER_A_CCTLN_CCIE; //enable capture and compare interrupt
    TIMER_A2->CCR[0] = 60000 - 1; //set count
}
void adcSetup(){
    // set as sample trigger, predivde clock by 64, divide by 5, sample hold time of 14, adc on
    ADC14->CTL0 =  ADC14_CTL0_SHP| ADC14_CTL0_ON | ADC14_CTL0_SHT0__192; // 3mhz/64/6/(64 + 14+2) == 97. hertz
    ADC14->CTL1 = ADC14_CTL1_RES_3;         // Use sampling timer, 14-bit conversion results
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt>
    //ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //enable conversion and start conversion
}
void ADC14_IRQHandler(void){
    adcRawReading = ADC14->MEM[0]; //save adc reading
    sprintf(buffer, "ADC->%u VOLT: %lf\r\n", adcRawReading, (3.0/16384 * adcRawReading)); // output the message of adc reading
    sendMessage(buffer); //send message
    tic = 0;
}
void port5Setup(){
    P5->SEL1 |= BIT4;   // Configure P5.4 for ADC
    P5->SEL0 |= BIT4;  // Confiugre P5.4 for ADC
    P5->DIR &= ~BIT4; //bit clear bit4
    P5->OUT &= ~BIT4; // bit clear bit4
}
void port2Setup(){
    P2->SEL0 &= ~BIT2; // clear select pin
    P2->SEL1 &= ~BIT2; // clear selet pin
    P2->DIR |= BIT2; // set P2.2 as ouput
    P2->OUT |= BIT2; // output P2.2
}

void set3MhzClock(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 |CS_CTL1_SELM_3; // Select ACLK = REFO// MCLK = DCO // SMCLK = DCO
    CS->KEY = 0; //lock CS module
}
void sendMessage(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){
      while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
             EUSCI_A0->TXBUF = message[i];
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
   EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
   EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
   // Enable eUSCIA0 interrupt in NVIC module
}
