/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * June 06, 2020
 *
 * Set A1 to read a potentiometer to iterate over the pwm of a SG90 servo motor for 5% to 10%.
 * Run SysTick at 25ms and after 125ms move the SG90 pulse gathered from A0.
 * NOTE:    can use ADC to control any PWM signal
//                MSP432P401x
//             -----------------
//            |          0V(GND)|--> Brown Pin(SG90)
//            |                 |
// 10k pot <--|P5.4(A1)   5.0V  |--> Red Pin(SG90)
//            |                 |
//            |     TA0.4-> P2.7|-->  Orange Pin (SG90)
//            |            (PWM)|
//            |                 |
//            |_________________|
//
*/
#include "msp.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
uint16_t adcValue = 0; //save current ADC reading
uint16_t pulse = 0; // set current pusle at 0
uint8_t tic = 0; // set tic at 0
uint16_t sum = 0; //set the total sum at 0
void adcSetup(); //adc setup with single channel, single conversion
void portSetup(uint8_t analogBit);//set port for ADC
void set3MhzClock(); //set SMCLK at 3Mhz
void pcEchoSetup(); //set up UART0 for PCECho to print at Putty Terminal
void sendMessage(char *message); //send message to Putty
void pwmPortSetup(uint8_t pwmBit); // set(P2.7)->TA0.4 for PWM
void pwmSetup(); //set PWM at 50 hz for servo motor
char buffer[48]; //string buffer to send message via PuttyTerminal
void sysTickSetup(uint32_t load); //enable sysTick at load
uint16_t remap(uint16_t value, uint16_t inputMin, uint16_t inputMax, uint16_t outputMin, uint16_t outputMax); //map adc value to pwm ranges
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	set3MhzClock(); //set clock at 3mhz // PUTTY, print to the terminal
	portSetup(BIT4); //set P2.4 for analog
	pwmPortSetup(BIT7); //set P2.7 for PWM at TA0.4
	pwmSetup(); //set TIMERA0.4 at OutMod7 at 50hz or 20ms
	pcEchoSetup(); //enable UART0 for pcEcho
	adcSetup(); //set adc single channel, single conversion, for A1

	NVIC->ISER[0] = (1 << (ADC14_IRQn & 31)); //set NVIC ADC

	sysTickSetup(75000); //set timer at 25ms delay
	__enable_irq(); // enable global interrupts

	while(1){ //infinte loop
	}
}
void sysTickSetup(uint32_t load)
{
    SysTick->CTRL |= 0x07; //enable interrupt, and current clock sources
    SysTick->LOAD = load; //set load as max value
    SysTick->VAL = 1; // set current value at 1
}
void adcSetup()
{
    //ADCON, sample timer trigger, single channel ((clock/preDivide)/divide)/(sampleHold + resolution + 2)
        ADC14->CTL0 = ADC14_CTL0_ON| ADC14_CTL0_SHT0__192|ADC14_CTL0_SHP|ADC14_CTL0_CONSEQ_0;
        ADC14->CTL1 = ADC14_CTL1_RES__14BIT; // 14 bit resolution, 2^14
        ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1; //A1
        ADC14->IER0 |= ADC14_IER0_IE0; //interrupt enable
       // ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //start conversion
        return;
}
void portSetup(uint8_t analogBit)
{
    P5->SEL0 |= analogBit; // enable select pin
    P5->SEL1 |= analogBit; //enable selct pin
    P5->DIR &= ~analogBit; //bit clear bit4
    P5->OUT &= ~analogBit; // bit clear bit4
    return;
}
void set3MhzClock()
{
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;                // Select ACLK = REFO // SMCLK = DCO   // MCLK = DCO
    CS->KEY = 0;
    return;
}
void ADC14_IRQHandler(void){
    adcValue = ADC14->MEM[0]; //store adcReading at the adcValue global variable
    sum += adcValue; // add adcValue to sum to average out the reading to avoid fluctuation
}
void SysTick_Handler(void){
    tic++;// increment tic
    if(tic == 5){ //after 125 ms move get the average pulse
        uint16_t average = sum/(tic - 1); //divide the sum by the tic - 1
        pulse = (uint16_t)remap(average, 0, 16384, 3000, 6000); //remap the analog reading to PWM range
        TIMER_A0->CCR[4] = pulse - 1; //set pulse for PWM to move the servo motor
        sprintf(buffer,"ADC14->MEM: %u\t\tPulse: %u\r\n", adcValue, pulse); //store string into buffer
        sendMessage(buffer);  //send message through Putty at 9600 bits/second
        tic = 0; //reset tic
        sum = 0; //rest sum
    }
    else
        ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC); //start conversion
}
void pcEchoSetup()
{
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
void sendMessage(char *message){
    int i; //local variable for the loop
    for(i = 0; i < strlen(message); i++){ //iterate over the string length
      while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //busywait
             EUSCI_A0->TXBUF = message[i]; //send character
    }
    return;
}
uint16_t remap(uint16_t value, uint16_t inputMin, uint16_t inputMax, uint16_t outputMin, uint16_t outputMax) {
  return (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin; //remap the ranges
}
void pwmSetup(){
    TIMER_A0->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_0 | TIMER_A_CTL_TASSEL_2; // upMode, SMCLK, divide by 2^0
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; //set Reset/Set...always use for PWM
    TIMER_A0->CCR[0] = 60000 - 1; //set frequency at 20ms or 50hz
    TIMER_A0->CCR[4] = 3000 - 1; //set duty at 5%
    return;
}
void pwmPortSetup(uint8_t pwmBit){
    P2->SEL0 |= pwmBit; //enable select pin for pwmBit
    P2->SEL1 &= ~pwmBit; //bit clear the pwmBit
    P2->DIR |= pwmBit; //set pwmBit as output pin
    P2->OUT |= pwmBit; //set pwmBit as high
    return;
}
