/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 25, 2020
 *
 * PWM with SysTick and TIMER_A0->CCR[4]
 *
 * Setup 2.7 PWM into 2.2 onboard led with PMAP configuration
 * Use SysTick timer running at 10ms to iterate over the LED duty cycle
 * Every half second increment the intensity
 *
//                MSP432P401x
//             -----------------
//            |                 |
//            |                 |
//            |RST              |
//            |                 |
//            |     TA0.4-> P2.2|-->  BLUE LED
//            |            (PWM)|
//            |                 |
//            |_________________|
//
*/
#include "msp.h"
int tic = 0; //globale variable to count total CPU clock cycles
void timerA0_Setup(); //timer A0 PWM setup
void port2ReMap(); // port 2 remap for PWM pins
void systickSetup(); //setup timer of the Systick
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	port2ReMap(); //call the function to setup reconfiguration for PWM
	timerA0_Setup(); //set up pwm setup for 2.7 which is TIMER_A0->CCR[4], look at the MSP pins
	systickSetup(); //setup systick at 10ms iterations
	__enable_irq(); //enable global interrupts
	while(1){} //infinite loop
}
void timerA0_Setup()
{
    TIMER_A0->CTL = TIMER_A_CTL_MC__UP | TIMER_A_CTL_ID_0 | TIMER_A_CTL_SSEL__SMCLK; //up-mode, timer divide by 1, SMCLK = DC0 = 3Mhz
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; //  reset/set, use OUTMODE 7 for pwm
    TIMER_A0->CCR[0] = 50000; //set PWM at 60hz , 3Mhz/50k = 60
    TIMER_A0->CCR[4] = 500; //set PWM at 1%
}
void port2ReMap()
{
    PMAP->KEYID = PMAP_KEYID_VAL; //unlock the PMAP to reconfigure the port
    P2MAP->PMAP_REGISTER2= PMAP_TA0CCR4A; //set up 2.2 to 2.7, or setup 2.2 to TIMER_A0->CCR[4]
    P2->SEL0 = 0x04; //setup BIT2 for pwm setup
    P2->SEL1 &= ~0x04; //bit clear the bit from sel1
    P2->DIR |= 0x04; //set BIT2 as output
    PMAP->CTL = PMAP_CTL_LOCKED; // lock PMAP
    PMAP->KEYID = 0; //clear the KEYID
}
void systickSetup()
{
    SysTick->CTRL = 0x07; //alway setup at 0x07 based on the data sheet of the timer, BIT0 enables, BIT1 NC, BIT2 set clock sources
    SysTick->LOAD = 300000 - 1; //load the max value that the timer will count, 3mhz/300k = 10, 1/10 = 10ms
    SysTick->VAL = 1; //se the value you want to start counting at
}

void SysTick_Handler(void)
{
    tic++; //increment after every time systick is trigger, every 10ms
    if(tic == 5) //every 50ms increment duty cycles
    {
        TIMER_A0->CCR[4] += 2500; //increment by 5% every half second
        if(TIMER_A0->CCR[4] > 50000) //if it exceeds 100% reset it at 1%
            TIMER_A0->CCR[4] = 500; //setup at 1%
        tic = 0; //reset the tic
    }
}
