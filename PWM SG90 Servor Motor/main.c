/*
 * JESUS MINJARES
 * BSEE with Computer Concentration
 * May 27, 2020
 *
 * Move a Servo Motor Sg90 with PWM at TA0.4
 *
 * Setup PWM for 2.7 to control the motor. SG90 operate at 20ms(50hz) from 1ms to 2ms, therefore it duty cycle must be
 * 5%-10%. After each pulse the motor must have a delay of 18-20ms.
 *
//                MSP432P401x
//             -----------------
//            |          0V(GND)|--> Brown Pin(SG90)
//            |                 |
//            |RST        5.0V  |--> Red Pin(SG90)
//            |                 |
//            |     TA0.4-> P2.7|-->  Orange Pin (SG90)
//            |            (PWM)|
//            |                 |
//            |_________________|
//
*/

#include "msp.h"

void timerSetup(); //setup timer to iterate every 20ms
void pwmSetup(); // setup PWM pin for SG90
void port2Setup(); //enable the pins for pwm
int tic = 0; //global variable to count clock cycles
void twoPulses(int delayMag); // iterate from 0 to 180 degrees
void fourPulses(int delayMag); // iterate every 45 degrees
void threePulses(int delayMag); //iterate every 90 degrees
void oneSweep(int delayMag); // 180 degrees
int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    port2Setup(); // setup for Port 2
    pwmSetup(); // pwm configuration of Timer A0
    timerSetup(); //timer setup of TIMER_A1 running at 50hz
    NVIC->ISER[0] = ( 1 <<( TA1_0_IRQn & 31)); //setup TA1 interript
    __enable_irq(); //enable global interrupt

    while(1){} //infinite loop
}

void port2Setup(){
        P2->SEL0 |= 0x80; // select BIT7 as PWM
        P2->SEL1 &= ~0x80; // bit clear BIT7
        P2->DIR |= 0x80; //set BIT7 as ouput
}
void pwmSetup()
{
    TIMER_A0->CTL = TIMER_A_CTL_MC_1 + TIMER_A_CTL_ID_0 + TIMER_A_CTL_TASSEL_2; // Up mode, Divide by 2^0, SMCLK-->3Mhz
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; // rest/set mode , always use for PWM
    TIMER_A0->CCR[0] = 60000 - 1; //set clock at 50 hz, 60k/3Mhz = .02(1/hz) -> 50hz
    TIMER_A0->CCR[4] = 3000 - 1; // set PWM at 5% duty cycle -----60000*(.05) = 3000
    return;
}
void timerSetup()
{
    TIMER_A1->CTL = TIMER_A_CTL_MC_1 + TIMER_A_CTL_ID_0 + TIMER_A_CTL_TASSEL_2; //set timer at up mode, divide by 1, smck at 3Mhz
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // enable capture & compare mode
    TIMER_A1->CCR[0] = 60000 - 1; // set timer at 50hz, the reason is that the SG90 needs 20ms pulse intervals
}
void TA1_0_IRQHandler()
{
    tic++; //increment tic
   // fourPulses(2);
    threePulses(2);
    //twoPulses(2);
    //oneSweep(2);

    TIMER_A1->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG); //clear flag for TIMERA1

}
void fourPulses(int delayMag)
{
        //every 10*20ms*magnitude will be the time delay
        // SG90 will move every 45 degrees
        if(tic == 10*delayMag)
          TIMER_A0->CCR[4] += 1000; // 1.66% increase
        if(tic == 20*delayMag)
          TIMER_A0->CCR[4] += 1000; // 1.66% increase
        if(tic == 30*delayMag)
          TIMER_A0->CCR[4] += 1000; // 1.66% increase
        if(tic == 40*delayMag)
            TIMER_A0->CCR[4] -=1000; // 1.66% decrease
        if(tic == 50*delayMag)
            TIMER_A0->CCR[4] -= 1000; // 1.66% decrease
        if(tic == 60*delayMag)
            TIMER_A0->CCR[4] -= 1000; // 1.66% decrease
        if(tic == 70*delayMag)
            tic = 0;
        return;
}
void twoPulses(int delayMag)
{
    //every 10*20ms*magnitude will be the time delay
    // SG90 will move 0-180 degrees
    if(tic == 10*delayMag)
        TIMER_A0->CCR[4] = 6000 - 1; // 5% increase
    if(tic == 20*delayMag)
        TIMER_A0->CCR[4] = 3000 - 1; // 5% decrease
    if(tic == 30*delayMag)
        tic = 0;
    return;
}
void threePulses(int delayMag)
{
    //every 10*20ms*magnitude will be the time delay
    //SG90 will iterate every 90 degrees
    if(tic == 10*delayMag)
        TIMER_A0->CCR[4] += 1500; // 2.5% increase
    if(tic == 20*delayMag)
        TIMER_A0->CCR[4] += 1500; // 2.5% incease
    if(tic == 30*delayMag)
        TIMER_A0->CCR[4] -= 1500; // 2.5% decrease
    if(tic == 40*delayMag)
        TIMER_A0->CCR[4] -= 1500; // 2.5% increase
    if(tic == 60*delayMag)
        tic = 0;
    return;
}
void oneSweep(int delayMag)
{
    //every 10*20ms*magnitude will be the time delay
    if(tic == 20*delayMag)
     TIMER_A0->CCR[4] = 6000 - 1; // set at 10%
    if(tic == 30*delayMag)
    {
        TIMER_A0->CCR[4] = 3000 - 1; // reset it a 5%
        tic = 0;
    }
    return;
}
