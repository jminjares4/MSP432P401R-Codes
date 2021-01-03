/*
 * Author:  Jesus Minjares
 *
 * App:
 *          Software Integration of IPIA System. The system will run for 1 minute to gather data
 *          from the GPS, enough time for hot and cold start. Once the minute has passed, it will parse the
 *          GNRMC string into data structures: NMEA_GNRMC, UTC_TIME and COORDIANTE. THe UTC_TIME will be parse to
 *          initialize the system timer that hold its data on SysTime (Data structure to hold current time). The system will
 *          trigger an ADC sampling every 10ms. During the ADC sampling, it will send the coordinates, time stamp, adc raw reading and voltage.
 *          i.e
 *              latitude longitude time_stamp adc_raw_reading voltage
 *
 *
 *               ---------------------
 *            | GPS Module (SIM33EAU) | @ 9600
 *             -----------------------
 *
 *                MSP432P401x
 *             -----------------
 *            |          0V(GND)|--> GND
 *            |                 |
 *            |RST        3.3-5V|--> VCC
 *            |                 |
 *            |         P3.0(RX)|--> TX
 *            |         P3.6    |--> EN
 *            |                 |
 *            |_________________|
 *
 *             --------------
 *            | HC-05 Sensor |  @ 9600
 *             --------------
 *
 *      S: State
 *      RX: Receiver
 *      TX: Transmitter
 *      VCC: 3.3-5v
 *      GND: 0v
 *      EN: Set on high, for AT Mode
 *                                                -----------------
 *               MSP432P401x                     |      HC-05      |
 *            -----------------                   S RX TX VCC GND EN
 *           |          0V(GND)|-->HC-05 GND
 *           |                 |
 *           |RST         VCC  |-->HC-05 VCC
 *           |                 |
 *           |         P3.2(RX)|-->HC-05 TX
 *           |         P3.3(TX)|-- 1K-.<--HC-05 RX
 *HC-05 EN<--|P2.3             |      |
 *           |_________________|      2k
 *                                    |
 *                                    GND
 *
 *             -----------------
 *            | Pressure Sensor |
 *             -----------------
 *               MSP432P401x
 *             -----------------
 *            |                 |
 *            |                 |
 *            |          (P5.4) |--> 10k Pot
 *            |                 |
 *            |                 |
 *            |                 |
 *            |                 |
 *            |_________________|
 *
*/
#include "msp.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "GPS.h"

/*Port Remap */
void portRemap(); //reconfigure UART1 pin P2.2 -> P3.0 for a more practical use

/*Clock*/
void set3Mhz(); //set the DC0 @ 3Mhz

/*GPS*/
void gpsUARTSetup(); //enable P3.0 to read GPS via Uart @ 9600
void gpsToggleSetup(uint8_t bit); //set P3.6 as an output
void turnOnGps(uint8_t bit); //bit set P3.6 to enable GPS
void turnOffGps(uint8_t bit); //bit clear P3.6 to disable GPS

/*adc*/
void adcSetup(); //enable ADC single channel A0
void port5Setup(uint8_t bit); //enable P5.4 to read analog data from the sensor

/*Addition functions*/
char * getTime(); //return the SysTime data structure as a string
char *getCoordinate(); //return the gpsCoordinate as a string
bool isSubstring(char *str, char *substr); //check if the substring exist in the string
uint8_t str_to_uint8(char *str);  //convert a string to an unsigned int of 8 bytes  (0-128)
uint16_t str_to_uint16(char *str); //convert a string to an unsigned int of 16 bytes (0-1024)

/*bluetooth*/
void sendBluetooth(char *message); //send message via bluetooth
void bluetoothSetup(); //initialize UART2/ bluetooth , P3.2

/*Putty*/
void sendPutty(char *message); //send messaga via serial port/USB using UART0
void puttySetup(); //enable PcEcho

/*Timer A2*/
void timerA2Setup(); //set timerA2 @ 10ms intervals
SysTime setSysTime(const UTC_TIME  *gpsUtcTime); //set sysTime data structure using UTC_TIME data structure

/*Data Structures*/
NMEA_GNRMC gps; //hold GNMRC parsed
COORDINATE gpsCoordinates; //hold latitude and longitude in degrees
UTC_TIME gpsTime; //hold the utc time
/*Data structure to hold current time */
typedef struct SysTime{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t milli;
}SysTime;
SysTime sysTime; //create an instance of the sysTime data structure to hold current time

/*Global variables*/
char *gpsUartStr = "GNRMC"; //string that is going to search
const char carriageReturn = 13; //  '\r'
const char newLineFeed = 10;   //   '\n'
int buffer_index = 0;
char buffer[82]; //82 bytes long as the NMEA is the max size of a string

void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    /*Initialize Clock at 3Mhz*/
    set3Mhz();
    /*initialize putty for debugging*/
    puttySetup();
    /*initialize gpsUart to get data*/
    gpsUARTSetup();
    /*enable ENable pin for the GPS*/
    gpsToggleSetup(BIT6);

    /*enable bluetooth*/
    bluetoothSetup();

    /*wait for the cold start of the GPS, 15 cold start...wait a minute to get stable data */
    int delay;
    for(delay = 0; delay < 30; delay++) //iterate 30 times
        __delay_cycles(6000000);//2 sec delay


    /*store GNMRC string into the NMEA_GNRMC structure*/
    gps = setGnrmc(buffer);
    /*store the GPS Coordinates in the COORDINATE Structure*/
    gpsCoordinates = setCoordinates(&gps);
    /*store the UTC time to the UTC_TIME Structure*/
    gpsTime = setTime(gps.utc_time);
    /*Store to the UTC-TIME into system timer, to time stamp data*/
    sysTime = setSysTime(&gpsTime);

    /*print GNRMC, Coordinates, and Time for debugging purposes*/

    //print_NMEA_GNRMC(&gps);
    //print_COORDINATE(&gpsCoordinates);
    //print_UTC_TIME(&gpsTime);


    /*enable 5.4 for ADC*/
    port5Setup(BIT4);

    /*enable TimerA2 at 10ms*/
    timerA2Setup();
    /*enable single channel repeat at 100Hz*/
    adcSetup();

    /*enable global interrupts*/
    __enable_irq();

    /*infinite loop
     * fix: replace with low power mode
     * */
    while(1){}

}
/**
 * @param   UTC_TIME data structure
 * @return  an instance of SysTime
 */
SysTime setSysTime(const UTC_TIME  *gpsUtcTime){
    SysTime tempSysTime;
    tempSysTime.hour = str_to_uint8((char*)gpsUtcTime->hour);
    tempSysTime.minute = str_to_uint8((char*)gpsUtcTime->minute);
    tempSysTime.second = str_to_uint8((char *)gpsUtcTime->second);
    tempSysTime.milli = str_to_uint16((char*)gpsUtcTime->milli);
    return tempSysTime;
}
/**
 * @param   string to be converted
 * @return  uin8_t
 */
uint8_t str_to_uint8(char *str){
    uint8_t result;
    uint8_t puiss;
    result = 0;
    puiss = 1;
    while (('-' == (*str)) || ((*str) == '+')){
        if (*str == '-')
            puiss = puiss * -1;
        str++;
    }
    while ((*str >= '0') && (*str <= '9')){
        result = (result * 10) + ((*str) - '0');
        str++;
    }
    return (result * puiss);
}
/**
 * @param   string to be converted
 * @return  uin16_t
 */
uint16_t str_to_uint16(char *str){
    uint16_t result;
    uint16_t puiss;
    result = 0;
    puiss = 1;
    while (('-' == (*str)) || ((*str) == '+')){
        if (*str == '-')
            puiss = puiss * -1;
        str++;
    }
    while ((*str >= '0') && (*str <= '9')){
        result = (result * 10) + ((*str) - '0');
        str++;
    }
    return (result * puiss);
}
/**
 * @param   Bit to set
 * @return  None
 */
void turnOnGps(uint8_t bit){
    P3->OUT |= bit;
}
/**
 * @param   Bit to cleared
 * @return  None
 */
void turnOffGps(uint8_t bit){
    P3->OUT &= ~bit;
}
/**
 * @param   P3.X BIT
 * @return  None
 * @note    Please set BIT6 as that the purpose of the method
 */
void gpsToggleSetup(uint8_t bit){
    P3->DIR |= bit;
    P3->OUT |= bit;
    P3->SEL0 &= ~bit;
    P3->SEL1 &= ~bit;
}
/**
 * @param   string to be sent
 * @return  None
 */
void sendPutty(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF = message[i];
    }
    return;
}
/**
 * @param   None
 * @return  None
 * @note    portRemap will set 3.0 to point to 2.2 to use UART1 to
 *          a more manageable pin.
 */
void portRemap(){
    PMAP->KEYID = PMAP_KEYID_VAL; //unlock the PMAP to reconfigure the port
    P3MAP->PMAP_REGISTER0 = PMAP_UCA1RXD; //set 2.2 RX  to
    P3->SEL0 |= BIT0;// set 2-UART pin as secondary function
    P3->SEL1 &= ~(BIT0);// Configure UART
    P3->DIR |= BIT0; //set bit as output
    PMAP->CTL = PMAP_CTL_LOCKED; // lock PMAP
    PMAP->KEYID = 0; //clear the KEYID
}
/**
 * @param   None
 * @return  None
 * @note    gpsUARTSetup() will set UART1 using P3.0 as RX.
 *          9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void gpsUARTSetup(){
    portRemap();
    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A1->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A1->BRW = 19;                     // 3000000/16/9600  = 19.53125
    EUSCI_A1->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A1->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A1->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA1_IRQn) & 31);
}
/**
 * @param   None
 * @return  None
 * @note    puttSetup() set UART0 to send data via serial port/usb
 *          9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void puttySetup(){
    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P1->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A0->BRW = 19;                      // 3000000/16/9600  = 19.53125
    EUSCI_A0->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}
/**
 * GPS UART Interrupt Subroutine
 * note:    The interrupt will capture the data from
 *          the GPS and store it in buffer.
 */
void EUSCIA1_IRQHandler(void){
    static uint8_t gpsColdStart = 0; //create a static variable
    if (EUSCI_A1->IFG & EUSCI_A_IFG_RXIFG){ //check the flag
        buffer[buffer_index++] = EUSCI_A1->RXBUF; //store character, increment index
        /*the gps will end with \r\n\0, so check if the last 3 character match */
        if(buffer[buffer_index-2] == carriageReturn && buffer[buffer_index - 1] == newLineFeed  && buffer[buffer_index] == '\0'){
            if(isSubstring((char*)buffer,gpsUartStr) && gpsColdStart++ < 50){ //check if gpsUartStr is a substring and if gpsColdStart is < 50
                sendBluetooth(buffer); //send via bluetooth
            }
            if(gpsColdStart < 50){ //if gpsColdStart still less than 50
                memset(&buffer,0,sizeof(buffer)); //clear the buffer
                buffer_index = 0; //reset index
            }
            else{
                turnOffGps(BIT6); //turn off GPS via P3.6 (EN) Pin
                gpsColdStart = 0; //reset gpsColdStart
            }
        }
    }
}
/**
 * @param   main string, substring
 * @return  return (substring in str) ? true : false
 * @note    use "stdbool" to use bool
 */
bool isSubstring(char *str, char *substr){
    int i;
    int len = strlen(str);
    int len2 = strlen(substr);
    for(i = 0; i < len; i++){
        if(str[i] == substr[0]){
            int j;
            int temp = i;
            for(j = 0; j < len2; j++){
                if(substr[j] == str[temp++])
                    continue;
                else{
                    break;
                }
            }
            if(temp - i == strlen(substr)){
                return true;
            }
        }
    }
    return false;
}
/**
 * @param   None
 * @return  None
 * @note    set3Mhz will set the main clock at 3Mhz
 */
void set3Mhz(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 |CS_CTL1_SELM_3; // Select ACLK = REFO// MCLK = DCO // SMCLK = DCO
    CS->KEY = 0; //lock CS module
}
/**
 * @param   None
 * @return  char * that will contain the time as a string
 * @note    getTime() converts the SysTime data structure as string
 */
char *getTime(){
   static char localTime[24]; //create a static buffer
   sprintf(localTime,"%u:%u:%u.%u ", sysTime.hour, sysTime.minute, sysTime.second, sysTime.milli); //use sprintf to print to the buffer
   return localTime; //return the time
}
/**
 * @param   None
 * @return  char * that will contain the time as a string
 * @note    getCoordinate() converts the COORDINATE data structure as string
 */
char *getCoordinate(){
    static char localCoordinate[48]; //create a static buffer
    sprintf(localCoordinate,"%s,%s ", gpsCoordinates.latitude,gpsCoordinates.longitude); //use sprintf to print into the buffer
    return localCoordinate; //return coordinates
}
/**
 * @param   None
 * @return  None
 * @note    set A1 to sample every 100hz
 */
void adcSetup(){
    // set as sample trigger, predivde clock by 64, divide by 5, sample hold time of 14, adc on
    ADC14->CTL0 =  ADC14_CTL0_SHP| ADC14_CTL0_ON | ADC14_CTL0_SHT0__192; // 3mhz/64/6/(64 + 14+2) == 97. hertz
    ADC14->CTL1 = ADC14_CTL1_RES_3;         // Use sampling timer, 14-bit conversion results
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt>
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31); //enable adc
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //enable conversion and start conversion
}
/**
 * @param   uint8_t
 * @return  None
 * @note    set any port 5 pin for ADC sampling
 */
void port5Setup(uint8_t bit){
    P5->SEL1 |= bit;   // Configure P5.4 for ADC
    P5->SEL0 |= bit;  // Confiugre P5.4 for ADC
    P5->DIR &= ~(bit); //bit clear bit4
    P5->OUT &= ~(bit); // bit clear bit4
}
/**
 * @param   None
 * @return  None
 * @note    set TimerA2 @ intervals of 1ms
 */
void timerA2Setup(){
    TIMER_A2->CCTL[0] = TIMER_A_CCTLN_CCIE; // capture and compare interrupt enable
    TIMER_A2->CCR[0] = 3000; //set limit at 30000, 3Mhz/30000, every 100 tics will be a 1 sec
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 & ~TIMER_A_CTL_MC__UP; //set SMCLK at 3mhz and upmode, upmode will count from 0->CCR[0]
    NVIC->ISER[0] = 1 << ((TA2_0_IRQn) & 31);
    TIMER_A2->CTL |= TIMER_A_CTL_MC__UP; //enable timer
    return;
}
/*
 * TIMERA2 Interrupt Subroutine
 * @note    TIMERA2 will be trigger every 1ms and
 *          update the current time for the sysTime.
 *          Will trigger ADC every 10 ms
 *
 */
void TA2_0_IRQHandler(){
    sysTime.milli++; //incrmement every TA2 interval
    if(sysTime.milli >= 999){ sysTime.second++; sysTime.milli = 0;  }
    if(sysTime.second >= 59){ sysTime.minute++; sysTime.second = 0; }
    if(sysTime.minute >= 59){ sysTime.hour++;   sysTime.minute = 0; }
    if(sysTime.hour >= 23){   sysTime.hour = 0;}
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear the capture and compare flag of the timer
    if(sysTime.milli % 10 == 0)
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //enable sample and start conversion
}
/**
 * ADC Interrupt Subroutine
 * @note    Get ADC reading every 10ms, and time stamp stamp the data
 *          send data via bluetooth once it done capturing
 */
void ADC14_IRQHandler(void){
    uint32_t adcRawReading = ADC14->MEM[0]; //save adc reading
    char tempBuffer[48] = ""; //create an empty buffer
    char *time = getTime(); //store time

    sprintf(tempBuffer,"%s %s ",gpsCoordinates.latitude, gpsCoordinates.longitude);
    sendBluetooth(tempBuffer);

    sendBluetooth(time);
    sprintf(tempBuffer, "%u %lf\r\n",adcRawReading, (3.3/16384 * adcRawReading)); // output the message of adc reading
    sendBluetooth(tempBuffer); //send message
}
/**
 * @param   None
 * @return  None
 * @note    bluetoothSetup() will set UART2
 *          9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void bluetoothSetup(){
    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P3->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A2->BRW = 19;   // 3000000/16/9600  = 19.53125
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}
/**
 * @param   message(string) to sent via bluetooth
 * @return  None
 * @note    9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void sendBluetooth(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){
     while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A2->TXBUF = message[i];
    }
    return;
}
