// TECH CUP //

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

#include "i2c_master_noint.h"
#include "ssd1306.h"
#include "font.h"

#define PIC32_SYS_FREQ 48000000ul // 48 million Hz
#define PIC32_DESIRED_BAUD 230400 // Baudrate for RS232
void UART2_Startup(void);
void ReadUART2(char * string, int maxLength);
void WriteUART2(const char * string);

#define MAXSPEED 50.0

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    TRISBbits.TRISB4 = 1;

    i2c_master_setup();
    ssd1306_setup();

    // UART to the Pico
    U2RXRbits.U2RXR = 0b0011; // Set B11 to U2RX
    RPB10Rbits.RPB10R = 0b0010; // Set B10 to U2TX
    UART2_Startup();

    // Motor pins
    RPB15Rbits.RPB15R = 0b0101; // B15 is OC1
    TRISBbits.TRISB14 = 0; // B14 is the direction pin
    LATBbits.LATB14 = 0;
    RPB13Rbits.RPB13R = 0b0110; // B13 is OC5
    TRISBbits.TRISB12 = 0; // B12 is direction pin
    LATBbits.LATB12 = 0;
    // servo pin
    RPA1Rbits.RPA1R = 0b0101; // A1 is OC2 for servo
    // PWM for motors
    T2CONbits.TCKPS = 2;    // Timer2 prescaler N=4 (1:4)
    PR2 = 599;             // period = (PR2+1) * N * (1/48000000) = 50 us, 20 kHz
    TMR2 = 0;               // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin
    OC1RS = 0;            // duty cycle
    OC1R = 0;             // initialize before turning OC1 on; afterward it is read-only
    OC5CONbits.OCM = 0b110; // PWM mode without fault pin
    OC5RS = 0;            // duty cycle
    OC5R = 0;             // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1;
    OC1CONbits.ON = 1;
    OC5CONbits.ON = 1;
    // PWM for servo
    T3CONbits.TCKPS = 0b100;    // prescaler of 16
    PR3 = 59999;             // 50Hz
    TMR3 = 0;               // initial TMR2 count is 0
    OC2CONbits.OCTSEL = 1; // use timer3
    OC2CONbits.OCM = 0b110; // PWM mode without fault pin
    OC2RS = 0;            // duty cycle
    OC2R = 0;             // initialize before turning OC1 on; afterward it is read-only
    T3CONbits.ON = 1;
    OC2CONbits.ON = 1;

    __builtin_enable_interrupts();

    char message[30];
    ssd1306_clear();
    sprintf(message, "No message yet");
    drawString(message, 5,24);
    ssd1306_update();

    while (1) {
        char m[25];
        ReadUART2(m,25); // get the number from the Pico
        int t = _CP0_GET_COUNT();
        float pos = 0;
        sscanf(m,"%f",&pos);

        float leftSpeed = 0;
        float rightSpeed = 0;
        // set the motor speed to follow the line
        if (pos < 15){
            rightSpeed = MAXSPEED;
            leftSpeed = pos*MAXSPEED/15.0;
        }
        else if (pos > 15){
            rightSpeed = MAXSPEED - (pos-15)*MAXSPEED/15;
            leftSpeed = MAXSPEED;
        }
        else if (pos == 15){
            rightSpeed = MAXSPEED;
            leftSpeed = MAXSPEED;
        }
        else {
            rightSpeed = 0;
            leftSpeed = 0;
        }
        // make sure the desired PWM is in the allowable range -100 to +100
        if(rightSpeed>100){
            rightSpeed = 100;
        }
        if(leftSpeed>100){
            leftSpeed = 100;
        }

        if(rightSpeed < -100){
            rightSpeed = -100;
        }
        if(leftSpeed < -100){
            leftSpeed = -100;
        }

        // set the PWM duty
        if (leftSpeed > 0){
            OC5RS = leftSpeed*599.0/100.0;
            LATBbits.LATB12 = 1;
        }
        else {
            leftSpeed = -leftSpeed;
            OC5RS = leftSpeed*599.0/100.0;
            LATBbits.LATB12 = 0;
        }
        if (rightSpeed > 0){
            OC1RS = rightSpeed*599.0/100.0;
            LATBbits.LATB14 = 1;
        }
        else {
            rightSpeed = -rightSpeed;
            OC1RS = rightSpeed*599.0/100.0;
            LATBbits.LATB14 = 0;
        }

        // servo
        OC2RS = 9000; // max 0-59999, for 1ms-2ms then 5999-11998

        sprintf(message, "t=%f pos=%6.4f", t/24000000.0, pos);
        drawString(message, 5,16);
        sprintf(message, "L=%6.4f R=%6.4f", leftSpeed, rightSpeed);
        drawString(message, 5,24);
        ssd1306_update();

        _CP0_SET_COUNT(0);
    }
}

// Read from UART1
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void ReadUART2(char * message, int maxLength) {
    char data = 0;
    int complete = 0, num_bytes = 0;
    // loop until you get a '\r' or '\n'
    while (!complete) {
        if (U2STAbits.URXDA) { // if data is available
            data = U2RXREG;      // read the data
            if ((data == '\n') || (data == '\r')) {
                complete = 1;
            } else {
                message[num_bytes] = data;
                ++num_bytes;
                // roll over if the array is too small
                if (num_bytes >= maxLength) {
                    num_bytes = 0;
                }
            }
        }
    }
    // end the string
    message[num_bytes] = '\0';
}

// Write a character array using UART1
void WriteUART2(const char * string) {
    while (*string != '\0') {
        while (U2STAbits.UTXBF) {
            ; // wait until tx buffer isn't full
        }
        U2TXREG = *string;
        ++string;
    }
}


void UART2_Startup() {
    // disable interrupts
    __builtin_disable_interrupts();

    // turn on UART1 without an interrupt
    U2MODEbits.BRGH = 0; // set baud to PIC32_DESIRED_BAUD
    U2BRG = ((PIC32_SYS_FREQ / PIC32_DESIRED_BAUD) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U2MODEbits.PDSEL = 0;
    U2MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;

    // enable the uart
    U2MODEbits.ON = 1;

    __builtin_enable_interrupts();
}