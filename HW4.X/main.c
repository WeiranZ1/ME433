#include <stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "spi.h"
#include <math.h>

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

// Function prototypes
void readUART1(char * string, int maxLength);
void writeUART1(const char * string);
unsigned short make16(char a_or_b, unsigned char v);
float sin_wave(float t);
float tri_wave(float t);
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
    TRISAbits.TRISA4 = 0;   // initializes A4 as output
    TRISBbits.TRISB4 = 1;   // initializes B4 as input
    LATAbits.LATA4 = 1;
    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX
    
    // turn on UART3 without an interrupt
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / 230400) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;

    // enable the uart
    U1MODEbits.ON = 1;

    __builtin_enable_interrupts();
    
    char m[100];
    
    // initialize SPI
    initSPI();
    float v;
    float t = 0.0;
    unsigned char response;
    unsigned short s;
   
    while (1) {

        // pick the voltage for sine and tri waves, an unsigned 8 bit number
//            unsigned char s = ; 
        // turn the voltage into a unsigned 16 bit number
        // 1st bit: A/B, next 3: 111, next 8 bits: voltage, last 4: don't matter
        v = sin_wave(t);
        s = make16(1, v);
        // send the sine wave by:
        LATAbits.LATA0 = 0;             // make cs low
        response = spi_io(s>>8);        // send 8 bits
        response = spi_io(s & 0xff);    // send 8 bits
        LATAbits.LATA0 = 1;             // make cs high
            
        // send the tri wave:
        v = tri_wave(t);
        s = make16(0, v);
        // make cs low
        LATAbits.LATA0 = 0;             // make cs low
        response = spi_io(s>>8);        // send 8 bits
        response = spi_io(s & 0xff);    // send 8 bits
        LATAbits.LATA0 = 1;             // make cs high

        //delay:
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 240000){} 
        t ++;
        if (t == 100){
            t = 0;
        }
    }

     
}

// turn voltage into an unsigned 16 bit number for SPI
unsigned short make16(char a_or_b, unsigned char v){
  unsigned short s;
  s = 0;
  s = s | (a_or_b<<15);
  s = s | (0b111<<12);
  s = s | (v << 4);
  return s;
}

float sin_wave(float t){
    return 127 + 127*sin(t/15.915);
}

float tri_wave(float t){
    if (t <= 50){
        return 255/50*t;
    }
    else{
        return 255-255/50*t;
    }
}