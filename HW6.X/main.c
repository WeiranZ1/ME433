#include <stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "i2c_master_noint.H"

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


unsigned short make16(char a_or_b, unsigned char v);
void blink();
void mcp23008_write(unsigned char ad, unsigned char reg, unsigned char val);
unsigned char  mcp23008_read(unsigned char ad, unsigned char reg);

#define ADDRESS 0b0100000
#define IODIR 0x00
#define GPIO 0x09
#define OLAT 0x0A

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
    LATAbits.LATA4 = 0;
    
    // HW6
    // turn on i2c
    i2c_master_setup();
    // send start bit 
    // send the address with write bit
    // send the name of the reg to change IODIR
    // send 8 bits for IO
    // send stop bit
    
    // Init the MCP23000 with GP7 as out and GP0 as in
    mcp23008_write(ADDRESS, IODIR , 0b01111111);
//    
//    // turn on gp7
//    mcp23008_write(ADDRESS, OLAT, 0b10000000);
    
    // read what pin are on
//    unsigned char r = mcp23008_read(ADDRESS, GPIO);
    
    __builtin_enable_interrupts();
    unsigned char rcvd;
    while (1) {
        //BLINK
        LATAbits.LATA4 = !LATAbits.LATA4;
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 3000000) {}
        
        // read what pin are on
        rcvd = mcp23008_read(ADDRESS, GPIO);
        // if gp0 is 0, turn on the led
        if (rcvd & 0b1 == 0b1) {
            mcp23008_write(ADDRESS, OLAT, 0b10000000);
        }
        else {
            mcp23008_write(ADDRESS, OLAT, 0b00000000);
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


void mcp23008_write(unsigned char ad, unsigned char reg, unsigned char val){
    // send start bit
    // send the address with write bit
    // send the name of the register to change IODIR
    // send 8 bits for IO
    // send stop bit
    i2c_master_start();
    i2c_master_send(ad<<1);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}


unsigned char  mcp23008_read(unsigned char ad, unsigned char reg){
    i2c_master_start();
    i2c_master_send(ad<<1); //write
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(ad<<1 | 0b1); //read
    unsigned char rcvd = i2c_master_recv();
    i2c_master_ack(1); //done
    i2c_master_stop();
    return rcvd;
}