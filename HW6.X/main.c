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

// Function prototypes
void readUART1(char * string, int maxLength);
void writeUART1(const char * string);
unsigned short make16(char a_or_b, unsigned char v);
void blink();
void mcp23008_write(unsigned char ad, unsigned char reg, unsigned char val);
unsigned char  mcp23008_read(unsigned char ad, unsigned char reg);

#define ADRESS 0b01000000
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

    
    // HW6
    // turn on i2c
    i2c_master_setup();
    // init the mcp23008 with pg7 as out and gp0 as in

    mcp23008_write(ADRESS, IODIR , 0b01111111);
    
    // turn on gp7
    mcp23008_write(ADRESS, OLAT, 0b10000000);
    
    // read what pin are on
    unsigned char r = mcp23008_read(ADRESS, GPIO);
    
    // if gp0 is 0, turn on the led
    // r = 0b01XXXXXX
    //     0b00000001
    if ((r>>1) & 0b1 == 0b1){
        mcp23008_write(ADRESS, OLAT, 0b00000000);
    }
    
    
    char m[100];
    int t;
    sscanf(m,"%d",&t);
    
    
    __builtin_enable_interrupts();
    
    while (1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        if (PORTBbits.RB4 == 0){
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 12000000){
                LATAbits.LATA4 = 1; // turn on A4 for 0.5s, 
            }
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 12000000){
                LATAbits.LATA4 = 0; // off for 0.5s
            }
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 12000000){
                LATAbits.LATA4 = 1; // on for 0.5s
            }
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 12000000){
                LATAbits.LATA4 = 0; // off for 0.5s
            }
            sprintf(m,"Hello!\r\n");
            writeUART1(m);
          
            //terminal window   -   ls /dev/tty.* 
            //screen /dev/tty.usbserial-0232B0A1 230400 

            
        }
    }

     
}
// Read from UART3
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void readUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
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

// Write a character array using UART3
void writeUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
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
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1); //done
    i2c_master_stop();
}