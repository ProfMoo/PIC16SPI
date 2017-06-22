/*
 * File:   main.c
 * Author: drmoo
 *
 * Created on June 14, 2017, 2:04 PM
 */
#include <xc.h>
#include <pic16f690.h>                       //PIC hardware mapping

// CONFIG
#pragma config FOSC = INTRCCLK  // Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

#define _XTAL_FREQ 4000000      // Required for _delay() function, internal OSC Max
                                // OSC cycle period of 0.5uS

void spiInit() {
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;
    TRISB4 = 1; // Serial Data IN: RB4 input, SDI (pin 13)
    TRISC7 = 0; // Serial Data OUT: RC7 output, SDO (pin 9)
    TRISB6 = 0; // Serial Clock OUT: RB6 output, SCK (pin 11) (0, Master), (1, Slave)
    TRISC6 = 0; // SS!, Active low, selects Slave (pin 8) (Slave this would be input, Master output)

    ADCON0 = 0b00000000;  // Disable ADC, make sure line as digital, required for SPI Lines to work
   
    SSPCONbits.SSPEN = 0; // Disable port to allow configuration
    SSPSTAT = 0b01000000; // 0 - BF<0>, Buffer Full Status bit, SSPBUFF is empty
                          // 1 - CKE<6>, CKP = 1, Data on falling edge of SCK
                          // 0 - SMP<7>, SPI slave, must be clear
    SSPCON = 0b00110010;  // 0010 - SSPM<3:0>, SPI Master mode, clock = FOSC/64
                          // 1 - CKP<4>, Clock Polarity Select Bit, Idle high state
                          // 1 - SSPEN<5>, Enable serial port and config SCK, SDO, SDI
                          // 0 - SSPOV<6>, Reset Synchronous Serial Port OVerflow, before sending
                          // 0 - WCOL<7>, Clear, No collisions
}

unsigned char spiWrite(unsigned char byte) {
    PORTCbits.RC6 = 0; // Active low, SS!, let the slave know that data is coming

    SSPBUF = byte; // Write to buffer to start a transmit
    while((SSPSTAT & (1<<0)) == 0); // Wait till buffer indicator says a receive has been completed
    return SSPBUF; // Read the buffer for the received byte.
}

void main(void) {
    spiInit();
    
    unsigned char foo = 0;
    unsigned char bar = 0;
    
//    foo = spiWrite(0x80);
//    foo = spiWrite(0x01);
//    PORTCbits.RC6 = 1;
//    foo = spiWrite(0x00);
//    foo = spiWrite(0x00);
//    PORTCbits.RC6 = 1;
    
    while(1) {
//        //foo = spiWrite(0x14);
//        foo = readReg();
//        PORTCbits.RC6 = 1;

        //shutdown mode
        foo = spiWrite(0x24);
        foo = spiWrite(0x01);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
//        //taken out of shutdown mode
//        foo = spiWrite(0x24);
//        foo = spiWrite(0x00);
//        PORTCbits.RC6 = 1;
//        foo = spiWrite(0x00);
//        foo = spiWrite(0x00);
//        PORTCbits.RC6 = 1;
//        __delay_us(200);
        
        //read control reg
        foo = spiWrite(0x20);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //change control register to 011
        foo = spiWrite(0x1C);
        foo = spiWrite(0x03);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //read control reg
        foo = spiWrite(0x20);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //read RDAC reg
        foo = spiWrite(0x08);
        //PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //change RDAC reg
        foo = spiWrite(0x85);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //read RDAC reg
        foo = spiWrite(0x08);
        //PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //software reset
        foo = spiWrite(0x10);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(200);
        
        //shutdown mode
        foo = spiWrite(0x24);
        foo = spiWrite(0x01);
        PORTCbits.RC6 = 1;
        foo = spiWrite(0x00);
        foo = spiWrite(0x00);
        PORTCbits.RC6 = 1;
        __delay_us(1000);
    }
}
