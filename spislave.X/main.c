/*
 * File:   main.c
 * Author: drmoo
 *
 * Created on June 16, 2017, 9:19 AM
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

#define _XTAL_FREQ 4000000      //required for _delay() function

void init_Slave_Output(void)
{
    // Explicitly Setup Output for LED's, bit 0 = output, 1 = input
    TRISAbits.TRISA2 = 0;  // Bit 7
    TRISCbits.TRISC0 = 0;  // Bit 6
    TRISCbits.TRISC1 = 0;  // Bit 5
    TRISCbits.TRISC2 = 0;  // Bit 4
    TRISBbits.TRISB5 = 0;  // Bit 3
    TRISBbits.TRISB7 = 0;  // Bit 2
    TRISCbits.TRISC3 = 0;  // Bit 1
    TRISCbits.TRISC4 = 0;  // Bit 0
}  // End of init_SPI_Slave()

void spiInit() {
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;
    TRISB4 = 1; // Serial Data IN: RB4 input, SDI (pin 13)
    TRISC7 = 0; // Serial Data OUT: RC7 output, SDO (pin 9)
    TRISB6 = 1; // Serial Clock OUT: RB6 output, SCK (pin 11) (0, Master), (1, Slave)
    TRISC6 = 1; // SS!, Active low, selects Slave (pin 8) (Slave this would be input, Master output)

    ADCON0 = 0b00000000;  // Disable ADC, make sure line as digital, required for SPI Lines to work
   
    SSPCONbits.SSPEN = 0; // Disable port to allow configuration
    SSPSTAT = 0b01000000; // 0 - BF<0>, Buffer Full Status bit, SSPBUFF is empty
                          // 1 - CKE<6>, CKP = 1, Data on falling edge of SCK
                          // 0 - SMP<7>, SPI Master Mode, Input data sample in middle of data output time
    SSPCON = 0b00110100;  // 0100 - SSPM<3:0>, SPI Slave, clock = SS! pin control
                          // 1 - CKP<4>, Clock Polarity Select Bit, Idle high state
                          // 1 - SSPEN<5>, Enable serial port and config SCK, SDO, SDI
                          // 0 - SSPOV<6>, Reset Synchronous Serial Port OVerflow, before sending
                          // 0 - WCOL<7>, Clear, No collisions
}

unsigned char spiWrite(unsigned char byte) {
    SSPBUF = byte; // Write to buffer to start a transmit
    while((SSPSTAT & (1<<0)) == 0); // Wait till buffer indicator says a receive has been completed
    return SSPBUF; // Read the buffer for the received byte.
}

void main(void) {
    unsigned char value = 0x02;     // Storage of master value
    unsigned char Dummy_Var = 0x03;

    spiInit();
    init_Slave_Output();

    while(1) {
        while (!SSPSTATbits.BF) {
            spiWrite(0x00);
        }
        while (!SSPSTATbits.BF) {
            spiWrite(0x00);
        }
        while (!SSPSTATbits.BF) {
            spiWrite(0xA3);
        }
    }
    
//    while(1) {
//        while (!SSPSTATbits.BF) {
//            spiWrite(0x00);
//            spiWrite(0xA3);
//        }
//    }
//        if (!SSPSTATbits.BF){  // if Buffer Full is '1' then the buffer is being filled, need to read
//            value = spiWrite(Dummy_Var);
//            PORTAbits.RA2 = (value >> 7)  & 1;  // Port A, Bit 7
//            PORTCbits.RC0 = (value >> 6) & 1;  // Port C, Bit 6
//            PORTCbits.RC1 = (value >> 5) & 1;  // Port C, Bit 5
//            PORTCbits.RC2 = (value >> 4) & 1;  // Port C, Bit 4
////            PORTBbits.RB5 = (value >> 3) & 1;  // Port B, Bit 3
////            PORTBbits.RB7 = (value >> 2) & 1;  // Port B, Bit 2
////            PORTCbits.RC3 = (value >> 1) & 1;  // Port C, Bit 1
////            PORTCbits.RC4 = (value >> 0) & 1;  // Port C, Bit 0
//            _delay(500);            // Output to LED's
//        }
}
