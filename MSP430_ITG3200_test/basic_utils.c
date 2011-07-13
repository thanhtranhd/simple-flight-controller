#include "msp430x22x4.h"
#include "basic_utils.h"

void set_8mhz_clock()
{
  BCSCTL1 = CALBC1_8MHZ; 
  DCOCTL = CALDCO_8MHZ;
  BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
}

void UART_init()
{
   P3SEL |= 0x30;                            // P3.4,5 = USCI_A0 TXD/RXD
   UCA0CTL1 = UCSSEL_2;                      // SMCLK
      
   /*
   UCA0BR0 = 142;                            // 57600 from 8Mhz
   UCA0BR1 = 0;
   */  
   UCA0BR0 = 0x41;                           // 9600 from 8Mhz
   UCA0BR1 = 0x3;
   /*  
   UCA0BR0 = 0x45;                           // 115200 from 8Mhz
   UCA0BR1 = 0x0;
   */
   UCA0MCTL = UCBRS_2;                       
   UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
   IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

void putc ( void* p, char c)
{
    UCA0TXBUF = c;
    while (!(IFG2&UCA0TXIFG));                 // USCI_A0 TX buffer ready?
}

		