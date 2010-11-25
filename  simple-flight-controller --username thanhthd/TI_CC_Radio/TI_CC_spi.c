//----------------------------------------------------------------------------
//  Description:  This file contains functions that allow the MSP430 device to 
//  access the SPI interface of the CC1100/CC2500.  There are multiple 
//  instances of each function; the one to be compiled is selected by the 
//  system variable TI_CC_RF_SER_INTF, defined in "TI_CC_hardware_board.h".
//
//  MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//----------------------------------------------------------------------------


#include "include.h"
#include "TI_CC_spi.h"


//----------------------------------------------------------------------------
//  void TI_CC_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//----------------------------------------------------------------------------
//  void TI_CC_SPIWriteReg(char addr, char value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//----------------------------------------------------------------------------
//  void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//----------------------------------------------------------------------------
//  char TI_CC_SPIReadReg(char addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//----------------------------------------------------------------------------
//  void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//----------------------------------------------------------------------------
//  char TI_CC_SPIReadStatus(char addr)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//----------------------------------------------------------------------------
//  void TI_CC_SPIStrobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//----------------------------------------------------------------------------


// Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
// it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
// delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}


// SPI port functions
#if TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USART0


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  ME1 |= USPIE0;                            // Enable USART0 SPI mode
  UCTL0 |= CHAR + SYNC + MM;                // 8-bit SPI Master **SWRST**
  UTCTL0 |= CKPH + SSEL1 + SSEL0 + STC;     // SMCLK, 3-pin mode
  UBR00 = 0x02;                             // UCLK/2
  UBR10 = 0x00;                             // 0
  UMCTL0 = 0x00;                            // No modulation
  TI_CC_SPI_USART0_PxSEL |= TI_CC_SPI_USART0_SIMO | TI_CC_SPI_USART0_SOMI | TI_CC_SPI_USART0_UCLK;
                                            // SPI option select
  TI_CC_SPI_USART0_PxDIR |= TI_CC_SPI_USART0_SIMO + TI_CC_SPI_USART0_UCLK;
                                            // SPI TX out direction
  UCTL0 &= ~SWRST;                          // Initialize USART state machine
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
    IFG1 &= ~URXIFG0;                       // Clear flag from first dummy byte
    U0TXBUF = addr;                         // Send address
    while (!(IFG1&URXIFG0));                // Wait for TX to finish
    IFG1 &= ~URXIFG0;                       // Clear flag from first dummy byte
    U0TXBUF = value;                        // Send value
    while (!(IFG1&URXIFG0));                // Wait for end of data TX
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    char i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
    U0TXBUF = addr | TI_CCxxx0_WRITE_BURST; // Send address
    while (!(IFG1&UTXIFG0));                // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      U0TXBUF = buffer[i];                  // Send data
      while (!(IFG1&UTXIFG0));              // Wait for TX to finish
    }
    IFG1 &= ~URXIFG0;
    while(!(IFG1&URXIFG0));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
  U0TXBUF = (addr | TI_CCxxx0_READ_SINGLE); // Send address
  while (!(IFG1&URXIFG0));                  // Wait for TX to finish
  IFG1 &= ~URXIFG0;                         // Clear flag set during last write
  U0TXBUF = 0;                              // Dummy write so we can read data
  while (!(IFG1&URXIFG0));                  // Wait for RX to finish
  x = U0RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
  IFG1 &= ~URXIFG0;                         // Clear flag
  U0TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(IFG1&UTXIFG0));                  // Wait for TXBUF ready
  U0TXBUF = 0;                              // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG1&URXIFG0));                  // Wait for end of addr byte TX
  IFG1 &= ~URXIFG0;                         // Clear flag
  while (!(IFG1&URXIFG0));                  // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    U0TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = U0RXBUF;                    // Store data from last data RX
    while (!(IFG1&URXIFG0));                // Wait for end of data RX
  }
  buffer[count-1] = U0RXBUF;                // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART0_PxIN & TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
  IFG1 &= ~URXIFG0;                         // Clear flag set during last write
  U0TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(IFG1&URXIFG0));                  // Wait for TX to finish
  IFG1 &= ~URXIFG0;                         // Clear flag set during last write
  U0TXBUF = 0;                              // Dummy write so we can read data
  while (!(IFG1&URXIFG0));                  // Wait for RX to finish
  x = U0RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
  U0TXBUF = strobe;                         // Send strobe
  // Strobe addr is now being TX'ed
  IFG1 &= ~URXIFG0;                         // Clear flag
  while (!(IFG1&URXIFG0));                  // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);// Wait for CCxxxx ready
  U0TXBUF = TI_CCxxx0_SRES;                 // Send strobe
  // Strobe addr is now being TX'ed
  IFG1 &= ~URXIFG0;                         // Clear flag
  while (!(IFG1&URXIFG0));                  // Wait for end of addr TX
  while (TI_CC_SPI_USART0_PxIN&TI_CC_SPI_USART0_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USART1


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  ME2 |= USPIE1;                            // Enable USART1 SPI mode
  UCTL1 |= CHAR + SYNC + MM;                // 8-bit SPI Master **SWRST**
  UTCTL1 |= CKPL + SSEL1 + SSEL0 + STC;     // SMCLK, 3-pin mode
  UBR01 = 0x02;                             // UCLK/2
  UBR11 = 0x00;                             // 0
  UMCTL1 = 0x00;                            // No modulation
  TI_CC_SPI_USART1_PxSEL |= TI_CC_SPI_USART1_SIMO | TI_CC_SPI_USART1_SOMI | TI_CC_SPI_USART1_UCLK;
                                            // SPI option select
  TI_CC_SPI_USART1_PxDIR |= TI_CC_SPI_USART1_SIMO + TI_CC_SPI_USART1_UCLK;
                                            // SPI TXD out direction
  UCTL1 &= ~SWRST;                          // Initialize USART state machine
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~URXIFG1;                       // Clear flag
    U1TXBUF = addr;                         // Send address
    while (!(IFG2&URXIFG1));                // Wait for TX to finish
    IFG2 &= ~URXIFG1;                       // Clear flag
    U1TXBUF = value;                        // Load data for TX after addr
    while (!(IFG2&URXIFG1));                // Wait for end of addr TX
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    char i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
    U1TXBUF = addr | TI_CCxxx0_WRITE_BURST; // Send address
    while (!(IFG2&UTXIFG1));                // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      U1TXBUF = buffer[i];                  // Send data
      while (!(IFG2&UTXIFG1));              // Wait for TX to finish
    }
    IFG2 &= ~URXIFG1;
    while(!(IFG2&URXIFG1));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~URXIFG1;                         // Clear flag set during addr TX
  U1TXBUF = (addr | TI_CCxxx0_READ_SINGLE); // Send address
  while (!(IFG2&URXIFG1));                  // Wait for TXBUF ready
  IFG2 &= ~URXIFG1;                         // Clear flag set during addr TX
  U1TXBUF = 0;                              // Load dummy byte for TX after addr
  while (!(IFG2&URXIFG1));                  // Wait for end of dummy byte TX
  x = U1RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~URXIFG1;                         // Clear flag
  U1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(IFG2&UTXIFG1));                  // Wait for TXBUF ready
  U1TXBUF = 0;                              // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG2&URXIFG1));                  // Wait for end of addr byte TX
  IFG2 &= ~URXIFG1;                         // Clear flag
  while (!(IFG2&URXIFG1));                  // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    U1TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = U1RXBUF;                    // Store data from last data RX
    while (!(IFG2&URXIFG1));                // Wait for end of data RX
  }
  buffer[count-1] = U1RXBUF;                // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~URXIFG1;                         // Clear flag set during last write
  U1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(IFG2&URXIFG1));                  // Wait for TX to finish
  IFG2 &= ~URXIFG1;                         // Clear flag set during last write
  U1TXBUF = 0;                              // Dummy write so we can read data
  while (!(IFG2&URXIFG1));                  // Wait for RX to finish
  x = U1RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  U1TXBUF = strobe;                         // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~URXIFG1;                         // Clear flag
  while (!(IFG2&URXIFG1));                  // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  U1TXBUF = TI_CCxxx0_SRES;                 // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~URXIFG1;                         // Clear flag
  while (!(IFG2&URXIFG1));                  // Wait for end of addr TX
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USCIA0


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  UCA0CTL0 |= UCMST+UCCKPL+UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 |= 0x02;                          // UCLK/2
  UCA0BR1 = 0;
  UCA0MCTL = 0;
  TI_CC_SPI_USCIA0_PxSEL |= TI_CC_SPI_USCIA0_SIMO | TI_CC_SPI_USCIA0_SOMI | TI_CC_SPI_USCIA0_UCLK;
                                            // SPI option select
  TI_CC_SPI_USCIA0_PxDIR |= TI_CC_SPI_USCIA0_SIMO | TI_CC_SPI_USCIA0_UCLK;
                                            // SPI TXD out direction
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCA0RXIFG;                     // Clear flag
    UCA0TXBUF = addr;                       // Send address
    while (!(IFG2&UCA0RXIFG));              // Wait for TX to finish
    IFG2 &= ~UCA0RXIFG;                     // Clear flag
    UCA0TXBUF = value;                      // Send data
    while (!(IFG2&UCA0RXIFG));              // Wait for TX to finish
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    unsigned int i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCA0RXIFG;
    UCA0TXBUF = addr | TI_CCxxx0_WRITE_BURST;// Send address
    while (!(IFG2&UCA0RXIFG));              // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      IFG2 &= ~UCA0RXIFG;
      UCA0TXBUF = buffer[i];                // Send data
      while (!(IFG2&UCA0RXIFG));            // Wait for TX to finish
    }
    //while (!(IFG2&UCA0RXIFG));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCA0TXIFG));                // Wait for TX to finish
  UCA0TXBUF = (addr | TI_CCxxx0_READ_SINGLE);// Send address
  while (!(IFG2&UCA0TXIFG));                // Wait for TX to finish
  UCA0TXBUF = 0;                            // Dummy write so we can read data
  // Address is now being TX'ed, with dummy byte waiting in TXBUF...
  while (!(IFG2&UCA0RXIFG));                // Wait for RX to finish
  // Dummy byte RX'ed during addr TX now in RXBUF
  IFG2 &= ~UCA0RXIFG;                       // Clear flag set during addr write
  while (!(IFG2&UCA0RXIFG));                // Wait for end of dummy byte TX
  // Data byte RX'ed during dummy byte write is now in RXBUF
  x = UCA0RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCA0RXIFG;                       // Clear flag
  UCA0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCA0TXIFG));                // Wait for TXBUF ready
  UCA0TXBUF = 0;                            // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG2&UCA0RXIFG));                // Wait for end of addr byte TX
  IFG2 &= ~UCA0RXIFG;                       // Clear flag
  while (!(IFG2&UCA0RXIFG));                // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCA0TXBUF = 0;                          //Initiate next data RX, meanwhile..
    buffer[i] = UCA0RXBUF;                  // Store data from last data RX
    while (!(IFG2&UCA0RXIFG));              // Wait for RX to finish
  }
  buffer[count-1] = UCA0RXBUF;              // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA0_PxIN & TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCA0RXIFG;                       // Clear flag set during last write
  UCA0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCA0RXIFG));                // Wait for TX to finish
  IFG2 &= ~UCA0RXIFG;                       // Clear flag set during last write
  UCA0TXBUF = 0;                            // Dummy write so we can read data
  while (!(IFG2&UCA0RXIFG));                // Wait for RX to finish
  x = UCA0RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  IFG2 &= ~UCA0RXIFG;                       // Clear flag
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
  UCA0TXBUF = strobe;                       // Send strobe
  // Strobe addr is now being TX'ed
  while (!(IFG2&UCA0RXIFG));                // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);// Wait for CCxxxx ready
  UCA0TXBUF = TI_CCxxx0_SRES;               // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~UCA0RXIFG;                       // Clear flag
  while (!(IFG2&UCA0RXIFG));                // Wait for end of addr TX
  while (TI_CC_SPI_USCIA0_PxIN&TI_CC_SPI_USCIA0_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USCIA1


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  UCA1CTL0 |= UCMST+UCCKPL+UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
  UCA1CTL1 |= UCSSEL_2;                     // SMCLK
  UCA1BR0 |= 0x02;                          // UCLK/2
  UCA1BR1 = 0;
  UCA1MCTL = 0;
  TI_CC_SPI_USCIA1_PxSEL |= TI_CC_SPI_USCIA1_SIMO | TI_CC_SPI_USCIA1_SOMI | TI_CC_SPI_USCIA1_UCLK;
                                            // SPI option select
  TI_CC_SPI_USCIA1_PxDIR |= TI_CC_SPI_USCIA1_SIMO | TI_CC_SPI_USCIA1_UCLK;
                                            // SPI TXD out direction
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCA1RXIFG;                     // Clear flag
    UCA1TXBUF = addr;                       // Send address
    while (!(IFG2&UCA1RXIFG));              // Wait for TX to finish
    IFG2 &= ~UCA1RXIFG;                     // Clear flag
    UCA1TXBUF = value;                      // Send data
    while (!(IFG2&UCA1RXIFG));              // Wait for TX to finish
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    unsigned int i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCA1RXIFG;
    UCA1TXBUF = addr | TI_CCxxx0_WRITE_BURST;// Send address
    while (!(IFG2&UCA1RXIFG));              // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      IFG2 &= ~UCA1RXIFG;
      UCA1TXBUF = buffer[i];                // Send data
      while (!(IFG2&UCA1RXIFG));            // Wait for TX to finish
    }
    //while (!(IFG2&UCA1RXIFG));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCA1TXIFG));                // Wait for TX to finish
  UCA1TXBUF = (addr | TI_CCxxx0_READ_SINGLE);// Send address
  while (!(IFG2&UCA1TXIFG));                // Wait for TX to finish
  UCA1TXBUF = 0;                            // Dummy write so we can read data
  // Address is now being TX'ed, with dummy byte waiting in TXBUF...
  while (!(IFG2&UCA1RXIFG));                // Wait for RX to finish
  // Dummy byte RX'ed during addr TX now in RXBUF
  IFG2 &= ~UCA1RXIFG;                       // Clear flag set during addr write
  while (!(IFG2&UCA1RXIFG));                // Wait for end of dummy byte TX
  // Data byte RX'ed during dummy byte write is now in RXBUF
  x = UCA1RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCA1RXIFG;                       // Clear flag
  UCA1TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCA1TXIFG));                // Wait for TXBUF ready
  UCA1TXBUF = 0;                            // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG2&UCA1RXIFG));                // Wait for end of addr byte TX
  IFG2 &= ~UCA1RXIFG;                       // Clear flag
  while (!(IFG2&UCA1RXIFG));                // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCA1TXBUF = 0;                          //Initiate next data RX, meanwhile..
    buffer[i] = UCA1RXBUF;                  // Store data from last data RX
    while (!(IFG2&UCA1RXIFG));              // Wait for RX to finish
  }
  buffer[count-1] = UCA1RXBUF;              // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA1_PxIN & TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCA1RXIFG;                       // Clear flag set during last write
  UCA1TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCA1RXIFG));                // Wait for TX to finish
  IFG2 &= ~UCA1RXIFG;                       // Clear flag set during last write
  UCA1TXBUF = 0;                            // Dummy write so we can read data
  while (!(IFG2&UCA1RXIFG));                // Wait for RX to finish
  x = UCA1RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  IFG2 &= ~UCA1RXIFG;                       // Clear flag
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
  UCA1TXBUF = strobe;                       // Send strobe
  // Strobe addr is now being TX'ed
  while (!(IFG2&UCA1RXIFG));                // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);// Wait for CCxxxx ready
  UCA1TXBUF = TI_CCxxx0_SRES;               // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~UCA1RXIFG;                       // Clear flag
  while (!(IFG2&UCA1RXIFG));                // Wait for end of addr TX
  while (TI_CC_SPI_USCIA1_PxIN&TI_CC_SPI_USCIA1_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USCIB0


void TI_CC_SPISetup(void)
{
#if 0
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable - CS == output

  UCB0CTL1 = UCSWRST;      
  
  UCB0CTL0 |= UCMST+/*UCCKPL*/ UCCKPH +UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                     // SMCLK    //* good one */
  
  UCB0BR0 |= 0x02;                          // UCLK/2
  UCB0BR1 = 0;
  
    
  TI_CC_SPI_USCIB0_PxSEL |= TI_CC_SPI_USCIB0_SIMO | TI_CC_SPI_USCIB0_SOMI | TI_CC_SPI_USCIB0_UCLK;
                                            // SPI option select
  TI_CC_SPI_USCIB0_PxDIR |= TI_CC_SPI_USCIB0_SIMO | TI_CC_SPI_USCIB0_UCLK;
                                            // SPI TXD out direction

  UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
#endif
  // Set Direction of /CS(0x01), SIMO(0x02), UCLK(0x08) - o/p direction
  // Set Direction of SOMI(0x00) - i/p direction
  P3DIR |= (0x08 | 0x00 | 0x02 | 0x01);
  
  // Pull-up P3.0, P3.1, P3.2, P3.3
  //P3REN |= (0x08 | 0x04 | 0x02 | 0x01);
  
  // /CS disable
  P3OUT |= 0x01;
  
  // USCI clock source select
  // UCSWRST = 1; Enabled.USCI logic held in reset state
  UCB0CTL1 = 0x01;
  
  // USCI clock source select -
  // SMCLK (0x80,0x00), UCSWRST = 1; Enabled. 
  // USCI logic held in reset state
  UCB0CTL1 |= (0x80 | 0x01);
  
  // Captured 1st edge - changed following edge(0x80),
  // Inactive State of CLK = low(0x00)/high( 0x40)
  // MSB First(0x20), 8 bit data(0x00), Master Mode(0x08), 
  // 3 pin SPI(0x00,0x00) , Sync mode(0x01),
  UCB0CTL0 |= 0x80 | 0x00 | 0x20 | 0x00 | 0x08 | 0x00 | 0x00 | 0x01;
  //
  UCB0BR0 = 0x02;
  UCB0BR1 = 0x00;
  
  // SPI option select: Primary peripheral module function selected for
  // P3.1 - UCB0SIMO(0x02) , P3.2 - UCB0SOMI(0x04) ,P3.3 - UCB0CLK (0x08)
  P3SEL |= (0x08 | 0x04 | 0x02);
  
  // Initialize USART state machine:
  // UCSWRST = 0; Disabled.USCI reset released for operation.
  UCB0CTL1 &=~(0x01);  
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCB0RXIFG;                     // Clear flag
    UCB0TXBUF = addr;                       // Send address
    while (!(IFG2&UCB0RXIFG));              // Wait for TX to finish
    IFG2 &= ~UCB0RXIFG;                     // Clear flag
    UCB0TXBUF = value;                      // Send data
    while (!(IFG2&UCB0RXIFG));              // Wait for TX to finish
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    unsigned int i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCB0RXIFG;
    UCB0TXBUF = addr | TI_CCxxx0_WRITE_BURST;// Send address
    while (!(IFG2&UCB0RXIFG));              // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      IFG2 &= ~UCB0RXIFG;
      UCB0TXBUF = buffer[i];                // Send data
      while (!(IFG2&UCB0RXIFG));            // Wait for TX to finish
    }
    //while (!(IFG2&UCB0RXIFG));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TX to finish
  UCB0TXBUF = (addr | TI_CCxxx0_READ_SINGLE);// Send address
  while (!(IFG2&UCB0TXIFG));                // Wait for TX to finish
  UCB0TXBUF = 0;                            // Dummy write so we can read data
  // Address is now being TX'ed, with dummy byte waiting in TXBUF...
  while (!(IFG2&UCB0RXIFG));                // Wait for RX to finish
  // Dummy byte RX'ed during addr TX now in RXBUF
  IFG2 &= ~UCB0RXIFG;                       // Clear flag set during addr write
  while (!(IFG2&UCB0RXIFG));                // Wait for end of dummy byte TX
  // Data byte RX'ed during dummy byte write is now in RXBUF
  x = UCB0RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCB0RXIFG;                       // Clear flag
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = 0;                            // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG2&UCB0RXIFG));                // Wait for end of addr byte TX
  IFG2 &= ~UCB0RXIFG;                       // Clear flag
  while (!(IFG2&UCB0RXIFG));                // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB0TXBUF = 0;                          //Initiate next data RX, meanwhile..
    buffer[i] = UCB0RXBUF;                  // Store data from last data RX
    while (!(IFG2&UCB0RXIFG));              // Wait for RX to finish
  }
  buffer[count-1] = UCB0RXBUF;              // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB0_PxIN & TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCB0RXIFG;                       // Clear flag set during last write
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCB0RXIFG));                // Wait for TX to finish
  IFG2 &= ~UCB0RXIFG;                       // Clear flag set during last write
  UCB0TXBUF = 0;                            // Dummy write so we can read data
  while (!(IFG2&UCB0RXIFG));                // Wait for RX to finish
  x = UCB0RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  IFG2 &= ~UCB0RXIFG;                       // Clear flag
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
  UCB0TXBUF = strobe;                       // Send strobe
  // Strobe addr is now being TX'ed
  while (!(IFG2&UCB0RXIFG));                // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
#if 0
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);// Wait for CCxxxx ready
  UCB0TXBUF = TI_CCxxx0_SRES;               // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~UCB0RXIFG;                       // Clear flag
  while (!(IFG2&UCB0RXIFG));                // Wait for end of addr TX
  while (TI_CC_SPI_USCIB0_PxIN&TI_CC_SPI_USCIB0_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
#endif

  while(!(P3OUT & 0x01));
  
  // /CS enable
  P3OUT &= ~(0x01);
  
  //delay of 10 us
  TI_CC_Wait(10) ;
  
  // /CS disable
  P3OUT |= 0x01;
  
  //delay of 100 us
  TI_CC_Wait(100) ;
  
  // /CS enable
  P3OUT &= ~(0x01);
  
  // Wait for CCxxxx ready i.e. Wait for SOMI bit to go low
  while (P3IN & 0x04);
  
  // Clear Receive flag UCB0RXIFG
  IFG2 &= ~(0x04);
  
  // Reset CC2500 (0x30)
  UCB0TXBUF = 0x30;
  
  // Strobe addr is now being TX'ed
  // Wait for end of addr RX
  while (!(IFG2 & UCB0RXIFG));
  
  // Wait for CCxxxx ready i.e. Wait for SOMI bit to go low
  while (P3IN & 0x04);
  
  // /CS disable
  P3OUT |= 0x01;  
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USCIB1


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  UCB1CTL0 |= UCMST+UCCKPL+UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
  UCB1CTL1 |= UCSSEL_2;                     // SMCLK
  UCB1BR0 |= 0x02;                          // UCLK/2
  UCB1BR1 = 0;
  UCB1MCTL = 0;
  TI_CC_SPI_USCIB1_PxSEL |= TI_CC_SPI_USCIB1_SIMO | TI_CC_SPI_USCIB1_SOMI | TI_CC_SPI_USCIB1_UCLK;
                                            // SPI option select
  TI_CC_SPI_USCIB1_PxDIR |= TI_CC_SPI_USCIB1_SIMO | TI_CC_SPI_USCIB1_UCLK;
                                            // SPI TXD out direction
  UCB1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCB1RXIFG;                     // Clear flag
    UCB1TXBUF = addr;                       // Send address
    while (!(IFG2&UCB1RXIFG));              // Wait for TX to finish
    IFG2 &= ~UCB1RXIFG;                     // Clear flag
    UCB1TXBUF = value;                      // Send data
    while (!(IFG2&UCB1RXIFG));              // Wait for TX to finish
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    unsigned int i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~UCB1RXIFG;
    UCB1TXBUF = addr | TI_CCxxx0_WRITE_BURST;// Send address
    while (!(IFG2&UCB1RXIFG));              // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      IFG2 &= ~UCB1RXIFG;
      UCB1TXBUF = buffer[i];                // Send data
      while (!(IFG2&UCB1RXIFG));            // Wait for TX to finish
    }
    //while (!(IFG2&UCB1RXIFG));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB1TXIFG));                // Wait for TX to finish
  UCB1TXBUF = (addr | TI_CCxxx0_READ_SINGLE);// Send address
  while (!(IFG2&UCB1TXIFG));                // Wait for TX to finish
  UCB1TXBUF = 0;                            // Dummy write so we can read data
  // Address is now being TX'ed, with dummy byte waiting in TXBUF...
  while (!(IFG2&UCB1RXIFG));                // Wait for RX to finish
  // Dummy byte RX'ed during addr TX now in RXBUF
  IFG2 &= ~UCB1RXIFG;                       // Clear flag set during addr write
  while (!(IFG2&UCB1RXIFG));                // Wait for end of dummy byte TX
  // Data byte RX'ed during dummy byte write is now in RXBUF
  x = UCB1RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCB1RXIFG;                       // Clear flag
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCB1TXIFG));                // Wait for TXBUF ready
  UCB1TXBUF = 0;                            // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while (!(IFG2&UCB1RXIFG));                // Wait for end of addr byte TX
  IFG2 &= ~UCB1RXIFG;                       // Clear flag
  while (!(IFG2&UCB1RXIFG));                // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB1TXBUF = 0;                          //Initiate next data RX, meanwhile..
    buffer[i] = UCB1RXBUF;                  // Store data from last data RX
    while (!(IFG2&UCB1RXIFG));              // Wait for RX to finish
  }
  buffer[count-1] = UCB1RXBUF;              // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB1_PxIN & TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
  IFG2 &= ~UCB1RXIFG;                       // Clear flag set during last write
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCB1RXIFG));                // Wait for TX to finish
  IFG2 &= ~UCB1RXIFG;                       // Clear flag set during last write
  UCB1TXBUF = 0;                            // Dummy write so we can read data
  while (!(IFG2&UCB1RXIFG));                // Wait for RX to finish
  x = UCB1RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  IFG2 &= ~UCB1RXIFG;                       // Clear flag
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
  UCB1TXBUF = strobe;                       // Send strobe
  // Strobe addr is now being TX'ed
  while (!(IFG2&UCB1RXIFG));                // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);// Wait for CCxxxx ready
  UCB1TXBUF = TI_CCxxx0_SRES;               // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~UCB1RXIFG;                       // Clear flag
  while (!(IFG2&UCB1RXIFG));                // Wait for end of addr TX
  while (TI_CC_SPI_USCIB1_PxIN&TI_CC_SPI_USCIB1_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USI


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  USICTL0 |= USIPE7 +  USIPE6 + USIPE5 + USIMST + USIOE;// Port, SPI master
  USICKCTL = USISSEL_2 + USICKPL;           // SCLK = SMCLK
  USICTL0 &= ~USISWRST;                     // USI released for operation

  USISRL = 0x00;                            // Ensure SDO low instead of high,
  USICNT = 1;                               // to avoid conflict with CCxxxx
}

void TI_CC_SPIWriteReg(char addr, char value)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = addr;                            // Load address
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  USISRL = value;                           // Load value
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = addr | TI_CCxxx0_WRITE_BURST;    // Load address
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  for (i = 0; i < count; i++)
  {
    USISRL = buffer[i];                     // Load data
    USICNT = 8;                             // Send it
    while (!(USICTL1&USIIFG));              // Wait for TX to finish
  }
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = addr | TI_CCxxx0_READ_SINGLE;    // Load address
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  USICNT = 8;                               // Dummy write so we can read data
  while (!(USICTL1&USIIFG));                // Wait for RX to finish
  x = USISRL;                               // Store data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = addr | TI_CCxxx0_READ_BURST;     // Load address
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  for (i = 0; i < count; i++)
  {
    USICNT = 8;                             // Dummy write so we can read data
    while (!(USICTL1&USIIFG));              // Wait for RX to finish
    buffer[i] = USISRL;                     // Store data
  }
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = addr | TI_CCxxx0_READ_BURST;     // Load address
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  USICNT = 8;                               // Dummy write so we can read data
  while (!(USICTL1&USIIFG));                // Wait for RX to finish
  x = USISRL;                               // Store data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);// Wait for CCxxxx ready
  USISRL = strobe;                          // Load strobe
  USICNT = 8;                               // Send it
  while (!(USICTL1&USIIFG));                // Wait for TX to finish
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  // Sec. 27.1 of CC1100 datasheet
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);
  USISRL = TI_CCxxx0_SRES;
  USICNT = 8;
  while (!(USICTL1&USIIFG));
  while (TI_CC_SPI_USI_PxIN&TI_CC_SPI_USI_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
}


#elif TI_CC_RF_SER_INTF == TI_CC_SER_INTF_BITBANG

void TI_CC_SPI_bitbang_out(char);
char TI_CC_SPI_bitbang_in();

void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  // Config bitbang pins
  TI_CC_SPI_BITBANG_PxOUT |= TI_CC_SPI_BITBANG_SIMO|TI_CC_SPI_BITBANG_UCLK;
  TI_CC_SPI_BITBANG_PxDIR |= TI_CC_SPI_BITBANG_SIMO | TI_CC_SPI_BITBANG_UCLK;
}

// Output eight-bit value using selected bit-bang pins
void TI_CC_SPI_bitbang_out(char value)
{
  char x;

  for(x=8;x>0;x--)
  {
    if(value & 0x80)                                     // If bit is high...
      TI_CC_SPI_BITBANG_PxOUT |= TI_CC_SPI_BITBANG_SIMO; // Set SIMO high...
    else
      TI_CC_SPI_BITBANG_PxOUT &= ~TI_CC_SPI_BITBANG_SIMO;// Set SIMO low...
    value = value << 1;                                  // Rotate bits

    TI_CC_SPI_BITBANG_PxOUT &= ~TI_CC_SPI_BITBANG_UCLK;  // Set clock low
    TI_CC_SPI_BITBANG_PxOUT |= TI_CC_SPI_BITBANG_UCLK;   // Set clock high
  }
}

// Input eight-bit value using selected bit-bang pins
char TI_CC_SPI_bitbang_in()
{
  char x=0,shift=0;
  int y;

  // Determine how many bit positions SOMI is from least-significant bit
  x = TI_CC_SPI_BITBANG_SOMI;
  while(x>1)
  {
    shift++;
    x = x >> 1;
  };

  x = 0;
  for(y=8;y>0;y--)
  {
    TI_CC_SPI_BITBANG_PxOUT &= ~TI_CC_SPI_BITBANG_UCLK;// Set clock low
    TI_CC_SPI_BITBANG_PxOUT |= TI_CC_SPI_BITBANG_UCLK;// Set clock high

    x = x << 1;                             // Make room for next bit
    x = x | ((TI_CC_SPI_BITBANG_PxIN & TI_CC_SPI_BITBANG_SOMI) >> shift);
  }                                         // Store next bit
  return(x);
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI); // Wait CCxxxx ready
    TI_CC_SPI_bitbang_out(addr);            // Send address
    TI_CC_SPI_bitbang_out(value);           // Send data
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    char i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI); // Wait CCxxxx ready
    TI_CC_SPI_bitbang_out(addr | TI_CCxxx0_WRITE_BURST);   // Send address
    for (i = 0; i < count; i++)
      TI_CC_SPI_bitbang_out(buffer[i]);     // Send data
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  TI_CC_SPI_bitbang_out(addr | TI_CCxxx0_READ_SINGLE);//Send address
  x = TI_CC_SPI_bitbang_in();               // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI); // Wait CCxxxx ready
  TI_CC_SPI_bitbang_out(addr | TI_CCxxx0_READ_BURST);    // Send address
  for (i = 0; i < count; i++)
    buffer[i] = TI_CC_SPI_bitbang_in();     // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_BITBANG_PxIN & TI_CC_SPI_BITBANG_SOMI); // Wait CCxxxx ready
  TI_CC_SPI_bitbang_out(addr | TI_CCxxx0_READ_BURST);      // Send address
  x = TI_CC_SPI_bitbang_in();               // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI);// Wait for CCxxxx ready
  TI_CC_SPI_bitbang_out(strobe);            // Send strobe
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  // Sec. 27.1 of CC1100 datasheet
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI);
  TI_CC_SPI_bitbang_out(TI_CCxxx0_SRES);
  while (TI_CC_SPI_BITBANG_PxIN&TI_CC_SPI_BITBANG_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
}
#endif
