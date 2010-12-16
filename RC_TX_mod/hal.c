/*===================================================================
 * Project: rc_ppm_tx_mod 
 *
 * Hardware settings and hardware related codes 
 * 
 * Copyright (c) 2010 Thanh H Tran (thanhthd@gmail.com)
 *
 * This file is part of rc_ppm_tx_mod project
 *
 * rc_ppm_tx_mod is free software: 
 * you can redistribute it and /or modify 
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * rc_ppm_tx_mod is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with rc_ppm_tx_mod.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include "msp430x22x4.h"
#include "types.h"
#include "hal.h"
#include "cmd.h"
#include "captures.h"
#include "tx_main.h"
#include "utils.h"

#ifdef USE_CC2500
#include "wireless.h"
#endif

unsigned int timer=TICKS_PER_100_MS;
volatile UINT16 sec_ticks = 0;
volatile UINT32 ms100_ticks = 0;

UINT16   adc_buffer[ADC_NUMBER_SAMPLES];

/*------------------------------------------------------------------------------
* configure hw pins
------------------------------------------------------------------------------*/
void configure_mcu_pins()
{
   P1OUT &= ~(RX_SW1);                       // internal pull down   
   P1REN |= (RX_SW1);                        // internal pull down enabled.
      
   P1IFG &= ~(RX_SW1);                       //Clr flags
   
   P1IE  |= (RX_SW1);                        //enable interrupt
   
   // set up P2 intr inputs
   P2OUT &= ~(PPM_INPUT);                    // internal pull down
   //P2OUT |= (PPM_INPUT);                     // internal pull up                          
   P2REN |= (PPM_INPUT);                     // internal pull (u/d) enabled.
        
   P2IFG &= ~(PPM_INPUT);                    //Clr flags
   
   P2IE  |= (PPM_INPUT);                     //enable interrupt from these pins   

   P2SEL &= ~(PPM_INPUT);                    //allow pin interrupts 
   
   P1DIR |= (RED_LED | GREEN_LED);           //Outputs
   P1OUT &= ~(RED_LED | GREEN_LED);          // turn off both leds.   
}

/*------------------------------------------------------------------------------
* UART
------------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------------
* timer A0: 1Mhz (1 micro second resolution to measure the pulse) & base signal
* timer A1: PWM1
* timer A2: PWM2
------------------------------------------------------------------------------*/
void timer_init()
{
   TACCTL0  = CCIE;                          // TACCR0 interrupt enabled
   TACCR0   = TMR_A_PERIOD;                     
   TACTL    =  TASSEL_2 +                    // SMCLK
               ID_3 +                        // devided by 4
               MC_1;                         // up   
}

/*------------------------------------------------------------------------------
*
------------------------------------------------------------------------------*/
void mcu_init()
{
   WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  
   BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
   DCOCTL = CALDCO_8MHZ;
   
   BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
                                             // SMCLK is now == DCOCLK == MCLK

   // Configure ports -- switch inputs, LEDs, 
   configure_mcu_pins();

   // for PPM reading interrupts
   set_p2_rising_edge(PPM_INPUT);

   timer_init();   
   UART_init();
   
   //ADC_init();   
   //start_adc(ADC_ROLL_INCH);

   __enable_interrupt();  
}

/*------------------------------------------------------------------------------
* Timer A0 interrupt service routine
* 100hz routine according to timer A settings.
------------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{   
   if (!timer)
   {
      timer = TICKS_PER_SEC;
      off_green_led();
	  //woken_up_by |= WOKEN_UP_BY_TIMER;
      //__bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
   } 
   timer--;
}


/*------------------------------------------------------------------------------
* PORT1 pin change ISR
------------------------------------------------------------------------------*/
#if 0
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR (void)
{  
	/* 
   if (P1IFG & RX_ROLL)
   {   
      capture_timing(RX_ROLL, &ail_ccb, &ail_pulse);
      P1IFG &= ~(RX_ROLL);                   //Clr flag that caused int
   }
   */
}
#endif

/*------------------------------------------------------------------------------
// PORT 2 ISR 
------------------------------------------------------------------------------*/
#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
   UINT16 num_pulses = 0;
   
   if (P2IFG & PPM_INPUT)
   { 
      xor_green_led();
   	  
      num_pulses = capture_p2_ppm(PPM_INPUT, &ppm_ccb, ppm_pulses);
      
      if (num_pulses > 0)
      {
      	 num_ppm_pulses = num_pulses;
      	 woken_up_by |= WOKEN_UP_BY_PPM_FRAME;
         __bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU      	
      }
      P2IFG &= ~(PPM_INPUT);                   //Clr flag that caused int      
   }

#ifdef USE_CC2500   
   if (P2IFG & CC2500_GDO0)
   {
   	  // Fetch packet from CCxxxx & check the CRC==0x80 (OK)
      if (0x80 == RFReceivePacket(cc2500_rx_buffer, &cc2500_rx_buffer_len))      
      {
         woken_up_by |= WOKEN_UP_BY_WIRELESS;
         __bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
      }
      P2IFG &= ~(CC2500_GDO0);               //Clr flag that caused int      
   }   
#endif   
}


/*------------------------------------------------------------------------------
* send a string to UART ....
------------------------------------------------------------------------------*/
void tx_string( char* string, int length )
{
  int pointer;
  for( pointer = 0; pointer < length; pointer++)
  {
    UCA0TXBUF = string[pointer];
    while (!(IFG2&UCA0TXIFG));                 // USCI_A0 TX buffer ready?
  }
}

/*------------------------------------------------------------------------------
* send a char to UART ....
------------------------------------------------------------------------------*/
void tx_char( char digit)
{
    UCA0TXBUF = digit;
    while (!(IFG2&UCA0TXIFG));                 // USCI_A0 TX buffer ready?
}

/*------------------------------------------------------------------------------
* USCIA interrupt service routine
* UART receiving a character.
------------------------------------------------------------------------------*/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{   
   char rx = UCA0RXBUF;
   
   if (cmdLen < CMD_MAX_LEN) 
   {
      cmdBuffer[cmdLen++] = rx;
   }
   else
   {
      cmdBuffer[cmdLen] = 0;
   }
  
   if (rx == 13)
   {    	              
      cmdBuffer[cmdLen-1] = 0;
      woken_up_by |= WOKEN_UP_BY_CMD;
      
      //wake_mcu();
      __bic_SR_register_on_exit(LOW_POWER_MODE);
   }
   
   tx_string(&rx,1); // might need to bring this print out of the interrupt!
}

/*------------------------------------------------------------------------------
* ADC10 interrupt service routine
------------------------------------------------------------------------------*/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{   
   on_red_led();
   
	// process ADC results from main loop
	woken_up_by |= WOKEN_UP_BY_ADC;
   //wake_mcu();
   __bic_SR_register_on_exit(LOW_POWER_MODE);
}

/*------------------------------------------------------------------------------
* ADC init
------------------------------------------------------------------------------*/
void ADC_init()
{
   ADC10CTL0 = SREF_1 +                      // Vr+ = Vref, Vr- = Vss
               ADC10SHT_2 +                  // sample and hold time: 16 clk
               REF2_5V + REFON +             // internal 2.5v ref   
               ADC10ON +                     // turn on ADC
               MSC +                         // multiple samples & conversions   
               ADC10IE;                      // ADC interrupt enabled
      
   ADC10AE0  = ADC_PITCH_PIN | 
               ADC_YAW_PIN;                  // enable analog inputs

   ADC10AE1  = ADC_ROLL_PIN;                 // enable analog inputs: A12 and up goes to ADC10AE1 

   ADC10DTC1 = ADC_NUMBER_SAMPLES;           // 16 conversions for each channel
                                             // doing dithering & oversampling   
}

/*------------------------------------------------------------------------------
// start ADC
------------------------------------------------------------------------------*/
void start_adc(UINT16 adc_channel)
{		
   ADC10CTL0 &= ~ENC;   
   while (ADC10CTL1 & BUSY);                 // Wait if ADC10 core is active
   
   ADC10CTL1 = SHS_0 +                       // ADC10SC bit to start conversion
               ADC10SSEL_3 +                 // SMCLK
               ADC10DIV_3 +                  // clock divider   
               CONSEQ_2 +                    // repeated single channel conversion
               adc_channel;                  
   
   
   ADC10SA = (UINT16)adc_buffer;             // Data buffer start
   
   ADC10CTL0 |= ENC + ADC10SC;               // Enable & start Sampling & conversion 
   
//   _BIS_SR(GIE);                 				// enable interrupts   
}



/*------------------------------------------------------------------------------
// Write value to flash. Copied from MSP430F22xx examples
------------------------------------------------------------------------------*/

void write_flash(UINT16 address, INT8* buffer, INT8 length)
{
	int i = 0;
   char *Flash_ptr = 0;               			// Flash pointer
   
   Flash_ptr = (char*)address;
   
   if (length >=64)
   {
   	// a segment is only 64 byte. 
   	return;
   }
   	
   FCTL3 = FWKEY;                            // Clear Lock bit
   FCTL1 = FWKEY + ERASE + EEI;              // Set Erase bit and allow interrupt
   *Flash_ptr = 0;                           // Dummy write to erase Flash seg
   
   FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
   
   for (i = 0; i < length; i++)
   {
      *Flash_ptr++ = buffer[i];              // Write value to flash
   }
   
   FCTL1 = FWKEY;                            // Clear WRT bit
   FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}
