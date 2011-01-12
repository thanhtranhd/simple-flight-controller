/*===================================================================
 * Project: simple-flight-controller 
 *
 * Hardware settings and hardware related codes 
 * 
 * Copyright (c) 2010 Thanh H Tran (thanhthd@gmail.com)
 *
 * This file is part of simple-flight-controller project
 *
 * Simple-Flight-Controller is free software: 
 * you can redistribute it and /or modify 
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Simple-Flight-Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with simple-flight-controller.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include "msp430x22x4.h"
#include "types.h"
#include "hal.h"
#include "cmd.h"
#include "captures.h"
#include "quad_main.h"
#include "mixer.h"
#include "utils.h"

#ifdef USE_CC2500
#include "wireless.h"
#endif

volatile UINT16 ms_100_timer=TICKS_PER_100_MS;
volatile UINT16 ms_10_timer = TICKS_PER_10_MS;
volatile UINT16 pwm_rate_ticks = PWM_UPDATE_FREQ;

volatile UINT16 sec_ticks = 0;
volatile UINT32 ms100_ticks = 0;

UINT16   adc_buffer[ADC_NUMBER_SAMPLES];

/*------------------------------------------------------------------------------
* SPI
------------------------------------------------------------------------------*/
#ifndef USE_CC2500
void SPI_init(void)
{
   P3OUT |= P3_SPI_CSN;
   P3DIR |= P3_SPI_CSN;                      // /CS disable - CS == output
   
   UCB0CTL1 = UCSWRST;      
   
   UCB0CTL0 |= UCMST+UCCKPL+UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master. clock phase low
   
   //UCB0CTL0 |= UCMST+/*UCCKPL*/ UCCKPH +UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
   
   UCB0CTL1 |= UCSSEL_2;                     // SMCLK    //* good one */
   
   UCB0BR0 |= 0x02;                          // UCLK/2
   UCB0BR1 = 0;
   
    
   P3SEL |= P3_SPI_USCIB0_SIMO | P3_SPI_USCIB0_SOMI | P3_SPI_USCIB0_UCLK;
                                            // SPI option select
   P3DIR |= P3_SPI_USCIB0_SIMO | P3_SPI_USCIB0_UCLK;
                                            // SPI TXD out direction
   
   UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}
#endif
/*------------------------------------------------------------------------------
* configure hw pins
------------------------------------------------------------------------------*/
void configure_mcu_pins()
{
   P1OUT &= ~(RX_ROLL | RX_PITCH);           // internal pull down   
   P1REN |= (RX_ROLL | RX_PITCH);            // internal pull down enabled.
      
   P1IFG &= ~(RX_ROLL | RX_PITCH);           //Clr flags
   
   P1IE  |= (RX_ROLL | RX_PITCH);            //enable interrupt

   // enable TAx OUT on P1x
   P1DIR |= (TA1_OUT | TA2_OUT);             // output
   P1SEL |= (TA1_OUT | TA2_OUT);             // TAx OUT

   // enable TBx OUT on P4x
   P4DIR |= (TB1_OUT | TB2_OUT);             // output
   P4SEL |= (TB1_OUT | TB2_OUT);             // TAx OUT
   
   // set up P2 intr inputs
   P2OUT &= ~(RX_THROT | RX_RUDD | 
             RX_TX_GAIN);                    // internal pull down
                          
   P2REN |= (RX_THROT | RX_RUDD |
             RX_TX_GAIN);                    // internal pull down enabled.
      
   P2IFG &= ~(RX_THROT | RX_RUDD |
             RX_TX_GAIN);                    //Clr flags
   
   P2IE  |= (RX_THROT | RX_RUDD |
             RX_TX_GAIN);                    //enable interrupt from these pins   

   P2SEL &= ~(RX_THROT | RX_RUDD |
             RX_TX_GAIN);                    //allow pin interrupts 
   
   P1DIR |= (RED_LED | GREEN_LED);           //Outputs
   P1OUT &= ~(RED_LED | GREEN_LED);          // turn off both leds.
   
   // LCD pins
   LCD_CTRL_DIR |= LCD_SPI_CSN;              // output direction. 
   LCD_CONTROL |= LCD_SPI_CSN;	            // CSn = 1; DAC is tri-stated
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
   TACCTL0  = OUTMOD_7 +                     // output clock 
              CCIE;                          // TACCR0 interrupt enabled
                                             
   TACCTL1  = OUTMOD_7;                      // output with set / reset - pwm
   TACCTL2  = OUTMOD_7;
   
   TACCR0   = TMR_A_PERIOD;                  
   
   TACTL    =  TASSEL_2 +                    // SMCLK
#ifdef USE_BRUSHED_ESC
#warn "!!! code built for brushed ESC!!!"
               ID_2 +                        // devided by 4
#else
               ID_3 +                        // devided by 8
#endif               
               MC_1;                         // up   

   // timer B
   TBCCTL1  = OUTMOD_7;                      // output with set / reset - pwm
   TBCCTL2  = OUTMOD_7;
   
   TBCCR0   = TMR_A_PERIOD;                  // 10ms
   
   TBCTL    =  TBSSEL_2 +                    // SMCLK
#ifdef USE_BRUSHED_ESC
               ID_2 +                        // devided by 4
#else
               ID_3 +                        // devided by 8
#endif               
               MC_1;                         // up      
}

/*------------------------------------------------------------------------------
* update PWM
------------------------------------------------------------------------------*/
void update_pwm()
{
   if (!(mixer_flags & MIXER_DISC_INPUT_ON)) 
   {
      mix_mixing_quad  ((INT16)ail_pulse, (INT16)pit_pulse, 
                        (INT16)thr_pulse, (INT16)rud_pulse, 
                        (INT16*)&front_motor,  (INT16*)&back_motor,  
                        (INT16*)&left_motor,  (INT16*)&right_motor);
   
      TACCR1 = front_motor;
      TACCR2 = back_motor;
      
      TBCCR1 = left_motor;
      TBCCR2 = right_motor;
   }
   else
   {
      TACCR1 = 0;
      TACCR2 = 0;
      
      TBCCR1 = 0;
      TBCCR2 = 0;
   }
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
   
   timer_init();   
   UART_init();
   //SPI_init();
   ADC_init();   

   start_adc(ADC_ROLL_INCH);

   __enable_interrupt();  
}

/*------------------------------------------------------------------------------
* Timer A0 interrupt service routine
* Timer A0 ticks every TIMER_A period as defined in hal.h. 
------------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{   
   if (!ms_100_timer)
   {
      ms_100_timer = TICKS_PER_100_MS;
      ms100_ticks++;
   } 
   ms_100_timer--;
   
   if (!ms_10_timer)
   {
      ms_10_timer = TICKS_PER_10_MS;      
   } 
   ms_10_timer--;
   
   if (!pwm_rate_ticks)
   {
      pwm_rate_ticks = PWM_UPDATE_FREQ;      
#ifdef USE_BRUSHED_ESC
      woken_up_by |= WOKEN_UP_BY_TIMER;
      __bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU: 100 hz
#endif      
   }
   pwm_rate_ticks--;

#ifndef USE_BRUSHED_ESC
   // wake up every timer ticks if we are not doing high speed PWM 
   // as in brushed ESC case.
   woken_up_by |= WOKEN_UP_BY_TIMER;
   __bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
#endif      

}


/*------------------------------------------------------------------------------
* PORT1 pin change ISR
------------------------------------------------------------------------------*/
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR (void)
{
#ifndef USE_CC2500		   
   if (P1IFG & RX_ROLL)
   {   
      capture_timing(RX_ROLL, &ail_ccb, &ail_pulse);
      P1IFG &= ~(RX_ROLL);                   //Clr flag that caused int
   }

   if (P1IFG & RX_PITCH)
   {   
      capture_timing(RX_PITCH, &pit_ccb, &pit_pulse);
      P1IFG &= ~(RX_PITCH);                  //Clr flag that caused int
   }
#endif   
}


/*------------------------------------------------------------------------------
// PORT 2 ISR 
------------------------------------------------------------------------------*/
#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
#ifndef USE_CC2500	
#warn "!!! code built for external RX!!!"
   if (P2IFG & RX_THROT)
   {   
      capture_p2_timing(RX_THROT, &thr_ccb, &thr_pulse);
      P2IFG &= ~(RX_THROT);                   //Clr flag that caused int
   }

   if (P2IFG & RX_RUDD)
   {   
      capture_p2_timing(RX_RUDD, &rud_ccb, &rud_pulse);
      P2IFG &= ~(RX_RUDD);                  //Clr flag that caused int
   }

   if (P2IFG & RX_TX_GAIN)
   {   
      capture_p2_timing(RX_TX_GAIN, &tx_gain_ccb, &tx_gain_pulse);
      P2IFG &= ~(RX_TX_GAIN);               //Clr flag that caused int
   }
#endif

#ifdef USE_CC2500   
#warn "!!! code built for built-in RX - using CC2500!!!"
   if (P2IFG & CC2500_GDO0)
   {
   	  // Fetch packet from CCxxxx & check the CRC==0x80 (OK)
      if (0x80 == RFReceivePacket(cc2500_rx_buffer, &cc2500_rx_buffer_len) &&
          (cc2500_rx_buffer_len == 11)) // TX module sent 10 bytes for 5 PWM values plus 1 byte address.   
      {
      	 // the following channels assignment is Spektrum / JR style 
      	 // which has the first channel in the PPM stream being the throttle
      	 // where as Futaba & Hitec use first channel as aileron.      	 
      	 thr_pulse = (cc2500_rx_buffer[1] << 8) | cc2500_rx_buffer[2];
      	 ail_pulse = (cc2500_rx_buffer[3] << 8) | cc2500_rx_buffer[4];
      	 pit_pulse = (cc2500_rx_buffer[5] << 8) | cc2500_rx_buffer[6];
      	 rud_pulse = (cc2500_rx_buffer[7] << 8) | cc2500_rx_buffer[8];
      	 tx_gain_pulse = (cc2500_rx_buffer[9] << 8) | cc2500_rx_buffer[10];
      	       	 
         //woken_up_by |= WOKEN_UP_BY_WIRELESS;
         //__bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
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

   //process_adc_results();
      
	// process ADC results from main loop
   woken_up_by |= WOKEN_UP_BY_ADC;
   __bic_SR_register_on_exit(LOW_POWER_MODE);  // wake MCU
}

/*------------------------------------------------------------------------------
* ADC init
------------------------------------------------------------------------------*/
void ADC_init()
{
   ADC10CTL0 = SREF_1 +                      // Vr+ = Vref, Vr- = Vss
               //ADC10SHT_2 +                  // sample and hold time: 16 clk
               ADC10SHT_3 +                  // sample and hold time: 64 clk
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
// send letter to SPI-enabled LCD.
// assume SPI is already set up correctly prior to this function being called.
------------------------------------------------------------------------------*/

void putCharLCD(UINT8 letter)
{		
   LCD_CONTROL &= ~LCD_SPI_CSN;              // /CS enable
   IFG2 &= ~UCB0RXIFG;	                     // Clear flag
   UCB0TXBUF = letter;                       // Send data
   while (!(IFG2&UCB0RXIFG));                // Wait for TX to finish
   
   LCD_CONTROL |= LCD_SPI_CSN;               // /CS disable
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
