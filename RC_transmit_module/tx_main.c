/*===================================================================
 * Project: simple-flight-controller 
 *
 * Transmitting Module 
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
 * along with simple-flight-controller .  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include <string.h>
#include "types.h"
#include "hal.h"
#include "captures.h"
#include "utils.h"
#include "cmd.h"
#include "tx_main.h"

#ifdef USE_CC2500
#include "wireless.h"
#endif


// prototypes
UINT8 process_adc_results();
UINT8 find_gyro_neutral_values();
void process_stick_input_commands();

//data for terminal output
const char splash[] = {"\r\n------- PPM TX Module says hello :) -----\r\n\r\n"};

// pulse lengths
UINT16 ail_pulse = 0;
UINT16 pit_pulse = 0;
UINT16 thr_pulse = 0;
UINT16 rud_pulse = 0;
UINT16 tx_gain_pulse = 0;           // if this value is read and is 
UINT16 num_ppm_pulses=0;
UINT16 ppm_pulses[MAX_PPM_CHANNELS]={0,0,0,0,0,0,0};

CAPTURE_CCB ppm_ccb;

UINT8   woken_up_by = WOKEN_UP_BY_UNKNOWN;

//--------------------------------------------------------------------------
// main program
//--------------------------------------------------------------------------
void main (void)
{  
   UINT16 init_time_ms; 

   init_ccb(&ppm_ccb);

   mcu_init();
   
   //Transmit splash screen 
   tx_string( (char*)splash, sizeof splash);
   tx_string( "\r\n", 2 );

#ifdef USE_CC2500   
   // initialize wireless stuff.
   wireless_init();   
#endif   
         
   while(1)
   {
      off_red_led();
      off_green_led();
      
      if (woken_up_by == WOKEN_UP_BY_UNKNOWN)
      {      	
         // there is nothing else to do now, go to sleep
         sleep_radio();         
         sleep_mcu();                           // Enter low power mode, 
                                                // enable interrupts
      }
      
      // CPU is awoken by some interrupts, continue here
      if (woken_up_by & WOKEN_UP_BY_CMD)
      {
         tx_string("\n\r#",3);                   // go to new line
         if (cmdLen>1) process_cmd(cmdBuffer, cmdLen);   
         cmdLen = 0;                            // reset the cmd index
         woken_up_by &= (~WOKEN_UP_BY_CMD);
      }

      if (woken_up_by & WOKEN_UP_BY_PPM_FRAME /*WOKEN_UP_BY_TIMER*/)  
      {  
         // gather the data captured by the captures module         
         // The variables here are for debugging only at this time
         thr_pulse = ppm_pulses[0];
         ail_pulse = ppm_pulses[1];
         pit_pulse = ppm_pulses[2];
         rud_pulse = ppm_pulses[3];
         tx_gain_pulse = ppm_pulses[4];
         
#ifdef USE_CC2500
         wake_radio();
            
         cc2500_tx_buffer[0] = 11;	// packet length: 10 PPM pulse bytes + 1;
         cc2500_tx_buffer[1] = 1;	// packet address

         cc2500_tx_buffer[2] = (thr_pulse & 0xff00)>>8;
         cc2500_tx_buffer[3] = (thr_pulse & 0xff);

         cc2500_tx_buffer[4] = (ail_pulse & 0xff00)>>8;;
         cc2500_tx_buffer[5] = (ail_pulse & 0xff);

         cc2500_tx_buffer[6] = (pit_pulse & 0xff00)>>8;;        
         cc2500_tx_buffer[7] = (pit_pulse & 0xff);
         
         cc2500_tx_buffer[8] = (rud_pulse & 0xff00)>>8;;        
         cc2500_tx_buffer[9] = (rud_pulse & 0xff);

         cc2500_tx_buffer[10] = (tx_gain_pulse & 0xff00)>>8;;        
         cc2500_tx_buffer[11] = (tx_gain_pulse & 0xff);

         on_red_led();
         RFSendPacket(cc2500_tx_buffer, 12);    // 5 channels, 5 UINT16, 10 bytes plus 2 bytes (length & address)
#endif
         woken_up_by &= (~WOKEN_UP_BY_PPM_FRAME);
      }
      
      if ((woken_up_by & WOKEN_UP_BY_ADC))
      {
         if (process_adc_results() == 0)
         {            
         } // if (calculate_adc_results()==0)
         
         woken_up_by &= (~WOKEN_UP_BY_ADC);         
      } // WOKEN_UP_BY_ADC
      
#ifdef USE_CC2500         
      if (woken_up_by & WOKEN_UP_BY_WIRELESS)
      {
      	 tx_string("Rx signal: ", 11);
         printU8(cc2500_rx_buffer[1]);      	 
         woken_up_by &= (~WOKEN_UP_BY_WIRELESS);
      }
#endif      
   } // while
} /* main */

/*------------------------------------------------------------------------------
* gather ADC results
* return zero when finish taking reading of all 3 gyros, non-zero otherwise
------------------------------------------------------------------------------*/
UINT8 process_adc_results()
{
   static UINT8  adc_idx = 0;
   int i = 0;
   UINT16 adc_val_holder = 0;

   // doing ADC Decimation - to increase to 12 bit from 10 bits. 
   // take 4^n samples & then shift right n steps. n is the extra bits: 2 bits
   // 10 bit max val = 1023. 16 times would be: 16*1023 = 16368   
   for (i = ADC_NUMBER_SAMPLES; i>0; i--)
   {
      adc_val_holder += adc_buffer[i-1];
   }
      
   adc_idx++;
  
   if (adc_idx == 1)
   {
      //adc_roll_gyro_val = adc_val_holder >> ADC_DECIMATION_BITS; 
      
      // switch to pitch channel
      start_adc(ADC_PITCH_INCH);
   }   
   
   return adc_idx;   // return this number so that the main program knows
                     // where it is at at doing ADC.
}

