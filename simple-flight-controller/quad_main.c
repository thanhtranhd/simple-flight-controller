/*===================================================================
 * Project: simple-flight-controller 
 *
 * Main Module 
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

#include <string.h>
#include "types.h"
#include "hal.h"
#include "captures.h"
#include "mixer.h"
#include "utils.h"
#include "cmd.h"
#include "quad_main.h"

#ifdef USE_CC2500
#include "wireless.h"
#endif


// prototypes
UINT8 process_adc_results();
UINT8 find_gyro_neutral_values();
void process_stick_input_commands();
void initial_arming(UINT16 init_time_stamp_ms);
void strobe_light(UINT16* strobe_state);


//data for terminal output
const char splash[] = {"\r\n------- MR Copter says hello :) -----\r\n\r\n"};

// pulse lengths
UINT16 ail_pulse = OFF_PULSE_VAL;
UINT16 pit_pulse = OFF_PULSE_VAL;
UINT16 thr_pulse = OFF_PULSE_VAL;
UINT16 rud_pulse = OFF_PULSE_VAL;
UINT16 tx_gain_pulse = 0;           // if this value is read and is 

// Motor PWM pulse lengths
UINT16 front_motor = 0;
UINT16 back_motor  = 0;
UINT16 left_motor  = 0;
UINT16 right_motor = 0;

CAPTURE_CCB ail_ccb;
CAPTURE_CCB pit_ccb;
CAPTURE_CCB thr_ccb;
CAPTURE_CCB rud_ccb;
CAPTURE_CCB tx_gain_ccb;

COPTER_CONFIG_DATA   copter_config_data;

UINT16   woken_up_by = WOKEN_UP_BY_UNKNOWN;
UINT16   strobe_state = 0;

// prototypes
void flash_both_lights();

//--------------------------------------------------------------------------
// main program
//--------------------------------------------------------------------------
void main (void)
{  
   UINT16 init_time_stamp_ms; 

   init_ccb(&ail_ccb);
   init_ccb(&pit_ccb);
   init_ccb(&thr_ccb);
   init_ccb(&rud_ccb);
   init_ccb(&tx_gain_ccb);

   // set default flags
   mixer_flags = 0;
   mixer_flags |= MIXER_DISC_INPUT_ON; // no pulses on any of the motor or servo
   
   mcu_init();
   
   //Transmit splash screen 
   tx_string( (char*)splash, sizeof splash);
   tx_string( "\r\n", 2 );

#ifdef USE_CC2500   
   // initialize wireless stuff.
   wireless_init();   
#endif   

   // record time stamp
   init_time_stamp_ms = ms100_ticks;

   // get settings
   read_config_from_flash();
   
   show_info();
         
   while(1)
   {
      off_red_led();
      
      if (woken_up_by == WOKEN_UP_BY_UNKNOWN)
      {
         // there is nothing else to do now, go to sleep
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
      
      if (woken_up_by & WOKEN_UP_BY_TIMER)  
      {        	
         // this portion is being called every timer tick (wakes up from timer)
         if (mixer_flags & MIXER_CALIBRATING)
         {  
            flash_both_lights();
         	mixer_flags &= ~MIXER_DISC_INPUT_ON;
            update_pwm();            
         }
         else if (mixer_flags & MIXER_IS_ARMED)
         {
            if (!(mixer_flags & MIXER_DISC_INPUT_ON))
            { 
               // do some lighting
               strobe_light(&strobe_state);            	 
            }
            // update PWM outputs or doing mixing here
            update_pwm();
         }
         else
         {
         	initial_arming(init_time_stamp_ms);         	
         }
                  
         // do some specific commands when the stick combinations.
         // this function is being called every wakes up by timer. 
         process_stick_input_commands();  

         woken_up_by &= (~WOKEN_UP_BY_TIMER);
      }
      
      if ((woken_up_by & WOKEN_UP_BY_ADC))
      {
         process_adc_results();

         woken_up_by &= (~WOKEN_UP_BY_ADC);         
      } // WOKEN_UP_BY_ADC
      
#ifdef USE_CC2500         
      if (woken_up_by & WOKEN_UP_BY_WIRELESS)
      {
      	 tx_char('.');
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
   UINT16 i = 0;
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
      adc_roll_gyro_val = adc_val_holder >> ADC_DECIMATION_BITS; 
      
      // switch to pitch channel
      start_adc(ADC_PITCH_INCH);
   }   
   if (adc_idx == 2)
   {
      adc_pitch_gyro_val = adc_val_holder >> ADC_DECIMATION_BITS;
            
      // switch to yaw channel
      start_adc(ADC_YAW_INCH);
   }
   if (adc_idx == 3)
   {
      adc_yaw_gyro_val = adc_val_holder >> ADC_DECIMATION_BITS;

      // switch to roll channel
      start_adc(ADC_ROLL_INCH);
      
      // restart counting and keep track of the decimation cycle
      adc_idx = 0;      
   }
   
   return adc_idx;   // return this number so that the main program knows
                     // where it is at at doing ADC.
}

/*------------------------------------------------------------------------------
* find gyro neutral values
* this function is repeatedly called each time we got result from ADC so that
* it gets numberous gyro samples without blocking any other threads.
* function returns zero when it has finished taking NUM_GYRO_SAMPLES samples 
------------------------------------------------------------------------------*/
UINT8 find_gyro_neutral_values()
{
   // this function is being called about 100 times per seconds (depending on timer & ADC settings) 
   // so we need to add a delay beetween captures of the gyro/acc samples.
   static UINT8 delay = 0;
   static UINT8 num_samples = 0;
   static UINT16 gyro_arr[3];
   int i = 0;

   if (num_samples == 0)
   {
      gyro_arr[0] = 0;
      gyro_arr[1] = 0;
      gyro_arr[2] = 0;
   }
   if (delay++ < 20)
   {
      return 1; // anything but zero so that the caller doesn't quit calling
   }
   else
   {
      delay=0; 
      // go on as normal         
   }
   

   num_samples++;
     
   gyro_arr[0] += adc_roll_gyro_val;
   gyro_arr[1] += adc_pitch_gyro_val;
   gyro_arr[2] += adc_yaw_gyro_val;
   
   if (num_samples>=NUM_GYRO_SAMPLES)
   {
      for (i=0; i<3; i++)
      {
         gyro_arr[i] = (gyro_arr[i] / NUM_GYRO_SAMPLES);
      }
      
      neutral_roll_gyro_val = gyro_arr[0];
      neutral_pitch_gyro_val = gyro_arr[1];
      neutral_yaw_gyro_val = gyro_arr[2];   
      
      num_samples = 0;
   }
   
   return num_samples;
}

/*------------------------------------------------------------------------------
* process some commands (basic safety command) from TX
------------------------------------------------------------------------------*/
void process_stick_input_commands()
{
   static UINT16 ticks = 0;
   
   // left stick left-down most
   // disconnect all motors and calibrate gyro again
   if ((thr_pulse <= 1000) && 
       (rud_pulse < 1300))
   {
      off_green_led();                   
      mixer_flags |= MIXER_DISC_INPUT_ON;
      
      //mixer_flags &= ~(MIXER_IS_ARMED);    // is_armed = 0;
      find_gyro_neutral_values();   // calibrate gyro again      
   }

   // left stick right-down most.
   // reconnect all motors
   if ((thr_pulse <= 1000) && 
       (rud_pulse > 1700))
   {
      on_green_led();                   
      mixer_flags &= ~MIXER_DISC_INPUT_ON;
   }
   
   // left stick: left down corner + right stick right up corner: save the stats
   if ((thr_pulse <= 1000) && (rud_pulse < 1300) &&
       (ail_pulse >  1600) && (pit_pulse > 1600))
   {     
      // protection mechanism to prevent continuous flash writing
      // when user keeps holding the sticks at the correct positions.
      // only allow flash write once per 30 seconds.
      // this function is being called every 5ms (depending on TMR_A_PERIOD)
      if (ticks == 0)
      {
         write_flash(FLASH_CONFIG_ADDR, 
                     (char*)&copter_config_data, sizeof(copter_config_data));
         ticks = 6000;  // about 30 seconds with 5ms tick        
      }
   }
   //release the flash write protection mechanism by counting down on writes
   if (ticks > 0) ticks--;
/*
   // left stick: left down corner + right stick left up corner: clear the stats
   if ((thr_pulse <= 1000) && (rud_pulse < 1300) &&
       (ail_pulse <  1400) && (pit_pulse > 1600))
   {
      clear_stats();
   }
*/
}

/*------------------------------------------------------------------------------
* read configuration from flash.
------------------------------------------------------------------------------*/
void read_config_from_flash()
{
   UINT8* cfg_ptr = (unsigned char*)FLASH_CONFIG_ADDR;
   if (*cfg_ptr!=0xFF)
   {
      // there is valid data.
      memcpy((char*)&copter_config_data, cfg_ptr, sizeof(copter_config_data));
      if (copter_config_data.mixer_calibrate==1)
      {
      	 mixer_flags |= MIXER_CALIBRATING;      	
      }
   }
   else
   {
      // there is no valid data in flash. Use default values
      memset((void*)&copter_config_data, 0, sizeof(copter_config_data));
      copter_config_data.min_m1_pulse = 9999;
      copter_config_data.min_m2_pulse = 9999;
      copter_config_data.min_m3_pulse = 9999;
      copter_config_data.min_m4_pulse = 9999;
      copter_config_data.min_x_gyro_rate = 9999;
      copter_config_data.min_y_gyro_rate = 9999;
      copter_config_data.min_z_gyro_rate = 9999;
            
      copter_config_data.gyro_dir = 0;

      copter_config_data.gyro_gain = DEFAULT_GYRO_GAIN;      
      copter_config_data.gain_x_bias = 0;
      copter_config_data.gain_y_bias = 0;
      copter_config_data.gain_z_bias = 0;
      
      copter_config_data.yaw_subtrim = 0;
   }

   printU8(copter_config_data.gyro_dir);
   tx_string("\n\r",2);                      
   printU16(copter_config_data.gyro_gain);           // gyro gain 
   tx_string("\n\r",2);                   
   printU16(copter_config_data.gain_x_bias);    // gyro x gain offset
   tx_string("\n\r",2);                   
   printU16(copter_config_data.gain_y_bias);    // gyro y gain offset
   tx_string("\n\r",2);                     
   printU16(copter_config_data.gain_z_bias);    // gyro z gain offset
   tx_string("\n\r",2);                      
   printU16(copter_config_data.yaw_subtrim);    // yaw substrim
   tx_string("\n\r",2);                      
}

/*------------------------------------------------------------------------------
 * wait for about 5 seconds before arming the controller.
 * Assuming we have finished reading ADC for all 3 gyro channels
 * wait for a few seconds before finding gyro neutral values 
 * to avoid false reading cause by vibration when the motors 
 * beep right after power up. or because user plugging / unplugging battery.
 * This code won't be exercised again after the board has been armed.
 * also since ADC starts immediately after boot, waiting about 5 second
 * should have all 3 gyro channels sampled completely. 
 *------------------------------------------------------------------------------*/
void initial_arming(UINT16 init_time_stamp_ms)
{
    if ((!(mixer_flags & MIXER_IS_ARMED)) && 
        (INIT_DELAY_TIME_100MS <= (ms100_ticks - init_time_stamp_ms)))
    {                   
       // find gyro neutral point. This function reads gyro for about 2 seconds
       if (find_gyro_neutral_values() == 0)
       {            
          // wait for radio signal exists in all channels and throttle 
          // is all the way down before arming the mixer
          if ((thr_pulse <= 1000) && 
              (pit_pulse >1400) && (pit_pulse < 1600) &&
              (ail_pulse >1400) && (ail_pulse < 1600) &&
              (rud_pulse >1400) && (rud_pulse < 1600))
          {
             on_green_led();                   // turn on green LED
             mixer_flags |= MIXER_IS_ARMED;
            
             // check to see if we can use TX gain. If there is
             // no pulse on the gain channel, use default settings. 
             if ((tx_gain_pulse < 800) || (tx_gain_pulse > 2000))
             {
                 tx_gain_pulse = copter_config_data.gyro_gain;
             }                     
          }
       }
    }            
}
 
/*------------------------------------------------------------------------------
* strobing the green LEDs
------------------------------------------------------------------------------*/
void strobe_light(UINT16* strobe_state)
{
   #define STROBE_OFF_STATE         0
   #define STROBE_ON_STATE          1
   #define STROBE_LONG_OFF_STATE    2
   
   #define STROBE_ON_TIME           5       // multiple of timer A interrupt
   #define STROBE_OFF_TIME          25
   #define STROBE_LONG_OFF_TIME     450     
   #define NUM_STROBES              1       // will strobe +1 times
   
   static UINT16 delay_time = 0;
   static UINT16 num_strobes = 0;

   delay_time++;
   
   switch (*strobe_state)
   {
   	  case STROBE_OFF_STATE:
   	     if (delay_time - STROBE_OFF_TIME <= 0)
   	     { 
   	        on_green_led();
   	        *strobe_state = STROBE_ON_STATE;
   	        delay_time = 0;
   	     }   	     
   	     break;

   	  case STROBE_ON_STATE:
   	     if (delay_time - STROBE_ON_TIME <= 0)
   	     { 
   	        off_green_led();
   	     	
   	     	if (num_strobes++ > NUM_STROBES)
   	     	{
   	     	   *strobe_state = STROBE_LONG_OFF_STATE;
   	     	   num_strobes = 0;
   	     	}
   	     	else
   	     	{
   	     	   *strobe_state = STROBE_OFF_STATE;
   	     	}
   	     
   	        delay_time = 0;
   	     }   	     
   	     break;

   	  case STROBE_LONG_OFF_STATE:
   	     if (delay_time - STROBE_LONG_OFF_TIME <= 0)
   	     { 
   	        on_green_led();
   	        *strobe_state = STROBE_ON_STATE;
   	        delay_time = 0;
   	     }   	     
   	     break;
   	     
   	  default:
   	     // we shouldn't be here but set the state just in case
         *strobe_state = STROBE_ON_STATE;   	     
   	     break;   	     
   }	
}

/*------------------------------------------------------------------------------
* slowly flashing both LED's
------------------------------------------------------------------------------*/
void flash_both_lights()
{
   #define FLASH_DELAY_TIME     80     
   static UINT16 delay_time = 0;
   delay_time++;
   
   if (delay_time > FLASH_DELAY_TIME)
   {
   	   xor_red_led();
   	   xor_green_led();
   	   delay_time = 0;
   }
}