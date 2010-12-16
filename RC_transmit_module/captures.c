/*===================================================================
 * Project: cc2500-rc-ppm-tx
 *
 * Capture events receiver signals and measure their timing
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
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include "types.h"
#include "captures.h"
#include "hal.h"

//------------------------------------------------------------------
void init_ccb(CAPTURE_CCB* ccb)
{
   ccb->tick1= 0;
   ccb->state = CAP_LOW;
   ccb->ch_idx = 0; 
}
//------------------------------------------------------------------

//------------------------------------------------------------------
// this function is called when the GPIO P1 interrupt is triggered.
//------------------------------------------------------------------
void capture_timing(UINT8 pin, CAPTURE_CCB* ccb, UINT16* time)
{
   register UINT16 tick2;
   
   switch (ccb->state)
   {               
      case CAP_LOW:
         // pulse is currently low, but got intr because it rises;
         ccb->tick1 = get_current_ticks();
         set_p1_falling_edge(pin);
                         
         ccb->state = CAP_HIGH;
         break;     
         
      case CAP_HIGH:
         // pulse is currently high but got intr because it falls.
         // capture the timing now, and set the intr edge to raising edge.
         tick2 = get_current_ticks();      
         if (tick2 > ccb->tick1)
         {
            *time = tick2 - ccb->tick1;
         }
         else
         {
            *time = TMR_A_PERIOD - ccb->tick1 + tick2;
         }
         
         set_p1_rising_edge(pin);
         
         ccb->state = CAP_LOW;
         break;
         
   default:
         break;
   }
}

//------------------------------------------------------------------
// this function is called when the GPIO interrupt on P2 happens
// used temporarily since there is no more P1 pins available.
//------------------------------------------------------------------
void capture_p2_timing(UINT8 pin, CAPTURE_CCB* ccb, UINT16* time)
{
   register UINT16 tick2;
   
   switch (ccb->state)
   {               
      case CAP_LOW:
         // pulse is currently low, but got intr because it rises;
         ccb->tick1 = get_current_ticks();
         set_p2_falling_edge(pin);
                         
         ccb->state = CAP_HIGH;
         break;     
         
      case CAP_HIGH:
         // pulse is currently high but got intr because it falls.
         // capture the timing now, and set the intr edge to raising edge.
         tick2 = get_current_ticks();
         
         if (tick2 > ccb->tick1)
         {
            *time = tick2 - ccb->tick1;
         }
         else
         {
            *time = TMR_A_PERIOD - ccb->tick1 + tick2;
         }
         
         set_p2_rising_edge(pin);
         
         ccb->state = CAP_LOW;
         break;
         
   default:
         break;
   }
}
//------------------------------------------------------------------
// this function is called when the GPIO interrupt on P2 happens
// it captures the PPM pulses on the PPM input pin and puts
// each pulse length into the times array.
// PPM pulses are active low. The pin is normally high.
// Bellow is what PPM looks like on an oscilloscope:
//
//  "0v" level    st 1     2    3    4     5     6   st      
// --------------| |---| |---| |--| |--| |---| |----| |-----------
//               | |   | |   | |  | |  | |   | |    | |
//  0.5v level   ---   ---   ---  ---  ---   ---    ---
//
// The above signal is not compatible with microcontroller input. The MCU
// can't decides which is 0 and which is 1. It needs to feed through a transistor to 
// clip and flip the pulses so that we have proper logic levels. The stream becomes:
//                st
//  "1"           ---   ---   ---  ---  ---   ---    ---
//                | |   | |   | |  | |  | |   | |    | |
//        synch   | |   | |   | |  | |  | |   | |    | |
// - "0"  --------| |---| |---| |--| |--| |---| |----| |-----------
//                  1     2     3    4    5     6
//
// synch pulse will be the long pulses between the pulse brust.
// each pulse #1 or #2, or ... #6 will have length from 1000-2500
// more info on the PPM stream: http://www.mftech.de/ppm_en.htm
//
// this function is called every PPM pulses. When it finishes captures
// all pulses, it will return number of frame. Else it returns 0.
//------------------------------------------------------------------
#define SYNCH_PULSE_LENGTH	2300

UINT16 capture_p2_ppm(UINT8 pin, CAPTURE_CCB* ccb, UINT16* times)
{
   register UINT16 tick2;
   register UINT16 time;	
   UINT16	num_pulses = 0;
   
   switch (ccb->state)
   {
      case CAP_LOW:
         // ppm line is currently low, but got intr because it rises;         
         ccb->tick1 = get_current_ticks();
                         
         ccb->state = CAP_HIGH;
         break;     

      case CAP_HIGH:
         // ppm line is currently low but got intr because it rises.
         // capture the timing now and ignore the falling signal 
         // but continue to wait until next rise.
         tick2 = get_current_ticks();
         
         if (tick2 > ccb->tick1)
         {
            time = tick2 - ccb->tick1;
         }
         else
         {
            time = TMR_A_PERIOD - ccb->tick1 + tick2;
         }
                  
         if (time >= SYNCH_PULSE_LENGTH)
         {
            // the pulse we have measured is actually a synch pulse.
            // We have finished getting all pulses. Note the number of pulses
            // and go back to measure next frame
            if (ccb->ch_idx!= 0) num_pulses = ccb->ch_idx;
            ccb->ch_idx = 0;            
         }
         else
         {
            // the pulse is a component of PPM pulse. Log it.
            times[ccb->ch_idx] = time;
            ccb->ch_idx++;
            
            num_pulses = 0;
         }

         ccb->tick1 = tick2;           // reset the timing.
         ccb->state = CAP_HIGH;	       // stay in the same state.

         break;         
         
   default:
         break;
   }
   
   return num_pulses;
}

