/*===================================================================
 * Project: simple-flight-controller 
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
 * along with simple-flight-controller.  If not, see <http://www.gnu.org/licenses/>.
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

