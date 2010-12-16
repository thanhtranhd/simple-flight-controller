/*===================================================================
 * Project: simple-flight-controller 
 *
 * Command Line Processing module
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

#include <string.h>
#include "types.h"
#include "cmd.h"
#include "captures.h"

#include "utils.h"
#include "hal.h"
#include "tx_main.h"

#include "wireless.h"

char cmdBuffer[CMD_MAX_LEN];
unsigned char cmdLen = 0;

//-------------------------------------------------------------------
// input: the string contain the command and its length
//-------------------------------------------------------------------
void process_cmd(char* cmd_str, unsigned char cmd_length)
{
   if (strcmp(cmd_str,"get_pulse") == 0)
   {
   	  show_info();      
   }
   else
   {
   	  tx_string("error!\n\r",8);
   	  tx_string(cmd_str, cmd_length);
   	  tx_string("\n\r",2);
   }
}


/*------------------------------------------------------------------------------
// show various data: rx pulses, motor outputs, stats
------------------------------------------------------------------------------*/
void show_info()
{
   UINT16 i = 0;
   tx_string("\n\r-----------",13);
   tx_string("\n\rnum ppm pulse = ",17);
   printU16(num_ppm_pulses);      
   
   tx_string("\n\r-----------\n\r",15);
   for (i=0;i<num_ppm_pulses;i++)
   {
   	  tx_string("ch ", 3);
   	  printU8(i+1);
   	  tx_string(" = ", 3);
   	  printU16(ppm_pulses[i]);
   	  tx_string("\n\r",2);
   }

}

