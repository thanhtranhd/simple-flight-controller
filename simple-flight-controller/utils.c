/*===================================================================
 * Project: simple-flight-controller 
 *
 * Utilies modules: printing, parsing command line inputs, etc.
 * 
 * Copyright (c) 2010 Thanh H Tran (thanhthd@gmail.com)
 *
 *  This file is part of simple-flight-controller project
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

/*------------------------------------------------------------------------------
* Utilities functions.
------------------------------------------------------------------------------*/
#include "utils.h"
#include "types.h"
#include "hal.h"
#include "stdlib.h"

/*------------------------------------------------------------------------------
*
------------------------------------------------------------------------------*/
void printU8(unsigned char val)
{
  char tempStr[3];
  char i;
  for (i = 0; i< 3; i++)
  {
    tempStr[2-i] = (val % 10) + 48;
    val = val / 10;
  }
  tx_string(tempStr,3);
}
/*------------------------------------------------------------------------------
*
------------------------------------------------------------------------------*/
void printU16(UINT16 val)
{
   //unsigned short divider = 0;
   UINT16 i = 0;
   UINT16 j = 0;
   UINT16 tempVal = val;
   UINT16 digith[5] = {1, 10, 100, 1000, 10000};
     
   if (val == 0) 
   {
      //sendLCD('0');
      tx_char('0');
      return;
   }
  
   // deviding is slow & costly
   // manually find the digits should be faster. Avoiding divisions.
   // U16 max value is 65536   
   for (j=5;j>0;j--)
   {
      i = 0;      
      if (tempVal >= digith[j-1]) 
      {
        //while (tempVal > 0) { tempVal = tempVal - digith[j-1]; i++; }
        while (tempVal >= digith[j-1]) { tempVal = tempVal - digith[j-1]; i++; }
      }
      //sendLCD(i+48);
      tx_char(i+48);      
   }
}

/*------------------------------------------------------------------------------
* parse string and get a numeric value that's part of the string
------------------------------------------------------------------------------*/
INT16 parse_value(char* cmdBuffer, unsigned char cmdLen)
{
  UINT16 i = 0;
  UINT16 iIdx = 0;
  UINT16 bIdx = 0;
  char tmpBuf[10];
  
  // xxxxxxx=50000;
  while (i<=cmdLen)
  {
    if (cmdBuffer[i] == '=')
    {
      iIdx = i;
      break;
    }
    i++;          
  }
  
  i = iIdx+1; // '='
  while ((cmdBuffer[i] != ';') && (i<=cmdLen))
  {
    tmpBuf[bIdx++] = cmdBuffer[i];
    i++;
  }
  tmpBuf[bIdx]=0;
  return atoi(tmpBuf);
}



