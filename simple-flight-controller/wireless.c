/*===================================================================
 * Project: simple-flight-controller 
 *
 * Wireless modules: Interfacing with TI's CC2500 example codes (SLAA325).
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
 * along with Simple-Flight-Controller.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#ifdef USE_CC2500

#include "TI_CC_Radio/include.h"
#include "types.h"
#include "hal.h"
#include "utils.h"
#include "wireless.h"


extern char paTable[];
extern char paTableLen;

INT8 cc2500_tx_buffer[CC2500_PKG_LENGTH];
INT8 cc2500_rx_buffer[CC2500_PKG_LENGTH];
INT8 cc2500_rx_buffer_len=CC2500_PKG_LENGTH; 

//--------------------------------------------------------------------------
// call several set up functions responsible for setting up the CC2500 radio
// most if not all of this code is taken from TI's SLAA325 app. note
//--------------------------------------------------------------------------
void wireless_init(void)
{  	    
   P2SEL &= ~(CC2500_GDO0);					 // allow interrupt on GDO0

   TI_CC_SPISetup();                         // Initialize SPI port
                                             // some how this function
                                             // causes interrupt on GDO0 if the 
                                             // GDO0 interrupt is enabled.
  
   TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
   writeRFSettings();                        // Write RF settings to config reg
   TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE
  

   //PARTNUM: 128 VERSION: 003 RSSI: 199 - GDO2: 011 - GDO1: 046 - GDO0: 006  
   //
   TI_CC_GDO0_PxIES |= CC2500_GDO0;          // Int on falling edge (end of pkt)
   TI_CC_GDO0_PxIFG &= ~CC2500_GDO0;         // Clear flag
   TI_CC_GDO0_PxIE |= CC2500_GDO0;           // Enable int on end of packet
 
   TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                             // When a pkt is received, it will
                                             // signal on GDO0 and wake CPU
}



#endif
