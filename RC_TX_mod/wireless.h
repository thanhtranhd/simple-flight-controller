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

#define CC2500_PKG_LENGTH	20

extern INT8   cc2500_tx_buffer[CC2500_PKG_LENGTH];
extern INT8   cc2500_rx_buffer[CC2500_PKG_LENGTH];
extern INT8   cc2500_rx_buffer_len;

extern char RFReceivePacket(char *rxBuffer, char *length);

void wireless_init(void);
void print_wireless_info();


