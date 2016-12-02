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

#include "msp.h"
#include "driverlib.h"

#define PRINT_STR             string2Lcd
#define PRINT_DIGIT           printU16lcd             


#define RED_LED               (1<<0)     // P2.0
#define GREEN_LED             (1<<1)	 // P2.1
#define BLUE_LED              (1<<2)     // P2.2

#define RX_ROLL               (RX_SW1)   // P1.2
#define RX_PITCH              (1<<5)	 // P1.5

#define RX_THROT              (1<<0)     // P2.0
#define RX_RUDD               (1<<1)     // P2.1
#define RX_TX_GAIN            (1<<2)     // P2.2 - gain setting is done by a signal from TX
                                         //        ex. gyro gain channel controlling the gain

//#define CC2500_GDO0           (1<<6)     // GDO0 of the CC2500 radio
//#define CC2500_GDO2           (1<<7)     // GDO1 of the CC2500 radio

#define P2_IRQ_1              (1<<6)     // P2.6


#define ADC_DECIMATION_BITS   2          // extra 2 bits resolution (4^2) by
                                         // over sampling and decimation
#define ADC_NUMBER_SAMPLES    16         // store all extra samples. This number

#define FLASH_CONFIG_ADDR     0x1000

extern volatile UINT16 sec_ticks;        // increased every second
extern volatile UINT32 ms100_ticks;      // increased every 100mS

extern UINT16   adc_buffer[];
void start_adc(UINT16 adc_channel);



void printU16lcd(unsigned short val);

void tx_string( char* string, int length );
void tx_char( char digit);

void mcu_init();
void ADC_init();

#ifdef TARGET_IS_MSP432P4XX
inline static unsigned short get_current_ticks(void) {return Timer_A_getCounterValue(TIMER_A0_BASE);}
#endif


inline static void xor_green_led() {P2OUT ^= GREEN_LED;}
inline static void xor_red_led()   {P2OUT ^= RED_LED;}
inline static void on_green_led() {P2OUT |= GREEN_LED;}
inline static void on_red_led() {P2OUT |= (RED_LED);}
inline static void on_blue_led() {P2OUT |= (BLUE_LED);}
inline static void off_green_led() {P2OUT &= (~GREEN_LED);}
inline static void off_red_led() {P2OUT &= (~RED_LED);}
inline static void off_blue_led() {P2OUT &= (~BLUE_LED);}

#define PWM_PORT   	          GPIO_PORT_P2

/* P6.6/TA2.4 : pitch channel. P6.7/TA2.3 : roll channel */
#define RX_CAP_ROL_PORT       GPIO_PORT_P6            // roll / pitch capture GPIO port
#define RX_CAP_PIT_PORT       GPIO_PORT_P6            // roll / pitch capture GPIO port
#define RX_CAP_ROL_PIN        GPIO_PIN6               // P6.6
#define RX_CAP_PIT_PIN        GPIO_PIN7               // P6.7

#define RX_CAP_ROL_TMR        TIMER_A2_BASE
#define RX_CAP_PIT_TMR        TIMER_A2_BASE
#define RX_CAP_TMR_INT        INT_TA2_N
#define RX_CAP_ROL_TAIV       TA2IV
#define RX_CAP_PIT_TAIV       TA2IV
#define RX_CAP_ROL_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_3
#define RX_CAP_PIT_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_4


#define TMR_A_PERIOD          5000       // 5ms; servo update frequency; PWM period; 200Hz
                                         // = timer A frequency: 200Hz
#define TICKS_PER_SEC         200
#define TICKS_PER_100_MS      20
#define TICKS_PER_10_MS       2
#define PWM_UPDATE_FREQ       1         // 1*5mS = 5mS => 200Hz


void update_pwm();


inline static void sleep_mcu() {   
#ifndef TARGET_IS_MSP432P4XX
   _BIS_SR(LOW_POWER_MODE + GIE);            // Enter LPM1, enable interrupts
                                             // SMCLK on, MCLK off, CPU off
#else
   MAP_Interrupt_enableMaster();
   MAP_PCM_gotoLPM0();
#endif
}

/*
inline static void wake_mcu() {   
#ifndef TARGET_IS_MSP432P4XX
   _BIC_SR(LOW_POWER_MODE);               
#else

#endif
}
*/
void write_flash(UINT16 address, INT8* buffer, INT8 length);
