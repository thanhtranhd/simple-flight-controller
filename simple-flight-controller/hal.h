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

#include <msp430x22x4.h>

#define LOW_POWER_MODE        LPM1_bits
#define LCD_SPI_CSN           (1<<6)      // P4.6
#define LCD_CONTROL           P4OUT
#define LCD_CTRL_DIR          P4DIR

#define PRINT_STR             string2Lcd
#define PRINT_DIGIT           printU16lcd             

#define RED_LED               (1<<0)     // P1.0
#define GREEN_LED             (1<<1)	  // P1.1
#define RX_SW1                (1<<2)     // P1.2

#define RX_ROLL               (RX_SW1)   // P1.2
#define RX_PITCH              (1<<5)	  // P1.5

#define RX_THROT              (1<<0)     // P2.0
#define RX_RUDD               (1<<1)     // P2.1
#define RX_TX_GAIN            (1<<2)     // P2.2 - gain setting is done by a signal from TX
                                         //        ex. gyro gain channel controlling the gain

#define CC2500_GDO0           (1<<6)     // GDO0 of the CC2500 radio
#define CC2500_GDO2           (1<<7)     // GDO1 of the CC2500 radio  

#define P2_IRQ_1              (1<<6)     // P2.6

#define TA1_OUT               (1<<6)     // P1.6 
#define TA2_OUT               (1<<7)     // P1.7

#define TB1_OUT               (1<<4)     // P4.4
#define TB2_OUT               (1<<5)     // P4.5

#define P3_SPI_CSN            (1<<0)     // P3.0     
#define P3_SPI_USCIB0_SIMO    (1<<1)     // P3.1
#define P3_SPI_USCIB0_SOMI    (1<<2)     // P3.2
#define P3_SPI_USCIB0_UCLK    (1<<3)     // P3.3

#ifdef USE_BRUSHED_ESC
// define the PWM frequency for brushed motors

// using 2Mhz clock on TAR, so each clock is: 1/2Mhz = 0.0000005 secs = 0.5uS
#define TMR_A_PERIOD          1000       // 1000*0.5uS = 0.5mS; => timer A frequency: 2000Hz

#define TICKS_PER_SEC         2000                                              
#define TICKS_PER_100_MS      200
#define TICKS_PER_10_MS       20
#define PWM_UPDATE_FREQ       4         // 4*0.5uS = 2mS => 500Hz

#else
// define the following for brushless motors
#define TMR_A_PERIOD          5000       // 5ms; servo update frequency; PWM period; 200Hz  
                                         // = timer A frequency: 200Hz
#define TICKS_PER_SEC         200      
#define TICKS_PER_100_MS      20
#define TICKS_PER_10_MS       2

#endif

#define ADC_PITCH_PIN         (1<<3)     // A3, P2.3
#define ADC_YAW_PIN           (1<<4)     // A4, P2.4
#define ADC_ROLL_PIN          (1<<4)     // A12, P4.3, Bit 4 of ADC10AE1

#define ADC_PITCH_INCH        INCH_3     // A3, P2.3
#define ADC_YAW_INCH          INCH_4     // A4, P2.4
#define ADC_ROLL_INCH         INCH_12    // A12, P4.3

#define ADC_DECIMATION_BITS   2          // extra 2 bits resolution (4^2) by
                                         // over sampling and decimation
#define ADC_NUMBER_SAMPLES    16         // store all extra samples. This number
                                         // goes hand in hand with the bits above

#define FLASH_CONFIG_ADDR     0x1000

extern volatile UINT16 sec_ticks;        // increased every second
extern volatile UINT32 ms100_ticks;      // increased every 100mS

extern UINT16   adc_buffer[];
void start_adc(UINT16 adc_channel);

void putCharLCD(unsigned char letter);

void string2Lcd( char* string, int length);
void printU16lcd(unsigned short val);

void tx_string( char* string, int length );
void tx_char( char digit);

void mcu_init();
void ADC_init();

inline static unsigned short get_current_ticks(void) {return TAR;}
inline static void set_p1_rising_edge(unsigned char pin) {P1IES &= ~(pin);}
inline static void set_p1_falling_edge(unsigned char pin) {P1IES |= (pin);}

inline static void set_p2_rising_edge(unsigned char pin) {P2IES &= ~(pin);}
inline static void set_p2_falling_edge(unsigned char pin) {P2IES |= (pin);}

void update_pwm();
inline static void xor_green_led() {P1OUT ^= GREEN_LED;}

inline static void on_green_led() {P1OUT |= GREEN_LED;}
inline static void on_red_led() {P1OUT |= (RED_LED);}             
inline static void off_green_led() {P1OUT &= (~GREEN_LED);}
inline static void off_red_led() {P1OUT &= (~RED_LED);}             


inline static void sleep_mcu() {   
   _BIS_SR(LOW_POWER_MODE + GIE);            // Enter LPM1, enable interrupts
                                             // SMCLK on, MCLK off, CPU off
}

inline static void wake_mcu() {   
   _BIC_SR(LOW_POWER_MODE);               
}

void write_flash(UINT16 address, INT8* buffer, INT8 length);
