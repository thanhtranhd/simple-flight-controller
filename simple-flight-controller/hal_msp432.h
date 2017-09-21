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


#define RED_LED               GPIO_PIN0
#define GREEN_LED             GPIO_PIN1
#define BLUE_LED              GPIO_PIN2
#define LED_GPIO_PORT         GPIO_PORT_P2

#define LED1_LED              GPIO_PIN0
#define LED1_GPIO_PORT        GPIO_PORT_P1


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
inline static void xor_blue_led()   {P2OUT ^= BLUE_LED;}
inline static void xor_led1_led()   {P1OUT ^= LED1_LED;}

inline static void on_green_led() {P2OUT |= GREEN_LED;}
inline static void on_red_led() {P2OUT |= (RED_LED);}
inline static void on_blue_led() {P2OUT |= (BLUE_LED);}
inline static void off_green_led() {P2OUT &= (~GREEN_LED);}
inline static void off_red_led() {P2OUT &= (~RED_LED);}
inline static void off_blue_led() {P2OUT &= (~BLUE_LED);}

#define PWM_PORT   	          GPIO_PORT_P2


/*
 * Using PPM receiver options:
 *
 * The following are capture pins for all the input signals from
 * radio receiver (roll pitch throttle rudder and gyro gain)
 *
 * all capture signals are using Timer TA3
 *
 * Will support newer receiver or WIFI later. If we do we could free up
 * a lot of GPIO pins.
 *
 * */

/* throttle channel P10.4 */
/* CCR0 interrupt is in TA3_0 interrupt handler */
#define RX_CAP_THR_PORT       GPIO_PORT_P10
#define RX_CAP_THR_PIN        GPIO_PIN4
#define RX_CAP_THR_TMR        TIMER_A3_BASE
#define RX_CAP_THR_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_0
#define RX_CAP_THR_TMR_INT    INT_TA3_0
#define RX_CAP_THR_CCTL	      TA3CCTL0
#define RX_CAP_THR_CAP	      TA3CCR0


/* roll channel P9.2 */
#define RX_CAP_ROL_PORT       GPIO_PORT_P9
#define RX_CAP_ROL_PIN        GPIO_PIN2
#define RX_CAP_ROL_TMR        TIMER_A3_BASE
#define RX_CAP_ROL_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_3
#define RX_CAP_ROL_TAIV       TA3IV
#define RX_CAP_ROL_TMR_INT    INT_TA3_N
#define RX_CAP_ROL_CCIFG      0x06	/* TAIV value for CCR3 CCIF - section 17.3.5 */
#define RX_CAP_ROL_CCTL	      TA3CCTL3
#define RX_CAP_ROL_CAP	      TA3CCR3


/* pitch channel P9.3 */
#define RX_CAP_PIT_PORT       GPIO_PORT_P9
#define RX_CAP_PIT_PIN        GPIO_PIN3
#define RX_CAP_PIT_TMR        TIMER_A3_BASE
#define RX_CAP_PIT_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_4
#define RX_CAP_PIT_TAIV       TA3IV
#define RX_CAP_PIT_TMR_INT    RX_CAP_ROL_TMR_INT
#define RX_CAP_PIT_CCIFG      0x08	/* TAIV value for CCR4 CCIF - section 17.3.5 */
#define RX_CAP_PIT_CCTL	      TA3CCTL4
#define RX_CAP_PIT_CAP	      TA3CCR4

/* rudder channel P10.5 */
#define RX_CAP_RUD_PORT       GPIO_PORT_P10
#define RX_CAP_RUD_PIN        GPIO_PIN5
#define RX_CAP_RUD_TMR        TIMER_A3_BASE
#define RX_CAP_RUD_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_1
#define RX_CAP_RUD_TAIV       TA3IV
#define RX_CAP_RUD_TMR_INT    RX_CAP_ROL_TMR_INT
#define RX_CAP_RUD_CCIFG      0x02	/* TAIV value for CCR1 CCIF - section 17.3.5 */
#define RX_CAP_RUD_CCTL	      TA3CCTL1
#define RX_CAP_RUD_CAP	      TA3CCR1

/* gyro gain channel P8.2 */
#define RX_CAP_GYR_PORT       GPIO_PORT_P8
#define RX_CAP_GYR_PIN        GPIO_PIN2
#define RX_CAP_GYR_TMR        TIMER_A3_BASE
#define RX_CAP_GYR_CCCR       TIMER_A_CAPTURECOMPARE_REGISTER_2
#define RX_CAP_GYR_TAIV       TA3IV
#define RX_CAP_GYR_TMR_INT    RX_CAP_ROL_TMR_INT
#define RX_CAP_GYR_CCIFG      0x04	/* TAIV value for CCR2 CCIF - section 17.3.5 */
#define RX_CAP_GYR_CCTL	      TA3CCTL2
#define RX_CAP_GYR_CAP	      TA3CCR2

#define AIL_CAP_IDX           0
#define ELV_CAP_IDX           1
#define THR_CAP_IDX           2
#define RUD_CAP_IDX           3
#define GYR_CAP_IDX           4

#define RX_MIN_PULSE_VAL      900
#define RX_MAX_PULSE_VAL	  2300

#define TAKE_VALID_CAPTURE(pulse, capture) \
	if (((pulse) >=RX_MIN_PULSE_VAL) && ((pulse)<=RX_MAX_PULSE_VAL))\
	{                                                          \
		capture = pulse;                                       \
	}                                                          \
	else \
	{    \
		xor_red_led(); \
	}

#define TAKE_VALID_THR_CAPTURE(pulse, capture) \
	if (((pulse) >=RX_MIN_PULSE_VAL) && ((pulse)<=RX_MAX_PULSE_VAL))\
	{                                                          \
		capture = pulse;                                       \
	}                                                          \
	else \
	{    \
		xor_blue_led(); \
	}

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
