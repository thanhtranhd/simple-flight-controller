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
#ifdef TARGET_IS_MSP432P4XX

#include "driverlib.h"
#include "types.h"
#include "hal.h"
#include "cmd.h"
#include "captures.h"
#include "quad_main.h"
#include "mixer.h"
#include "utils.h"

#ifdef USE_CC2500
#include "wireless.h"
#endif

volatile UINT16 ms_100_timer=TICKS_PER_100_MS;
volatile UINT16 ms_10_timer = TICKS_PER_10_MS;
volatile UINT16 pwm_rate_ticks = PWM_UPDATE_FREQ;

volatile UINT16 sec_ticks = 0;
volatile UINT32 ms100_ticks = 0;

UINT16   adc_buffer[ADC_NUMBER_SAMPLES];


/* channel 0: roll, 1: pitch, 2: throttle, 3: rudder,
 * 4: gyro - just like a Futaba radio system */
#define  MAX_CAPTURES_HISTORY   25
int channel_captures[5][MAX_CAPTURES_HISTORY];


/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        32000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        3200
};

/*------------------------------------------------------------------------------
* SPI
------------------------------------------------------------------------------*/
#ifndef USE_CC2500
void SPI_init(void)
{

}
#endif


/***********************************************************
  Function:

   The following function is responsible for starting XT1 in the
   MSP432 that is used to source the internal FLL that drives the
   MCLK and SMCLK.
*/
void startCrystalOscillator(void)
{
    /* Configuring pins for peripheral/crystal HFXT*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring pins for peripheral/crystal LFXT*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
}

/***********************************************************
  Function:

   The following function is responsible for setting up the system
   clock at a specified frequency.
*/
void setSystemClock(uint32_t CPU_Frequency)
{
    /* Setting the external clock frequency. This API is optional, but will
     * come in handy if the user ever wants to use the getMCLK/getACLK/etc
     * functions
     */
    MAP_CS_setExternalClockSourceFrequency(32768, CPU_Frequency);

    /* Before we start we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_AM_LDO_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);

    /* Starting HFXT and LFXT in non-bypass mode without a timeout. */
    MAP_CS_startHFXT(false);
    MAP_CS_startLFXT(false);

    /* Initializing the clock source as follows:
     *      MCLK = HFXT = 48MHz
     *      ACLK = LFXT = 32KHz
     *      HSMCLK = HFXT/4 = 12MHz
     *      SMCLK = HFXT/2 = 12MHz
     *      BCLK  = REFO = 32kHz
     */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    MAP_CS_initClockSignal(CS_BCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Setting ACLK to REFO 128Khz
//  MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
//  MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}


/*------------------------------------------------------------------------------
* configure hw pins
------------------------------------------------------------------------------*/
void configure_mcu_pins()
{
   P2OUT = 0;
   P1OUT = 0;
   MAP_GPIO_setAsOutputPin(LED_GPIO_PORT, RED_LED);
   MAP_GPIO_setAsOutputPin(LED_GPIO_PORT, BLUE_LED);
   MAP_GPIO_setAsOutputPin(LED_GPIO_PORT, GREEN_LED);
   MAP_GPIO_setAsOutputPin(LED1_GPIO_PORT, LED1_LED);

   on_red_led();
   off_red_led();

}

/*------------------------------------------------------------------------------
* UART
------------------------------------------------------------------------------*/
void UART_init()
{
    /* UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 115200 baud rate. These
     * values were calculated using the online calculator that TI provides
     * at:
     * http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection
     */
    eUSCI_UART_Config uartConfig =
    {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
            104, // BRDIV
            0, // UCxBRF
            1, // UCxBRS
            EUSCI_A_UART_NO_PARITY, // No Parity
            EUSCI_A_UART_LSB_FIRST, // LSB First
            EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
            EUSCI_A_UART_MODE, // UART mode
            EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION // Low Frequency Mode
    };

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}

/*------------------------------------------------------------------------------
* Timer TA3.0, TA3.1: capture for throttle / rudder (P10.4, P10.5)
* Timer TA3.2, TA3.3, TA3.4 : capture for gyro, roll, pitch channels respecctively (P8.2, P9.2, P9.3)
------------------------------------------------------------------------------*/
static void timer_capture_init()
{
    /* Timer_A Continuous Mode Configuration Parameter */
    const Timer_A_ContinuousModeConfig continuousModeConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_12,      // SMCLK/12 = 1MHz
            TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
            TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
    };

    /* Timer_A Capture Mode Configuration Parameter */
    Timer_A_CaptureModeConfig captureModeConfig =
    {
            TIMER_A_CAPTURECOMPARE_REGISTER_3,        // CCCR 3
            TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,   // both Edges
            TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // captureInputSelect - B doesn't work
            TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
            TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
    };

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_THR_PORT,
                                                   RX_CAP_THR_PIN,
                                                   GPIO_SECONDARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_ROL_PORT,
                                                   RX_CAP_ROL_PIN,
                                                   GPIO_SECONDARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_PIT_PORT,
                                                  RX_CAP_PIT_PIN,
                                                  GPIO_SECONDARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_RUD_PORT,
                                                  RX_CAP_RUD_PIN,
                                                  GPIO_SECONDARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_GYR_PORT,
                                                  RX_CAP_GYR_PIN,
                                                  GPIO_SECONDARY_MODULE_FUNCTION);

    /* Configuring Capture Mode */
    captureModeConfig.captureRegister = RX_CAP_THR_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_THR_TMR, &captureModeConfig);

    /* roll channel capture */
    captureModeConfig.captureRegister = RX_CAP_ROL_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_ROL_TMR, &captureModeConfig);

    /* pitch channel capture */
    captureModeConfig.captureRegister = RX_CAP_PIT_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_PIT_TMR, &captureModeConfig);

    /* rudder channel capture */
    captureModeConfig.captureRegister = RX_CAP_RUD_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_RUD_TMR, &captureModeConfig);

    /* gyro channel capture */
    captureModeConfig.captureRegister = RX_CAP_GYR_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_GYR_TMR, &captureModeConfig);

    /* Configuring timer Continuous Mode  -
     * all capture channels are on Timer A3 so only configure Timer A3 */
    MAP_Timer_A_configureContinuousMode(RX_CAP_ROL_TMR, &continuousModeConfig);

    /* Starting the Timer_A3 in continuous mode */
    MAP_Timer_A_startCounter(RX_CAP_ROL_TMR, TIMER_A_CONTINUOUS_MODE);

    /* enable interrupt - CCR0 is on TA0 interrupt while other CCRs on TA3 are on TA3N interrupt */
    MAP_Interrupt_enableInterrupt(RX_CAP_THR_TMR_INT);
    MAP_Interrupt_enableInterrupt(RX_CAP_ROL_TMR_INT);
}

/*------------------------------------------------------------------------------
* Timer TA0.1, TA0.2, TA0.3, TA0.4: PWM for quad motors
* TA0.0 is in UP mode.
* Effective frequency is SMCLK/12 = 12MHz / 12 = 1MHz
* ------------------------------------------------------------------------------*/
static void timer_pwm_init()
{
    /* Configuring GPIO2.4i,5,6,7 as peripheral output for PWM  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    /* Configure TimerA0 without using Driverlib */
    TA0CCTL0 &= ~CCIFG;
    TA0CCTL0 = OUTMOD_7 | CCIE;                          // TACCR0 interrupt enabled
    TA0CCR0 = TMR_A_PERIOD;                   // PWM Period

    TA0CTL = TASSEL__SMCLK |                  // SMCLK
             MC__UP |                         // up mode
             ID_2;                            // divided by 4

    TA0EX0 = TIMER_A_EX0_TAIDEX_2;            // input divider extra of 3. so a total of /12 is applied

    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA0CCR1 = 0;                              // CCR1 PWM duty cycle

    TA0CCTL2 = OUTMOD_7;                      // CCR2 reset/set
    TA0CCR2 = 0;                              // CCR2 PWM duty cycle

    TA0CCTL3 = OUTMOD_7;                      // CCR3 reset/set
    TA0CCR3 = 0;                              // CCR3 PWM duty cycle

    TA0CCTL4 = OUTMOD_7;                      // CCR4 reset/set
    TA0CCR4 = 0;                              // CCR4 PWM duty cycle

}

/*------------------------------------------------------------------------------
*
------------------------------------------------------------------------------*/
void timer_init()
{

    timer_capture_init();

    timer_pwm_init();

    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(24000000);
    MAP_SysTick_enableInterrupt();
}


/*******************************************************************************
 * This is the TIMERA_3_0 interrupt vector service routine.
 *
 * the input signal looks like this:
 *
 *                +-----+                                     +----+
 *                |     |                                     |    |
 * ---------------+     +-------------------------------------+    +-------------
 *
 * we need to capture the length of the high pulse only - which should be 900-2300
 * The low end can vary
 *
 * So start the capture only when we see the rising edge and take the time stamps
 * between the rising and fallin.
 *
 * The interrupt happens for both rising and falling edges. So make sure we start
 * the timestamp keeping only when signal rises then falls
 *
 *
 ******************************************************************************/
void TA3_0_IRQ(void)
{
    static int32_t thr_temp=0, tidx=0;
    int32_t val=0, tmp=0;

    tmp = RX_CAP_THR_CAP;

    channel_captures[THR_CAP_IDX][tidx++] = tmp;
    if (tidx>=MAX_CAPTURES_HISTORY) tidx=0;

    /* find out if we get here because of the raising or falling edge */
    if ((RX_CAP_THR_CCTL & TIMER_A_CCTLN_CCI) == 0)
    {
        /* we are in low level now  - must have gotten here by the falling edge */
        if (thr_temp == 0)
        {
            /* we started the capture at wrong phase - skip this capture */
            goto capture_end;
        }
    }

    if (thr_temp == 0)
    {
        thr_temp = tmp;
    }
    else
    {
        val = tmp  - thr_temp;

        if (val<0)
        {
            /*
             * case of when second capture has smaller value than previous capture
             * timer rolls over (16 bit), which happens very often       *
            */
            val = val + 0xffff;
        }

        TAKE_VALID_THR_CAPTURE(val, thr_pulse);

        thr_temp = 0;
    }

capture_end:

    RX_CAP_THR_CCTL &= ~(TIMER_A_CCTLN_COV); /* clear capture overflow if it happens */

    RX_CAP_THR_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */

}

/*******************************************************************************
 * This is the TIMERA_3_n interrupt vector service routine.
 * the input signal looks like this:
 *
 *                +-----+                                     +----+
 *                |     |                                     |    |
 * ---------------+     +-------------------------------------+    +-------------
 *
 * we need to capture the length of the high pulse only - which should be 900-2300
 * The low end can vary
 *
 * So start the capture only when we see the rising edge and take the time stamps
 * between the rising and fallin.
 *
 * The interrupt happens for both rising and falling edges. So make sure we start
 * the timestamp keeping only when signal rises then falls
 *
 */
//******************************************************************************/
void TA3_N_IRQ(void)
{
    static int32_t roll_temp=0, pitch_temp=0, rudder_temp=0, gyr_temp=0;
    static uint8_t  aidx = 0, eidx=0, ridx=0, gidx=0;
    int32_t val=0, atmp, ptmp, rtmp, gtmp;

    switch (RX_CAP_ROL_TAIV)
    {
        case RX_CAP_ROL_CCIFG:

            atmp = RX_CAP_ROL_CAP;

            channel_captures[AIL_CAP_IDX][aidx++] = atmp;
            if (aidx>=MAX_CAPTURES_HISTORY) aidx=0;

            if ((RX_CAP_ROL_CCTL & TIMER_A_CCTLN_CCI) == 0)
            {
                /* we are in low level now  - got here by the falling edge */
                if (roll_temp  == 0)
                {
                    /* we started the capture at wrong phase - skip this capture */

                    RX_CAP_ROL_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */
                    break;
                }
            }

            if (roll_temp == 0)
            {
                roll_temp = atmp;
            }
            else
            {
                val = atmp - roll_temp;
                if (val<0) val+=0xffff;

                roll_temp = 0;

                /* don't take the captures when timer overflows */
                TAKE_VALID_CAPTURE(val, ail_pulse);
            }

            RX_CAP_ROL_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */

            break;

        case RX_CAP_PIT_CCIFG:

            ptmp = RX_CAP_PIT_CAP;

            channel_captures[ELV_CAP_IDX][eidx++] = ptmp;
            if (eidx>=MAX_CAPTURES_HISTORY) eidx=0;

            if ((RX_CAP_PIT_CCTL & TIMER_A_CCTLN_CCI) == 0)
            {
                /* we are in low level now  - got here by the falling edge */
                if (pitch_temp  == 0)
                {
                    /* we started the capture at wrong phase - skip this capture */

                    RX_CAP_PIT_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */
                    break;
                }
            }

            if (pitch_temp == 0)
            {
                pitch_temp = ptmp;
            }
            else
            {
                val = ptmp - pitch_temp;

                if (val<0) val+=0xffff;

                /* take valid values only - timer seems to glitch from time to time */
                TAKE_VALID_CAPTURE(val, pit_pulse);

                pitch_temp = 0;
            }

            RX_CAP_PIT_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */

            break;

        case RX_CAP_RUD_CCIFG:

            rtmp = RX_CAP_RUD_CAP;

            channel_captures[RUD_CAP_IDX][ridx++] = rtmp;
            if (ridx>=MAX_CAPTURES_HISTORY) ridx=0;

            if ((RX_CAP_RUD_CCTL & TIMER_A_CCTLN_CCI) == 0)
            {
                /* we are in low level now  - got here by the falling edge */
                if (rudder_temp  == 0)
                {
                    /* we started the capture at wrong phase - skip this capture */

                    RX_CAP_RUD_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */
                    break;
                }
            }

            if (rudder_temp == 0)
            {
                rudder_temp = rtmp;
            }
            else
            {
                val = rtmp - rudder_temp;
                if (val<0) val+=0xffff;

                /* don't take the captures when timer overflows */
                TAKE_VALID_CAPTURE(val, rud_pulse);

                rudder_temp = 0;
            }

            RX_CAP_RUD_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */

            /* clear overflow flag if any */
            RX_CAP_RUD_CCTL &= ~(TIMER_A_CCTLN_COV);

            break;

        case RX_CAP_GYR_CCIFG:
            gtmp = RX_CAP_GYR_CAP;

            channel_captures[GYR_CAP_IDX][gidx++] = gtmp;
            if (gidx>=MAX_CAPTURES_HISTORY) gidx=0;

            if ((RX_CAP_GYR_CCTL & TIMER_A_CCTLN_CCI) == 0)
            {
                /* we are in low level now  - got here by the falling edge */
                if (gyr_temp  == 0)
                {
                    /* we started the capture at wrong phase - skip this capture */

                    RX_CAP_GYR_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */
                    break;
                }
            }

            if (gyr_temp == 0)
            {
                gyr_temp = gtmp;
            }
            else
            {
                val = gtmp - gyr_temp;
                if (val<0) val+=0xffff;

                gyr_temp = 0;

                /* don't take the captures when timer overflows */
                TAKE_VALID_CAPTURE(val, tx_gain_pulse);
            }

            RX_CAP_GYR_CCTL &= ~(TIMER_A_CCTLN_CCIFG); /* clear interrupt flag */

            break;

        default:
            /* either no interrupt or timer overflow. */
            break;
    }
}


void TA2_N_IRQHandler(void)
{
}

/*------------------------------------------------------------------------------
* update PWM
*
* 4 Motors are on TA0.1, TA0.2, TA0.3, TA0.4
------------------------------------------------------------------------------*/
void update_pwm()
{
   if (!(mixer_flags & MIXER_DISC_INPUT_ON)) 
   {
      mix_mixing_quad  ((INT16)ail_pulse, (INT16)pit_pulse, 
                        (INT16)thr_pulse, (INT16)rud_pulse, 
                        (INT16*)&front_motor,  (INT16*)&back_motor,  
                        (INT16*)&left_motor,  (INT16*)&right_motor);
   
      TA0CCR1 = front_motor;
      TA0CCR2 = back_motor;
      
      TA0CCR3 = left_motor;
      TA0CCR4 = right_motor;
   }
   else
   {
      TA0CCR1 = 0;
      TA0CCR2 = 0;
      
      TA0CCR3 = 0;
      TA0CCR4 = 0;
   }
}

/*------------------------------------------------------------------------------
*
------------------------------------------------------------------------------*/
void mcu_init()
{
   /* Halting WDT and disabling master interrupts */
   MAP_WDT_A_holdTimer();
   MAP_Interrupt_disableMaster();

    // Configure clock
   startCrystalOscillator();
   setSystemClock(48000000);

   // Configure ports -- switch inputs, LEDs, 
   configure_mcu_pins();

   timer_init();
   UART_init();

   ADC_init();   


   //start_adc(ADC_ROLL_INCH);
   MAP_Interrupt_enableSleepOnIsrExit();
   MAP_Interrupt_enableMaster();


   __enable_interrupt();  
}

/*------------------------------------------------------------------------------
* Timer A0 interrupt service routine
* Timer A0 ticks every TIMER_A period as defined in hal.h. 
------------------------------------------------------------------------------*/
void TA0_0_IRQHandler(void)
{
   xor_led1_led();

   if (!ms_100_timer)
   {
      ms_100_timer = TICKS_PER_100_MS;
      ms100_ticks++;
   }
   ms_100_timer--;

   if (!ms_10_timer)
   {
      ms_10_timer = TICKS_PER_10_MS;
   }
   ms_10_timer--;

   if (!pwm_rate_ticks)
   {
      pwm_rate_ticks = PWM_UPDATE_FREQ;
#ifdef USE_BRUSHED_ESC
      woken_up_by |= WOKEN_UP_BY_TIMER;
      __bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
#endif
   }
   pwm_rate_ticks--;

#ifndef USE_BRUSHED_ESC
   // wake up every timer ticks if we are not doing high speed PWM
   // as in brushed ESC case.
   woken_up_by |= WOKEN_UP_BY_TIMER;
   //__bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
#endif

   TA0CCTL0 &= ~CCIFG;
}


/*------------------------------------------------------------------------------
* PORT1 pin change ISR
------------------------------------------------------------------------------*/
void PORT1_IRQHandler(void)
{
}


/*------------------------------------------------------------------------------
// PORT 2 ISR 
------------------------------------------------------------------------------*/
void PORT2_IRQHandler(void)
{

#ifdef USE_CC2500   
#warn "!!! code built for built-in RX - using CC2500!!!"
   if (P2IFG & CC2500_GDO0)
   {
      // Fetch packet from CCxxxx & check the CRC==0x80 (OK)
      if (0x80 == RFReceivePacket(cc2500_rx_buffer, &cc2500_rx_buffer_len) &&
          (cc2500_rx_buffer_len == 11)) // TX module sent 10 bytes for 5 PWM values plus 1 byte address.   
      {
         // the following channels assignment is Spektrum / JR style 
         // which has the first channel in the PPM stream being the throttle
         // where as Futaba & Hitec use first channel as aileron.        
         thr_pulse = (cc2500_rx_buffer[1] << 8) | cc2500_rx_buffer[2];
         ail_pulse = (cc2500_rx_buffer[3] << 8) | cc2500_rx_buffer[4];
         pit_pulse = (cc2500_rx_buffer[5] << 8) | cc2500_rx_buffer[6];
         rud_pulse = (cc2500_rx_buffer[7] << 8) | cc2500_rx_buffer[8];
         tx_gain_pulse = (cc2500_rx_buffer[9] << 8) | cc2500_rx_buffer[10];
                 
         //woken_up_by |= WOKEN_UP_BY_WIRELESS;
         //__bic_SR_register_on_exit(LOW_POWER_MODE);   // wake CPU
      }
      P2IFG &= ~(CC2500_GDO0);               //Clr flag that caused int      
   }   
#endif   
}


/*------------------------------------------------------------------------------
* send a string to UART ....
------------------------------------------------------------------------------*/
void tx_string( char* string, int length )
{
  int pointer;
  char c;

  for( pointer = 0; pointer < length; pointer++)
  {
    c = string[pointer];
    MAP_UART_transmitData(EUSCI_A0_BASE, c);
  }
}

/*------------------------------------------------------------------------------
* send a char to UART ....
------------------------------------------------------------------------------*/
void tx_char( char digit)
{
    MAP_UART_transmitData(EUSCI_A0_BASE, digit);
}


/*------------------------------------------------------------------------------
* USCIA interrupt service routine
* UART receiving a character.
------------------------------------------------------------------------------*/
/* EUSCI A0 UART ISR */
void EUSCIA0_IRQHandler(void)
{
   uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
   char rx = ' ';

   MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

   if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
   {

      rx = MAP_UART_receiveData(EUSCI_A0_BASE); //UCA0RXBUF
   
      if (cmdLen < CMD_MAX_LEN)
      {
         cmdBuffer[cmdLen++] = rx;
      }
      else
      {
         cmdBuffer[cmdLen] = 0;
      }
  

      if (rx == ENTER_KEY_CODE)
      {
          if ((cmdLen <= 1) && (last_cmd_len > 0))
          {
              cmdLen = last_cmd_len;
              memcpy(cmdBuffer, last_cmd_buff, last_cmd_len);
              tx_string(cmdBuffer, cmdLen);
          }
          else
          {
              cmdBuffer[cmdLen-1] = 0;
              woken_up_by |= WOKEN_UP_BY_CMD;
          }
         //wake_mcu();
         //__bic_SR_register_on_exit(LOW_POWER_MODE);
      }

      tx_string(&rx,1); // might need to bring this print out of the interrupt!
   }
}


/*------------------------------------------------------------------------------
* ADC14 interrupt service routine
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
* ADC init
------------------------------------------------------------------------------*/
void ADC_init()
{
}

/*------------------------------------------------------------------------------
// start ADC
------------------------------------------------------------------------------*/
void start_adc(UINT16 adc_channel)
{
   
//   _BIS_SR(GIE);                              // enable interrupts   
}



/*------------------------------------------------------------------------------
// Write value to flash. Copied from MSP430F22xx examples
------------------------------------------------------------------------------*/

void write_flash(UINT16 address, INT8* buffer, INT8 length)
{
   int i = 0;
   char *Flash_ptr = 0;                         // Flash pointer
   
   Flash_ptr = (char*)address;
   
}



void SysTick_Handler()
{
    /* flashing RED LED on GPIO0 */
    //MAP_GPIO_toggleOutputOnPin(LED1_GPIO_PORT, LED1_LED);
    off_blue_led();
    off_red_led();
}

void TA2_0_IRQ(void)
{
    MAP_GPIO_toggleOutputOnPin(LED_GPIO_PORT, GREEN_LED);
}
#endif
