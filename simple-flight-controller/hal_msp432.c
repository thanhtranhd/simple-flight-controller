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
	 *      SMCLK = HFXT/2 = 24MHz
	 *      BCLK  = REFO = 32kHz
	 */
	MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
	MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
	MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
	MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
	MAP_CS_initClockSignal(CS_BCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Setting ACLK to REFO 128Khz
//	MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
//  MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}


/*------------------------------------------------------------------------------
* configure hw pins
------------------------------------------------------------------------------*/
void configure_mcu_pins()
{
   // enable TAx OUT on P1x
   // enable TBx OUT on P4x

   
   /* Confinguring radio rx pins as an input and enabling interrupts */
/*   MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, RX_ROLL | RX_PITCH | RX_THROT | RX_RUDD | RX_TX_GAIN);
   MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, RX_ROLL | RX_PITCH | RX_THROT | RX_RUDD | RX_TX_GAIN);
   MAP_GPIO_enableInterrupt(GPIO_PORT_P4, RX_ROLL | RX_PITCH | RX_THROT | RX_RUDD | RX_TX_GAIN);
   MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, RX_ROLL | RX_PITCH | RX_THROT | RX_RUDD | RX_TX_GAIN, GPIO_LOW_TO_HIGH_TRANSITION);
*/
   P1OUT &= ~(RED_LED | GREEN_LED);          // turn off both leds.
   
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
			208, // BRDIV = 2500
			0, // UCxBRF = 0
		    2, // UCxBRS = 0
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
* Timer TA0.1, TA0.2, TA0.3, TA0.4: PWM for quad motors
*
* Timer TA2.3, TA2.4: capture for roll / pitch(P6.6, P6.7)
* Timer TA3.0, TA3.1: capture for throttle / rudder
*
------------------------------------------------------------------------------*/

void timer_init()
{
    /* Configure TimerA0 without using Driverlib */
    TA0CCR0 = TMR_A_PERIOD;                   // PWM Period
    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA0CCR1 = 0;  	                          // CCR1 PWM duty cycle
    TA0CCTL2 = OUTMOD_7;                      // CCR2 reset/set
    TA0CCR2 = 0;                              // CCR2 PWM duty cycle
    TA0CCTL3 = OUTMOD_7;                      // CCR3 reset/set
    TA0CCR3 = 0;                              // CCR3 PWM duty cycle

    TA0CCTL4 = OUTMOD_7;                      // CCR4 reset/set
    TA0CCR4 = 0;                              // CCR4 PWM duty cycle

    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR


    /* Configuring GPIO2.4i,5,6,7 as peripheral output for PWM  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);



    /* Timer_A Continuous Mode Configuration Parameter */
    const Timer_A_ContinuousModeConfig continuousModeConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_24,      // SMCLK/24 = 1MHz
            TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
            TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
    };

    /* Timer_A Capture Mode Configuration Parameter */
    Timer_A_CaptureModeConfig captureModeConfig =
    {
            TIMER_A_CAPTURECOMPARE_REGISTER_3,        // CCCR 3
			TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,   // both Edges
            TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // captureInputSelect
            TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
            TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
    };

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_ROL_PORT,
                                                   RX_CAP_ROL_PIN,
                                                   GPIO_SECONDARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RX_CAP_PIT_PORT,
                                                  RX_CAP_PIT_PIN,
                                                  GPIO_SECONDARY_MODULE_FUNCTION);

    /* Configuring Capture Mode */
    captureModeConfig.captureRegister = RX_CAP_ROL_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_ROL_TMR, &captureModeConfig);

    /* capture another channel */
    captureModeConfig.captureRegister = RX_CAP_PIT_CCCR;
    MAP_Timer_A_initCapture(RX_CAP_PIT_TMR, &captureModeConfig);

    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(RX_CAP_ROL_TMR, &continuousModeConfig);
    MAP_Timer_A_configureContinuousMode(RX_CAP_PIT_TMR, &continuousModeConfig);

    /* Starting the Timer_A2 in continuous mode */
    MAP_Timer_A_startCounter(RX_CAP_ROL_TMR, TIMER_A_CONTINUOUS_MODE);
    MAP_Timer_A_startCounter(RX_CAP_PIT_TMR, TIMER_A_CONTINUOUS_MODE);

    /* enable interrupt */
    MAP_Interrupt_enableInterrupt(RX_CAP_TMR_INT);


    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(24000000);
    MAP_SysTick_enableInterrupt();

    /* Starting the Timer_A0 in continuous mode */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

}

/*******************************************************************************
//This is the TIMERA interrupt vector service routine.
//******************************************************************************/
void TA2_N_IRQHandler(void)
{
	static uint16_t ailTemp=0, pitTemp=0;

	MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);	// blue LED

    switch (RX_CAP_ROL_TAIV)
    {
    	case 0x06:
    		if (ailTemp == 0)
    		{
    			ailTemp = MAP_Timer_A_getCaptureCompareCount(RX_CAP_ROL_TMR, RX_CAP_ROL_CCCR);
    		}
    		else
    		{
    			ail_pulse = MAP_Timer_A_getCaptureCompareCount(RX_CAP_ROL_TMR, RX_CAP_ROL_CCCR) - ailTemp;
    			ailTemp = 0;
    		}

    		MAP_Timer_A_clearCaptureCompareInterrupt(RX_CAP_ROL_TMR, RX_CAP_ROL_CCCR);

            break;

    	case 0x08:
    		if (pitTemp == 0)
    		{
    			pitTemp = MAP_Timer_A_getCaptureCompareCount(RX_CAP_PIT_TMR, RX_CAP_PIT_CCCR);
    		}
    		else
    		{
    			pit_pulse = MAP_Timer_A_getCaptureCompareCount(RX_CAP_PIT_TMR, RX_CAP_PIT_CCCR) - pitTemp;
    			pitTemp = 0;
    		}

    		MAP_Timer_A_clearCaptureCompareInterrupt(RX_CAP_PIT_TMR, RX_CAP_PIT_CCCR);
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------------------------
* update PWM
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

	// Configure clocks
	startCrystalOscillator();
	setSystemClock(48000000);

#if 0
   #if 0
   /* Initialize main clock to 3MHz */
   MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
   MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

   #elif 0

   /* Setting DCO to 24MHz (upping Vcore) */
   FlashCtl_setWaitState(FLASH_BANK0, 2);
   FlashCtl_setWaitState(FLASH_BANK1, 2);
   MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
   CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);

   MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

   #else

   /* Configuring pins for peripheral/crystal usage - 48MHz */
   MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
		   	   	   	   GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

   /* Setting the external clock frequency. This API is optional, but will
   * come in handy if the user ever wants to use the getMCLK/getACLK/etc
   * functions
   */
   CS_setExternalClockSourceFrequency(32000, 48000000);

   /* Starting HFXT in non-bypass mode without a timeout. Before we start
   * we have to change VCORE to 1 to support the 48MHz frequency */
   MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
   MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
   MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

   CS_startHFXT(false);

   /* Initializing MCLK to HFXT (effectively 48MHz) */
   /* using xtal for accurate timing for reading receiver signal */
   MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
   MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
   
#endif
#endif

   // Configure ports -- switch inputs, LEDs, 
   configure_mcu_pins();

   timer_init();
   UART_init();

   ADC_init();   

   /* enable SYS Tick  for clock testing */
   MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);    // red LED - LED#1
   MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);    // blueLED - LED#2


   //start_adc(ADC_ROLL_INCH);
   MAP_Interrupt_enableSleepOnIsrExit();
   MAP_Interrupt_enableMaster();


   __enable_interrupt();  
}

/*------------------------------------------------------------------------------
* Timer A2 0 interrupt service routine
* Timer A0 ticks every TIMER_A period as defined in hal.h. 
------------------------------------------------------------------------------*/
void TA2_0_IRQHandler(void)
{
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
  
      if (rx == 13)
      {
         cmdBuffer[cmdLen-1] = 0;
         woken_up_by |= WOKEN_UP_BY_CMD;
      
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
   
//   _BIS_SR(GIE);                 				// enable interrupts   
}



/*------------------------------------------------------------------------------
// Write value to flash. Copied from MSP430F22xx examples
------------------------------------------------------------------------------*/

void write_flash(UINT16 address, INT8* buffer, INT8 length)
{
   int i = 0;
   char *Flash_ptr = 0;               			// Flash pointer
   
   Flash_ptr = (char*)address;
   
}



void SysTick_Handler()
{
	/* flashing RED LED on GPIO0 */
	MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void TA2_0_IRQ(void)
{
	MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);	// GREEN LED
}
#endif
