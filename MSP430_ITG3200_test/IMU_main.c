/******************************************************************************
 * 
 * ITG3200 test code for MSP430.
 * This code uses TI's I2c library for I2C communication. 
 * It also uses tiny printf by Kustaa Nyholm 
 * The gyro initialization was based on Fabio's work: 
 * http://www.varesano.net/blog/fabio/homebrew-diy-breakout-board-itg3200-gyroscope-tested-arduino-and-processing
 * The same processing code on Fabio's website can be used with this code.
 * 
 * The program inits the I2C interface, checks to see if the gyro exist on I2C bus,  
 * then reads the gyro data, and prints them out to the UART repeatedly.
 * 
 * No warranty of any kind. 
 *(C) Thanh H Tran (thanhthd@gmail.com)
 * 
*******************************************************************************/
#include "msp430x22x4.h"
#include "TI_USCI_I2C_master.h"
#include "basic_utils.h"
#include "printf.h"


#define I2C_FREQUENCY_SCALE		0x50

#define GYRO_ADDR               0x68 // gyro address, binary = 11101000 when AD0 is connected to GND

#define SMPLRT_DIV              0x15
#define DLPF_FS                 0x16
#define INT_CFG                 0x17
#define PWR_MGM                 0x3E

// ITG3200 Register Defines
#define WHO	    0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x1D
#define	GX_L	0x1E
#define	GY_H	0x1F
#define	GY_L	0x20
#define GZ_H	0x21
#define GZ_L	0x22
#define PWR_M	0x3E


#define GYRO_DATA_ADDR          0x1B
#define NUM_READ                8 // 2 bytes for each axis x, y, z

unsigned char i2c_buffer[40] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0};


unsigned int   itg_temp_data;
int   itg_gyro_x;
int   itg_gyro_y;
int   itg_gyro_z;

unsigned char  itg3200_setup_data[20]={  
 /*****************************************
  * ITG 3200
  * power management set to:
  * clock select = internal oscillator
  *                no reset, no sleep mode
  *                no standby mode
  * sample rate to = 125Hz
  * parameter to +/- 2000 degrees/sec
  * low pass filter = 5Hz
  * no interrupt
  ******************************************/
  
  PWR_MGM, 0x00,
  SMPLRT_DIV, 0x07, // EB, 50, 80, 7F, DE, 23, 20, FF
  DLPF_FS, 0x1E,    // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  INT_CFG, 0x00,
  
  0x00,0x00         // end of data
};


void main(void)
{
  int i = 0;
  int addr = 0;
      
  WDTCTL = WDTPW + WDTHOLD;      // Stop WDT
  set_8mhz_clock();

  UART_init();
  
  init_printf(0, putc);
    
  P1DIR |= (1 << 1);  
  P1DIR |= (1 << 0); 
  P1OUT = 0;
    
  _EINT();


  addr = 1;          
  while (addr<=0x7f)
  { 
    TI_USCI_I2C_transmitinit(addr, I2C_FREQUENCY_SCALE);    // init transmitting with USCI 
    while ( TI_USCI_I2C_notready() );                       // wait for bus to be free
    
    if( TI_USCI_I2C_slave_present(addr++) )
    {
  	   printf("slave I2C device found at: 0x%x\r\n", addr-1);  	  
    }
  }
      
  TI_USCI_I2C_transmitinit(GYRO_ADDR,I2C_FREQUENCY_SCALE);  // init transmitting with USCI 
  while ( TI_USCI_I2C_notready() );                         // wait for bus to be free

  if ( TI_USCI_I2C_slave_present(GYRO_ADDR) )               // slave address may differ from
  {                                                         // initialization  	        
    for (i = 0; i<8; i++)
    {
       printf("program: 0x%x with 0x%x\r\n", itg3200_setup_data[i], 
                                             itg3200_setup_data[i+1]);
       
       TI_USCI_I2C_transmitinit(GYRO_ADDR, I2C_FREQUENCY_SCALE);  // init transmitting  
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free 
       
       TI_USCI_I2C_transmit(2, &itg3200_setup_data[i]);           // start transmitting 
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free
       
       i += 1;
    }
             
    while (1)
    {
       P1OUT ^= (1 << 0);  // blink LED
       	       
       TI_USCI_I2C_transmitinit(GYRO_ADDR, I2C_FREQUENCY_SCALE);  // init transmitting with 
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free
        
       // write the address to bus. 
       i2c_buffer[0] = GYRO_DATA_ADDR;						 
       TI_USCI_I2C_transmit(1, i2c_buffer);       	              // start transmitting 
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free

       TI_USCI_I2C_receiveinit(GYRO_ADDR, I2C_FREQUENCY_SCALE);   // init receiving with USCI 
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free
       
       TI_USCI_I2C_receive(NUM_READ, i2c_buffer);
       while ( TI_USCI_I2C_notready() );                          // wait for bus to be free

       // finish reading all the required bytes
       // constructing the gyro values now
       itg_temp_data = ((i2c_buffer[0] << 8) | i2c_buffer[1]);
       itg_gyro_x    = ((i2c_buffer[2] << 8) | i2c_buffer[3]);
       itg_gyro_y    = ((i2c_buffer[4] << 8) | i2c_buffer[5]);
       itg_gyro_z    = ((i2c_buffer[6] << 8) | i2c_buffer[7]);
         
       printf("%d,%d,%d,%d,", itg_temp_data, itg_gyro_x, itg_gyro_y, itg_gyro_z);
    }
  }
}
