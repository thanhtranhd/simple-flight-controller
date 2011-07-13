#ifndef BASIC_UTILS_H_
#define BASIC_UTILS_H_

typedef   unsigned int     UINT16;		// 16 bit for MSP430
typedef   int              INT16;
typedef   unsigned char    UINT8;
typedef   char             INT8;
typedef   unsigned long    UINT32;
typedef   long             INT32;

void UART_init();
void putc ( void* p, char c);

void set_8mhz_clock();



#endif /*BASIC_UTILS_H_*/
