/*
 * Created 4/8/2020 
 * Author: ramez 
 * Description: Mostly for RS232 configuration, reading and writing. 
 * UART on SERCOM4 is connected to PB08-Tx and PB09-Rx.
 * A 3.3V signal level to USB cable is used to connect PB08/PB09
 * to the host PC COM port.
 * Use TeraTerm on the host PC to send keystrokes and observe the display
 * of the characters echoed.
 *
 * By default, the clock is running at 1 MHz.
 */

#include "saml21j18b.h"

unsigned char* ARRAY_PORT_PINCFG1 = (unsigned char*)&REG_PORT_PINCFG1;
unsigned char* ARRAY_PORT_PMUX1 = (unsigned char*)&REG_PORT_PMUX1;


void UART4_init(void);

/* initialize UART4 to transmit at 9600 Baud */
void UART4_init(void) {

//REG_MCLK_AHBMASK |=0x4;	APBC bus clock enabled by default
//REG_MCLK_APBCMASK |=0x10;	SERCOM1 APBC bus clock enabled by default

    REG_GCLK_PCHCTRL22 = 0x40;		//SERCOM4 core clock not enabled by default

//Configure SERCOM4

    REG_SERCOM4_USART_CTRLA |= 1;               /* reset SERCOM4 */
    while (REG_SERCOM4_USART_SYNCBUSY & 1) {}   /* wait for reset to complete */
    REG_SERCOM4_USART_CTRLA = 0x40106004;       /* LSB first, async, no parity,
        PAD[1]-Rx, PAD[0]-Tx, BAUD uses fraction, 8x oversampling, internal clock */
    REG_SERCOM4_USART_CTRLB = 0x00030000;       /* enable Tx, one stop bit, 8 bit */
    REG_SERCOM4_USART_BAUD  = 52;               /* 1000000/8/9600 = 13.02 */
    REG_SERCOM4_USART_CTRLA |= 2;               /* enable SERCOM4 */
    while (REG_SERCOM4_USART_SYNCBUSY & 2) {}   /* wait for enable to complete */
        
	//configure PINCFG and PMUX registers
    ARRAY_PORT_PINCFG1[8] |= 1;    /* allow pmux to set PB08 pin configuration */
	ARRAY_PORT_PINCFG1[9] |= 1;    /* allow pmux to set PB09 pin configuration */
    ARRAY_PORT_PMUX1[4] = 0x33;    /* PB08 = TxD , PB09=RxD*/
}


void UART4_write(char data) {
	while(!(REG_SERCOM4_USART_INTFLAG & 1)) {}  /* wait for data register empty */
	REG_SERCOM4_USART_DATA = data;              /* send a char */
}

char UART4_read(void) {
	while(!(REG_SERCOM4_USART_INTFLAG & 4)) {}  /* wait until receive complete */
	return REG_SERCOM4_USART_DATA;       /* read the receive char and return it */
}


