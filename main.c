//***************************************************************************
// Title			"ese 381 lab10task2"
// Date				05/08/2020
// Version			1.0
// Author			Ramez Kaupak && Miguel Rivas
// DESCRIPTION:		The program initializes the MCU for SPI communication with
//					the BME680 and the LCD, as well as RS232 communication to
//					the terminal. It incorporates overall learnings of ESE381 w. SAML21 MCU
// Target MCU:		saml21j18b
// Target Hardware	SPI interface with BME 680 and LCD DOG
// Restrictions		Must use 3.3V as Vin for BME680
// Revision History	Initial version
//
//**************************************************************************
/*
;
;*********************************************************************
;
;   *** Port Interface Definitions for BME680:
;
;  Ports              PA16  PA17  PA19   PB07
;  Port alt names	  MOSI  SCK	  MISO  /SS
;  BME680 pins		  SDI	SCK	  SDO	CSB
;  /SS = active low SPI select signal
;
;*********************************************************************
*/
/*
;
;*********************************************************************
;
;   *** Port Interface Definitions for LCD:
;
;  Ports              PA16  PA17  PA19   PA18
;  Port alt names	  MOSI  SCK	  MISO  /SS
;  /SS = active low SPI select signal
;
;*********************************************************************
*/

#include "saml21j18b.h"
#include <stdint.h>
#include "bme680.h"
#include "bme680_defs.h"
#include "rs232.h"
#include "lcd.h"
#include "sys_support.h"

unsigned char* ARRAY_PORT_PINCFG0 = (unsigned char*)&REG_PORT_PINCFG0;
unsigned char* ARRAY_PORT_PMUX0 = (unsigned char*)&REG_PORT_PMUX0;
unsigned char* ARRAY_PORT_PINCFG1 = (unsigned char*)&REG_PORT_PINCFG1;
unsigned char* ARRAY_PORT_PMUX1 = (unsigned char*)&REG_PORT_PMUX1;

//************ Function prototypes ************
void user_delay_ms(uint32_t period); //wait for a *period* amount of milliseconds
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);//returns contents of register address
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);//writes in register address

void init_spi_bme680 (void);//initialize SERCOM for SPI with BME680





int main (void) {
	
	//configuring sw0 button
	REG_PORT_DIRCLR0=0x4;		//make PA02 input for SW0
	ARRAY_PORT_PINCFG0[2]|=6;	//Enable PA02 input buffer with pull up
	REG_PORT_OUTSET0=0x4;		//make PA10 pull-up
	int state_status=0;//initial sfor pushbutton
	
	
	UART4_init();
	

	while(1)
	{
	init_spi_bme680();
	
	
/******* INITIALIZING SENSOR: Device structure for BME680 *******/		 
	struct bme680_dev gas_sensor;	
 gas_sensor.dev_id = 0;
 gas_sensor.intf = BME680_SPI_INTF;
 gas_sensor.read = user_spi_read;
 gas_sensor.write = user_spi_write;
 gas_sensor.delay_ms = user_delay_ms;
 /* amb_temp can be set to 25 prior to configuring the gas sensor
 * or by performing a few temperature readings without operating the gas sensor.
 */
 gas_sensor.amb_temp = 25;
 int8_t rslt = BME680_OK;
 rslt = bme680_init(&gas_sensor); /************ CONFIGURING SENSOR SETTINGS ************/ uint8_t set_required_settings;
 /* Set the temperature, pressure and humidity settings */
 gas_sensor.tph_sett.os_hum = BME680_OS_2X;
 gas_sensor.tph_sett.os_pres = BME680_OS_4X;
 gas_sensor.tph_sett.os_temp = BME680_OS_8X;
 gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
 /* Set the remaining gas sensor settings and link the heating profile */
 gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
 /* Create a ramp heat waveform in 3 steps */
 gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
 gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */
 /* Select the power mode */
 /* Must be set before writing the sensor configuration */
 gas_sensor.power_mode = BME680_FORCED_MODE;
 /* Set the required sensor settings needed */
 set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL |BME680_FILTER_SEL| BME680_GAS_SENSOR_SEL;
 /* Set the desired sensor configuration */
 rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
 /* Set the power mode */
 rslt = bme680_set_sensor_mode(&gas_sensor);
/* Get the total measurement duration so as to sleep or wait till the
 * measurement is complete */
/************ READING ALL SENSOR DATA ************/
 uint16_t meas_period;
 bme680_get_profile_dur(&meas_period, &gas_sensor);
 struct bme680_field_data data;
		 
		 
		 
		 user_delay_ms(meas_period); /* Delay till the measurement is ready */
		 rslt = bme680_get_sensor_data(&data, &gas_sensor);
		 printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,data.pressure / 100.0f, data.humidity / 1000.0f );
		 /* Avoid using measurements from an unstable heating setup */
		 if(data.status & BME680_GASM_VALID_MSK)
		 printf(", G: %ld ohms", data.gas_resistance);
		 printf("\r\n");
		 /* Trigger the next measurement if you would like to read data out
		 continuously */
		
		//pushbutton select measurements to display
		if (!(REG_PORT_IN0 & 0x00000004)&& !state_status) //check switch status, check if pressed (1)
		{state_status=1;}
		
		else if (!(REG_PORT_IN0 & 0x00000004)&&state_status)
		{state_status=0;}
	
			//after the state is determined, output is selected
			if (state_status==0)
			{
			//display temperature and humidity and pressure
			init_lcd_dog();
			//LCD DOG DISPLAY
			sprintf(rawBuff, "T: %.2f degC\n \rP: %.2f hPa\n \rH: %.2f %%rH ", data.temperature / 100.0f,data.pressure / 100.0f, data.humidity / 1000.0f);
			lcd_escape(rawBuff, displayBuff, dsp_buff_1, dsp_buff_2, dsp_buff_3);
			update_lcd_dog();
	
			for (int i=0; i<35;i++)
			delay_40mS();
			}
				else
					{
					
					init_lcd_dog();
					clr_dsp_buffs ();
					//LCD DOG DISPLAY
					sprintf(rawBuff, "G: %ld ohms ", data.gas_resistance);
					lcd_escape(rawBuff, displayBuff, dsp_buff_1, dsp_buff_2, dsp_buff_3);
					update_lcd_dog();
	
					for (int i=0; i<35;i++)
					delay_40mS();
					}	
		
		 if (gas_sensor.power_mode == BME680_FORCED_MODE) 
		 {
			 rslt = bme680_set_sensor_mode(&gas_sensor);
		 }
	} }	
	
	
	
	

/*
;********************************
;NAME:			spi_transfer
;ASSUMES:		8 bit uint as input
;RETURNS:		8 bit uint in SPI data register
;CALLED BY:		main, spi_read_bme680, spi_write_bme680
;DESCRIPTION:	transfer function, sends a byte and checks flags for
;				completion of task (Tx / Rx)
;NOTE: Can be used as is with MCU clock speeds of 4MHz or less.
;*****************************
*/
uint8_t spi_transfer(uint8_t data) {
	while(!(REG_SERCOM1_SPI_INTFLAG & 1)){}		//wait until Tx ready
	REG_SERCOM1_SPI_DATA = data;
	if (data & 0x80) {
		while(!(REG_SERCOM1_SPI_INTFLAG & 4)){}	//if reading (R/W=1), wait until Rx complete (RxC flag)
	}
	while(!(REG_SERCOM1_SPI_INTFLAG & 2)){}		//wait until Tx complete (TxC flag)
	return REG_SERCOM1_SPI_DATA;
}

/*
;********************************
;NAME:			user_spi_write
;ASSUMES:		8 bit register address and 8 bit data as pointer to input, device ID
;				and length of bytes
;RETURNS:		result=0 if successful
;CALLED BY:		main, init_spi_bme680
;DESCRIPTION:	write function, sends a byte of address and uses
;				spi_transfer to access the contents to overwrite
;NOTE: Can be used as is with MCU clock speeds of 4MHz or less.
;*****************************
*/
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
	REG_PORT_OUTCLR1 |= 0x80;		//Bring CS low
	spi_transfer(reg_addr);			//define address to write
	for (int i=0;i<len;i++,reg_data++){
	//for (len;len>0;len--,++reg_data){
	spi_transfer(*reg_data);	//write in reg_addr
	}
	REG_PORT_OUTSET1 |= 0x80;		//Bring CS high
	int8_t rslt = 0; /* Return 0 for Success, non­zero for failure */
	return rslt;
}


/*
;********************************
;NAME:			user_spi_read
;ASSUMES:		8 bit register address as input, device ID, register data, length
;RETURNS:		result=0 if successful, modifies content of pointer
;CALLED BY:		main
;DESCRIPTION:	read function, sends a byte of address and uses
;				spi_transfer to read the contents and store in variable
;NOTE: Can be used as is with MCU clock speeds of 4MHz or less.
;*****************************
*/
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	REG_PORT_OUTCLR1 |= 0x80;				//Bring CS low
	spi_transfer(reg_addr | 0x80);			//making it read byte
	for (int i=0;i<len;i++,reg_data++){
	*reg_data= spi_transfer(0x00);
	}	//dummy byte for reading
	REG_PORT_OUTSET1 |= 0x80;				//Bring CS high
	int8_t rslt = 0; /* Return 0 for Success, non­zero for failure */
	return rslt;
	}


/*
;********************************
;NAME:			user_delay_ms
;ASSUMES:		period of ms as parameter
;RETURNS:		nothing
;CALLED BY:		main
;DESCRIPTION:	User delay function, defines amount of ms in 32 bit int parameter
;NOTE: Can be used as is with MCU clock speeds of 4MHz or less.
;********************************
*/
void user_delay_ms(uint32_t period) {
	for (int j=0;j<period;j++){
		for (int i = 0; i < 307; i++)				//~1ms
		__asm("nop");
	}
}



/*
;********************************
;NAME:			init_spi_bme680
;ASSUMES:		nothing
;RETURNS:		nothing
;CALLED BY:		main
;DESCRIPTION:	initialization function, sets up saml21j18b for SPI in SERCOM1
;				also sets up pin for /CS in the BME680. Finally, switch to page 0
;				in the status register of the BME680
;NOTE: Can be used as is with MCU clock speeds of 4MHz or less.
;*****************************
*/
void init_spi_bme680(void){
	REG_GCLK_PCHCTRL19 = 0x00000040;	/* SERCOM1 core clock not enabled by default */
	ARRAY_PORT_PINCFG0[16] |= 1;		/* allow pmux to set PA16 pin configuration */
	ARRAY_PORT_PINCFG0[17] |= 1;		/* allow pmux to set PA17 pin configuration */
	ARRAY_PORT_PINCFG0[18] |= 1;		/* allow pmux to set PA18 pin configuration !!SS for LCD!!*/
	ARRAY_PORT_PINCFG0[19] |= 1;		/* allow pmux to set PA19 pin configuration */
	
	ARRAY_PORT_PMUX0[8] |= 0x22;     /* PA16 = MOSI, PA17 = SCK */
	ARRAY_PORT_PMUX0[9] |= 0x20;     /* PA18 = /SS PA19 = MISO */
	
	REG_PORT_DIRSET0 |= 0x40000;	/*set PA18 as /SS for LCD*/
	REG_PORT_OUTSET1 |= 0x40000;	/*PA18 hi initially*/
	REG_PORT_DIRSET1 |= 0x80;		/* Set PB07 as /SS for BME */
	REG_PORT_OUTSET1 |= 0x80;		/* PB07 hi initially */
	
	REG_SERCOM1_SPI_CTRLA = 1;              /* reset SERCOM1 */
	while (REG_SERCOM1_SPI_CTRLA & 1) {}    /* wait for reset to complete */
	REG_SERCOM1_SPI_CTRLA = 0x3030000C;     /* MISO-pad[3], MOSI-pad[0], SCK-pad[1], SS-pad[2], */
	/*SW controlled SS so ctrlB.MSSEN=0 so LCD can be used in next lab */
	REG_SERCOM1_SPI_CTRLB = 0x00020000;		/* RXEN=1, MSSEN=0 (SW controlled), 8-bit */
	REG_SERCOM1_SPI_BAUD = 1;               /* SPI clock is = 1MHz */
	REG_SERCOM1_SPI_CTRLA |= 2;             /* enable SERCOM1 */
	while(!(REG_SERCOM1_SPI_INTFLAG & 1)){} //wait until Tx ready

}