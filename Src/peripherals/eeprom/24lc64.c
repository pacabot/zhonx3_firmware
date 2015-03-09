/**************************************************************************/
/*!
    @file    24LC64.c
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Use the I2C bus with EEPROM 24LC64

    inspired by Author: hkhijhe
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/eeprom/24lc64.h"

/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/**************************************************************************/
/* Structure init                                                 		  */
/**************************************************************************/

void eepromWP(char state)
{
	if (state == ON)
		HAL_GPIO_WritePin(GPIOB, WRITE_PROTECT, SET);
	else
		HAL_GPIO_WritePin(GPIOB, WRITE_PROTECT, RESET);
}

void eepromWriteByte(unsigned int eeaddress, unsigned char data )
{
	unsigned char aTxBuffer[3];

	aTxBuffer[0] = (unsigned char)((eeaddress & 0xFF00)>> 8); //MSB
	aTxBuffer[1] = (unsigned char) (eeaddress & 0x00FF);	  //LSB
	aTxBuffer[2] = (unsigned char)(data);

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)aTxBuffer, 3);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
}

// WARNING: eeaddresspage is a page address, 6-bit end will wrap around
void eepromWritePage(unsigned int eeaddresspage, unsigned char* data, unsigned char length)
{
	unsigned char aTxBuffer[34] = {0};

	aTxBuffer[0] = (unsigned char)((eeaddresspage & 0xFF00)>> 8); //MSB
	aTxBuffer[1] = (unsigned char) (eeaddresspage & 0x00FF);	  //LSB

	for (int c = 0; c < length; c++)
		aTxBuffer[c+2] = data[c];

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)aTxBuffer, length+2);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
}

char eepromReadByte(unsigned int eeaddress)
{
	unsigned char aTxBuffer[2];
	unsigned char aRxBuffer;

	aTxBuffer[0] = (unsigned char)((eeaddress & 0xFF00)>> 8); //MSB
	aTxBuffer[1] = (unsigned char) (eeaddress & 0x00FF);	  //LSB

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)aTxBuffer, 2);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
	HAL_I2C_Master_Receive_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)&aRxBuffer, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}

	return aRxBuffer;
}

void eepromReadBuffer(unsigned int eeaddress, unsigned char *buffer, int length)
{
	unsigned char aTxBuffer[2];
	unsigned char aRxBuffer[34];

	aTxBuffer[0] = (unsigned char)((eeaddress & 0xFF00)>> 8); //MSB
	aTxBuffer[1] = (unsigned char) (eeaddress & 0x00FF);	  //LSB

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)aTxBuffer, 2);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
	HAL_I2C_Master_Receive_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)aRxBuffer, length);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}

	buffer = aRxBuffer;
}

void eepromTest(void)
{
	int addr = 0; //first address multiple of 32
	unsigned char b = 0;
	char somedata[32] = {0}; // data to write

	somedata[0] = 1;
	somedata[1] = 2;
	somedata[2] = 3;
	somedata[3] = 4;
	somedata[4] = 5;
	somedata[5] = 6;
	somedata[6] = 7;
	somedata[7] = 8;
	somedata[8] = 9;
	somedata[9] = 10;
	somedata[10] = 11;
	somedata[11] = 12;
	somedata[12] = 13;
	somedata[13] = 14;
	somedata[14] = 15;
	somedata[15] = 16;
	somedata[16] = 17;
	somedata[17] = 18;
	somedata[18] = 19;
	somedata[19] = 20;
	somedata[20] = 21;
	somedata[21] = 22;
	somedata[22] = 23;
	somedata[23] = 24;
	somedata[24] = 25;
	somedata[25] = 26;
	somedata[26] = 27;
	somedata[27] = 28;
	somedata[28] = 29;
	somedata[29] = 30;
	somedata[30] = 31;
	somedata[31] = 32;

	HAL_Delay(10); //add a small delay

//	Eeprom_Write_Page(addr, (unsigned char *)somedata, sizeof(somedata)); // write to EEPROM
	eepromWritePage(addr, (unsigned char *)somedata, 32); // write to EEPROM

	HAL_Delay(10); //add a small delay

	ssd1306ClearScreen();
	ssd1306DrawString(10, 10, "Memory written", &Font_5x8);
	ssd1306Refresh();

//	for (int i = 0; i < sizeof(somedata); i++) //increase address
	for (int i = 0; i < 32; i++) //increase address
	{
		b = eepromReadByte(i+addr); //access an address from the memory

		if (i < 8)
			ssd1306PrintInt((i*15), 20, " ",(char) b, &Font_5x8);
		else if (i < 16)
			ssd1306PrintInt(((i-8)*15), 30, " ",(char) b, &Font_5x8);
		else if (i < 24)
			ssd1306PrintInt(((i-16)*15), 40, " ",(char) b, &Font_5x8);
		else if (i < 32)
			ssd1306PrintInt(((i-24)*15), 50, " ",(char) b, &Font_5x8);
		ssd1306Refresh();
		HAL_Delay(100);
	}
//	ssd1306DrawString(10, 50, "End Cycle", &Font_5x8);
	ssd1306Refresh();
	HAL_Delay(2000);
}
