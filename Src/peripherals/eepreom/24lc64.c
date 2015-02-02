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
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/eeprom/24lc64.h"

/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/**************************************************************************/
/* Structure init                                                 		  */
/**************************************************************************/

void Eeprom_WP(char state)
{
	if (state == ON)
		HAL_GPIO_WritePin(GPIOB, WRITE_PROTECT, SET);
	else
		HAL_GPIO_WritePin(GPIOB, WRITE_PROTECT, RESET);
}

void Eeeprom_Write_Byte(unsigned int eeaddress, unsigned char data )
{
	unsigned char aTxBuffer[3];

	aTxBuffer[0] = (unsigned char)(eeaddress >> 8);		//MSB
	aTxBuffer[1] = (unsigned char)(eeaddress & 0xFF);	//LSB
	aTxBuffer[2] = (unsigned char)(data);

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)&aTxBuffer, 3);
}

void Eeprom_Write_Page(unsigned int eeaddresspage, unsigned char* data, unsigned char length)
{
	unsigned char aTxBuffer[100];

	aTxBuffer[0] = (unsigned char)(eeaddresspage >> 8);		//MSB
	aTxBuffer[1] = (unsigned char)(eeaddresspage & 0xFF);	//LSB

	for (int c = 0; c < length; c++)
		aTxBuffer[c+2] = data[c];

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)&aTxBuffer, length+2);
}

char Eeprom_Read_Byte(unsigned int eeaddress)
{
	unsigned char aTxBuffer[2];
	unsigned char aRxBuffer[1];

	aTxBuffer[0] = (unsigned char)(eeaddress >> 8);		//MSB
	aTxBuffer[1] = (unsigned char)(eeaddress & 0xFF);	//LSB

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)&aTxBuffer, 2);

	HAL_I2C_Master_Receive_DMA(&hi2c1, (unsigned char)0x50<<1, (uint8_t *)aRxBuffer, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}

	return aRxBuffer[0];
}

void Eeprom_Read_Buffer(unsigned int eeaddress, unsigned char *buffer, int length)
{
	unsigned char aTxBuffer[100];
	unsigned char aRxBuffer[1];

	aTxBuffer[0] = (unsigned char)(eeaddress >> 8);		//MSB
	aTxBuffer[1] = (unsigned char)(eeaddress & 0xFF);	//LSB

	HAL_I2C_Master_Transmit_DMA(&hi2c1, (unsigned char)0x50<<1, (unsigned char*)&aTxBuffer, 2);

	HAL_I2C_Master_Receive_DMA(&hi2c1, (unsigned char)0x50<<1, (uint8_t *)aRxBuffer, length);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}

	buffer = aRxBuffer;
}

void Debug_Eeprom(void)
{
	char somedata[8]; // data to write

	somedata[0] = 18;
	somedata[1] = 27;
	somedata[2] = 36;
	somedata[3] = 45;
	somedata[4] = 54;
	somedata[5] = 63;
	somedata[6] = 72;
	somedata[7] = 81;

	//TODO: Erreur sur les deux dernières mesures

	Eeprom_Write_Page(0, (unsigned char *)somedata, 8);// sizeof(somedata)); // write to EEPROM

	HAL_Delay(10); //add a small delay

	ssd1306ClearScreen();
	ssd1306DrawString(10, 10, "Memory written", &Font_5x8);
	ssd1306Refresh();

	while (1)
	{
		int addr = 0; //first address
		char b = Eeprom_Read_Byte(0); // access the first address from the memory

		while (b!=0)
		{
			ssd1306PrintInt(1+(addr*15), 20, " ",(char) b, &Font_5x8);

			addr++; //increase address
			b = Eeprom_Read_Byte(addr); //access an address from the memory
			ssd1306Refresh();
			HAL_Delay(200);
		}
		ssd1306DrawString(10, 30, "End Cycle", &Font_5x8);
		ssd1306Refresh();
		HAL_Delay(2000);
	}
}

