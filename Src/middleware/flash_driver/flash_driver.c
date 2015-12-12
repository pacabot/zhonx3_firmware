/**************************************************************************/
/*!
 @file    flash_driver.c
 @author  Netanel (PACABOT)
 @date    15/10/2015
 @version 0.1
 */
/**************************************************************************/

#include <string.h>

/* Common declarations */
#include "config/config.h"
#include "config/basetypes.h"
#include "config/errors.h"

#include "peripherals/flash/flash.h"
#include "middleware/flash_driver/flash_driver.h"

/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

static uint32_t GetSector(uint32_t Address);


void flash_driver_init(void)
{
}

void flash_driver_terminate(void)
{
    // TODO: Implement this function if needed
    return;
}

int flash_driver_sector_write_buf(unsigned long addr, unsigned long offset,
                                  unsigned char *p_src, unsigned int length)
{
    int rv = FLASH_DRIVER_E_SUCCESS;

    // Add offset to base address
    addr += offset;
    unsigned long data = 0;
    unsigned int words;
    unsigned int remain_bytes;

    // Compute the number of words to write
    words = length / 4;
    // Compute the number of remaining byes
    remain_bytes = length % 4;

    /* Unlock the Flash to enable the Flash control register access */
    HAL_FLASH_Unlock();

    // Loop to write into the Flash
    if (words > 0)
    {
        while (length > 0)
        {
            data = (p_src[3] << 24) |
                   (p_src[2] << 16) |
                   (p_src[1] << 8)  |
                   (p_src[0] << 0);
            rv = HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, data);
            if (rv != HAL_OK)
            {
                rv = FLASH_DRIVER_E_ERROR;
            }
            else
            {
                rv = FLASH_DRIVER_E_SUCCESS;
            }
            length -= 4;
            addr += 4;
            p_src += 4;
        }
    }

    while (remain_bytes > 0)
    {
        rv = HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr, *p_src++);
        remain_bytes--;
        addr++;
    }

    HAL_FLASH_Lock();

    return rv;
}

int flash_driver_sector_read_buf(unsigned long addr, unsigned long offset,
                                 unsigned char *p_dst, unsigned int length)
{
    int rv = FLASH_DRIVER_E_SUCCESS;

    memcpy(p_dst, (unsigned char *) (addr + offset), length);

    return rv;
}

int flash_driver_sector_erase(unsigned long addr)
{
    int rv = FLASH_DRIVER_E_SUCCESS;
    /*Variable used for Erase procedure*/
    FLASH_EraseInitTypeDef EraseInitStruct;
    unsigned long SectorError = 0;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = GetSector(addr);
    EraseInitStruct.NbSectors = 1;

    /* Unlock the Flash to enable the Flash control register access */
    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        /*
         Error occurred while sector erase.
         You can add here some code to deal with this error.
         SectorError will contain the faulty sector and then to know the code error on this sector,
         user can call function 'HAL_FLASH_GetError()'
         */
        /*
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
         */
        rv = FLASH_DRIVER_E_ERROR;
    }
    else
    {
        rv = FLASH_DRIVER_E_SUCCESS;
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) */
    HAL_FLASH_Lock();

    return rv;
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}
