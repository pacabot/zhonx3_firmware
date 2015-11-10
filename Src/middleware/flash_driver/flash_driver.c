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

void flash_driver_init(void)
{
    FLASH_DRIVER_DESC flash_driver_desc;

    /* Internal flash */
    flash_driver_desc.props.FLASH_DRIVER_FLASH_BASE = FLASH_DRIVER_FLASH_BASE;
    flash_driver_desc.props.FLASH_DRIVER_FLASH_SIZE = FLASH_DRIVER_FLASH_SIZE;
    flash_driver_desc.props.FLASH_DRIVER_QUANTUM_SIZE = FLASH_DRIVER_QUANTUM_SIZE;
    flash_driver_desc.props.FLASH_DRIVER_SECTOR_SIZE = FLASH_DRIVER_SECTOR_SIZE;
    flash_driver_desc.props.FLASH_DRIVER_VIRGIN_BYTE = FLASH_DRIVER_VIRGIN_BYTE;
    flash_driver_desc.ops.flash_driver_init = flash_driver_init;
    flash_driver_desc.ops.flash_driver_terminate = flash_driver_terminate;
    flash_driver_desc.ops.flash_driver_sector_erase = flash_driver_sector_erase;
    flash_driver_desc.ops.flash_driver_sector_check_blank = NULL; // XXX TODO not implemented yet, not used yet
    flash_driver_desc.ops.flash_driver_sector_read_buf = flash_driver_sector_read_buf;
    flash_driver_desc.ops.flash_driver_sector_write_buf = flash_driver_sector_write_buf;
    return;
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

    /* Unlock the Flash to enable the Flash control register access */
    HAL_FLASH_Unlock();

    // Loop to write into the Flash
    while (length > 0)
    {
        rv = HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, (unsigned long)p_src);
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
    }
    HAL_FLASH_Lock();

    return rv;
}

int flash_driver_sector_read_buf(unsigned long addr, unsigned long offset,
                                 unsigned char *p_dst, unsigned int length)
{
    int rv = FLASH_DRIVER_E_SUCCESS;

    memcpy(p_dst, (unsigned int *) (addr + offset), length);

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
    EraseInitStruct.Sector = addr;
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
