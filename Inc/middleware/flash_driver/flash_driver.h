/**************************************************************************/
/*!
    @file    flash_driver.h
    @author  Netanel (PACABOT)
    @date    15/10/2015
    @version 0.1
 */
/**************************************************************************/

#ifndef __FLASH_DRIVER_H__
#define __FLASH_DRIVER_H__

/* MANDATORY */
#define FLASH_DRIVER_SECTOR_SIZE	(16 * 1024)
#define FLASH_DRIVER_QUANTUM_SIZE   (1)

#define FLASH_DRIVER_VIRGIN_BYTE    (0xFF)
#define FLASH_DRIVER_FLASH_BASE     (ADDR_FLASH_SECTOR_8)
#define FLASH_DRIVER_FLASH_SIZE     (0x80000 /*512KB*/)

/* MANDATORY*/
#define FLASH_DRIVER_E_SUCCESS  	0

/* Error codes - optional */
#define FLASH_DRIVER_E_ERROR        1 // generic error
#define FLASH_DRIVER_E_UNDERFLOW    2
#define FLASH_DRIVER_E_OVERFLOW     3
#define FLASH_DRIVER_E_WRONG_PARAMS 4


/* MANDATORY API */
#ifdef __cplusplus
extern "C" {
#endif

void flash_driver_init(void);
void flash_driver_terminate(void);

int flash_driver_sector_erase(unsigned long addr);

int flash_driver_sector_read_buf(  unsigned long addr, unsigned long offset, unsigned char *p_dst, unsigned int length);
int flash_driver_sector_write_buf( unsigned long addr, unsigned long offset, unsigned char *p_src, unsigned int length);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_DRIVER_H__ */ 
