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

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

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
