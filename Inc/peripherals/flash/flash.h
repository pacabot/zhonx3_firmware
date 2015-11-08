/**************************************************************************/
/*!
 @file    flash.h
 @author  Netanel (PACABOT)
 @date    08/10/2015
 @version 0.1
 */
/**************************************************************************/

#ifndef _FLASH_H_
#define _FLASH_H_

#ifndef uintptr_t
typedef unsigned int uintptr_t;
#endif

/**
 *
 * @defgroup FLASH_Constant File Constants
 *
 * @ingroup FLASH
 * @ingroup Constants_Codes
 *
 * This chapter contains descriptions of all constants used by the \link
 * FLASH File \endlink of the Hardware
 * Abstraction Layer.
 *
 * @{
 *
 */

/** Module ID for the File interface */
#define FLASH_MODULE_ID    0x10

/**
 *
 * @defgroup FLASH_Error_Codes File Error Codes
 *
 * @ingroup FLASH
 * @ingroup Error_Codes
 *
 * This chapter contains descriptions of all error codes used by the \link
 * FLASH File Interface \endlink of the Hardware
 * Abstraction Layer.
 *
 * @{
 *
 */

/** \showinitializer No error */
#define FLASH_E_SUCCESS  0
/** \showinitializer Generic error */
#define FLASH_E_ERROR                   MAKE_ERROR(FLASH_MODULE_ID,0x01)
#define FLASH_E_UNDERFLOW               MAKE_ERROR(FLASH_MODULE_ID,0x02)
#define FLASH_E_OVERFLOW                MAKE_ERROR(FLASH_MODULE_ID,0x03)
#define FLASH_E_ADDRESS_NOT_ALIGNED     MAKE_ERROR(FLASH_MODULE_ID,0x04)
#define FLASH_E_WRONG_PARAMS            MAKE_ERROR(FLASH_MODULE_ID,0x05)
#define FLASH_E_DRIVER_ERROR            MAKE_ERROR(FLASH_MODULE_ID,0x06)

/** @}*/

/** @brief Opaque data type definition for the Flash interface */
typedef void* FLASH_HANDLE;

/** @brief Describe an underlying flash driver.
 * 
 * The idea is to describe an underlying Flash driver. The Flash module can indeed
 * use different flash chips at once.
 */
#define FLASH_DRIVER_E_SUCCESS  0
typedef struct
{
    struct ops
    {
        /** init the flash driver */
        void (*flash_driver_init)(void);
        /** terminate the flash driver */
        void (*flash_driver_terminate)(void);

        /** erase a sector in flash */
        int (*flash_driver_sector_erase)(unsigned long addr);

        /** read data in flash, into p_dst, which must be in RAM */
        int (*flash_driver_sector_read_buf)(unsigned long addr,
                unsigned long offset, unsigned char *p_dst, unsigned int length);

        /** check the provided area is erased.
         * WARNING: should check if 1s in flash are of "good" quality, to reduce the risk of data loss.
         * (in case the 1s are close to 0s, erasing again the area will refresh them)
         * If not, should consider the area as not erased
         */
        int (*flash_driver_sector_check_blank)(unsigned long addr,
                unsigned long offset, unsigned int length);
        /** write data in flash, from p_src, which must be in RAM */
        int (*flash_driver_sector_write_buf)(unsigned long addr,
                unsigned long offset, unsigned char *p_src, unsigned int length);
    } ops;
    struct props
    {
        unsigned int FLASH_DRIVER_SECTOR_SIZE;
        unsigned int FLASH_DRIVER_QUANTUM_SIZE;
        uint8_t FLASH_DRIVER_VIRGIN_BYTE; // should be 0xFF
        uintptr_t FLASH_DRIVER_FLASH_BASE;
        unsigned int FLASH_DRIVER_FLASH_SIZE;
    } props;
} FLASH_DRIVER_DESC;

#ifdef __cplusplus
extern "C"
{
#endif
/**
 *
 * @defgroup FLASH File
 * This chapter contains descriptions of the functions
 * accessing the underlying File System.
 *
 * @addtogroup FLASH
 *
 * @{
 */

/**
 * @brief Initializes the Flash interface.
 *
 * This function initializes the Flash interface.
 * This function is called once when the system starts up, before
 * any other call to the for this interface. It must perform all actions
 * required to prepare the service for use. This includes the allocation
 * and/or initialization of all structures and variables, and the loading of
 * any required drivers.
 *
 * @return #FLASH_E_SUCCESS if the operation is successful,
 * @return #FLASH_E_ERROR otherwise.
 */
int flash_init(void);

/**
 * @brief Shutdowns the Flash interface.
 *
 * This function is called once when the system shuts down.
 * It must perform all operations required to cleanup after interface use.
 * This includes freeing of any allocated memory, the reinitialization of
 * any static memory (if needed), and the unloading of any drivers.
 *
 * @return #FLASH_E_SUCCESS if the operation is successful,
 * @return #FLASH_E_ERROR otherwise.
 */
int flash_terminate(void);

/**
 * @brief Open a Flash interface.
 *
 * The function opens a Flash interface.
 *
 * @param[in]   flash_driver_desc    describes the flash driver to which Flash must connect
 * @param[out]  handle               valid pointer toward a FLASH_HANDLE handle on the provided flash device
 *
 * @return #FLASH_E_SUCCESS if the operation is successful,
 * @return #FLASH_E_WRONG_PARAMS if one argument is invalid
 * @return #FLASH_E_ERROR otherwise.
 */
int flash_open(FLASH_DRIVER_DESC *flash_driver_desc, FLASH_HANDLE *handle);

/**
 * @brief Closes a Flash interface.
 *
 * This function closes a specific Flash interface
 * and releases its resources.
 *
 * @param[in]   handle   handle of the Flash interface to close.
 *
 * @return #FLASH_E_SUCCESS if the operation is successful,
 * @return #FLASH_E_WRONG_PARAMS if the argument is invalid
 * @return #FLASH_E_ERROR otherwise.
 */
int flash_close(FLASH_HANDLE handle);

/**
 * @brief Erase flash at the given address, on the given length
 * 
 * @param[in]   handle  handle of the Flash interface
 * @param[in]   p_dest  pointer to a valid address in Flash
 * @param[in]   length  number of bytes to erase, see WARNING above
 * 
 * @return status
 */
int flash_erase(FLASH_HANDLE handle, unsigned char *p_dest, unsigned int length);

/**
 * @brief Write data in RAM to Flash
 * 
 * @param[in]   handle  handle of the Flash interface
 * @param[in]   p_dest  pointer to a valid address in Flash, destination
 * @param[in]   p_src   pointer to a valid address in RAM, source
 * @param[in]   length  number of bytes to copy from p_src to p_dest
 * 
 * @return status
 */
int flash_write(FLASH_HANDLE handle, unsigned char *p_dest,
        unsigned char *p_src, unsigned int length);

/**
 * @brief Flash to RAM copy
 * 
 * @param[in]   handle  handle of the Flash interface
 * @param[in]   p_dest  pointer to a valid address in RAM, destination
 * @param[in]   p_src   pointer to a valid address in Flash, source
 * @param[in]   length  number of bytes to copy from p_src to p_dest
 * 
 * @return status
 */
int flash_read(FLASH_HANDLE handle, unsigned char *p_dest, unsigned char *p_src,
        unsigned int length);

/**
 * @brief Return the flash driver descriptor associated to the provided handle
 * 
 * @param[in]   handle  handle of the Flash interface
 * @param[out]  p_desc	pointer to the descriptor, populated if the function succeeds
 * 
 * @return status
 */
int flash_get_desc(FLASH_HANDLE handle, FLASH_DRIVER_DESC *p_desc);

/* TODO: flash_flash2flash_copy() */

/** @} */
#ifdef __cplusplus
}
#endif

#endif /*  _FLASH_H_ */

