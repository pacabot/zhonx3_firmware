/**************************************************************************/
/*!
    @file    flash.c
    @author  Netanel (PACABOT)
    @date    08/10/2015
    @version 0.1
 */
/**************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Common declarations */
#include "config/config.h"
#include "config/basetypes.h"
#include "config/module_id.h"
#include "config/errors.h"

/* This module */
#include "peripherals/flash/flash.h"

/* pointer utilities */
#include "middleware/misc_utils/misc_utils.h"
#include "middleware/flash_driver/flash_driver.h"

#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "stm32f4xx.h"

#ifdef CONFIG_FLASH_HAVE_TRACE
# define TRACE(_msg_)               DEBUG_TRACE(_msg_)
# define TRACE1(_msg_,_a1_)         DEBUG_TRACE1(_msg_,_a1_)
# define TRACE2(_msg_,_a1_,_a2_)    DEBUG_TRACE2(_msg_,_a1_,_a2_)
# define TRACE3(_msg_,_a1_,_a2_,_a3_)    DEBUG_TRACE3(_msg_,_a1_,_a2_,_a3_)
# define DUMP(_msg_,_a1_,_a2_)      DEBUG_DUMP(_msg_,_a1_,_a2_)
# define ASSERT(_c_,_msg_)          DEBUG_ASSERT(_c_,_msg_)
#else
# define TRACE(_msg_)
# define TRACE1(_msg_,_a1_)
# define TRACE2(_msg_,_a1_,_a2_)
# define TRACE3(_msg_,_a1_,_a2_,_a3_)
# define DUMP(_msg_,_a1_,_a2_)
# define ASSERT(_c_,_msg_)
#endif


/* Internal types */
/** @brief The actual Flash handle type, internal to this module, must not be exposed
 * 
 * The idea is to describe an underlying Flash driver. The Flash module can indeed
 * use different flash chips at once.
 * 
 */
typedef struct
{
    int              	is_used;
    FLASH_DRIVER_DESC	drv_desc;
} FLASH_INTERNAL_HANDLE;

/*
 * WARNING
 * XXX
 * Because of this static buffer, Flash is NOT reentrant, and CONFIG_FLASH_SECTOR_BUFFER_SIZE is required, which is lame.
 * If reentrancy is required, 2 options are possible:
 * - use 'buffer = alloca(ihandle->drv_desc.props.sector_size)' in flash_read and flash_write
 * - make the caller of read/write functions provide a large enough buffer (XXX trusted by Flash then)
 */
/* Private members */
#ifndef CONFIG_FLASH_SECTOR_BUFFER_SIZE
#   error Please define mandatory symbol CONFIG_FLASH_SECTOR_BUFFER_SIZE. Value is in bytes. Good value is the max of all FLASH_DRIVER_SECTOR_SIZE.
#endif
static unsigned char flash_sector_buffer[CONFIG_FLASH_SECTOR_BUFFER_SIZE];

#ifndef CONFIG_FLASH_NB_FLASH_DEVICES
#   error Please define mandatory symbol CONFIG_FLASH_NB_FLASH_DEVICES. Indicate the number of flash devices that Flash can handle.
#endif
static FLASH_INTERNAL_HANDLE flash_internal_handles[CONFIG_FLASH_NB_FLASH_DEVICES];


uintptr_t ptr_align(uintptr_t ptr, size_t alignment)
{
    uintptr_t p = ptr;

    p += -p & (alignment - 1);
    return p;
}

int is_ptr_aligned(uintptr_t ptr, size_t alignment)
{
    return (ptr & (alignment - 1)) ? FALSE : TRUE;
}

uintptr_t get_align_mask(size_t alignment)
{
    return ~((uintptr_t)alignment - 1);
}


/* local helpers */
static int check_addr_range(FLASH_INTERNAL_HANDLE *ihandle, unsigned int p, unsigned int len);
// factorize code for flash_erase and flash_write. flash_read differs
static int update_flash_with_buffer(FLASH_INTERNAL_HANDLE *ihandle, unsigned int p, unsigned int length, unsigned char *buf);



/* API */
/*
 * TODO
 * XXX
 * I did not have time to implement a decent Flash abstraction layer that would use ihandle->drv_desc.ops.flash_driver_sector_check_blank()
 * The current implementation erases the flash before ANY write, which over-stresses the device.
 * This _SHOULD_ be corrected.
 */
static FLASH_DRIVER_DESC _flash_driver_desc;

int flash_init(void)
{
    int rv = FLASH_E_SUCCESS;


    memset(flash_internal_handles, 0 , sizeof(flash_internal_handles));

    /* Internal flash */
    _flash_driver_desc.props.flash_base = FLASH_DRIVER_FLASH_BASE;
    _flash_driver_desc.props.flash_size= FLASH_DRIVER_FLASH_SIZE;
    _flash_driver_desc.props.quantum_size = FLASH_DRIVER_QUANTUM_SIZE;
    _flash_driver_desc.props.sector_size = FLASH_DRIVER_SECTOR_SIZE;
    _flash_driver_desc.props.virgin_byte = FLASH_DRIVER_VIRGIN_BYTE;
    _flash_driver_desc.ops.flash_driver_init = flash_driver_init;
    _flash_driver_desc.ops.flash_driver_terminate = flash_driver_terminate;
    _flash_driver_desc.ops.flash_driver_sector_erase = flash_driver_sector_erase;
    _flash_driver_desc.ops.flash_driver_sector_check_blank = NULL; // XXX TODO not implemented yet, not used yet
    _flash_driver_desc.ops.flash_driver_sector_read_buf = flash_driver_sector_read_buf;
    _flash_driver_desc.ops.flash_driver_sector_write_buf = flash_driver_sector_write_buf;

    return rv;
}


int flash_terminate(void)
{
    int rv = FLASH_E_SUCCESS;
    unsigned int i;
    
    /* clear any handle that was not closed */
    for (i = 0; i < CONFIG_FLASH_NB_FLASH_DEVICES; ++i)
    {
        if (flash_internal_handles[i].is_used)
        {
            flash_internal_handles[i].drv_desc.ops.flash_driver_terminate();
        }
        flash_internal_handles[i].is_used = FALSE;
    }

    return rv;
}


int flash_open(FLASH_DRIVER_DESC *flash_driver_desc, FLASH_HANDLE *handle)
{
    int rv = FLASH_E_SUCCESS;
    unsigned int i;
    
    if (handle == NULL)
    {
        TRACE("ERROR: provided pointer to handle is NULL");
        return FLASH_E_WRONG_PARAMS;
    }
//    if (flash_driver_desc == NULL)
//    {
//        TRACE("ERROR: provided pointer to flash driver descriptor is NULL");
//        return FLASH_E_WRONG_PARAMS;
//    }
    /* XXX elementary check, NOT STRONG ENOUGH FOR REAL LIFE APPLICATION, and props are not checked */
//    if (
//               flash_driver_desc->ops.flash_driver_init == NULL
//            || flash_driver_desc->ops.flash_driver_terminate == NULL
//            || flash_driver_desc->ops.flash_driver_sector_erase == NULL
//            /*|| flash_driver_desc->ops.flash_driver_sector_check_blank == NULL*/ //currently unused, see above
//            || flash_driver_desc->ops.flash_driver_sector_read_buf == NULL
//            || flash_driver_desc->ops.flash_driver_sector_write_buf == NULL
//            )
//    {
//        TRACE("ERROR: provided flash driver descriptor has some undefined operation(s)");
//        return FLASH_E_WRONG_PARAMS;
//    }
    
    for (i = 0; i < CONFIG_FLASH_NB_FLASH_DEVICES; ++i)
    {
        if (!flash_internal_handles[i].is_used)
            break;
    }
    if (i >= CONFIG_FLASH_NB_FLASH_DEVICES)
    {
        TRACE("ERROR: all handles are already in use");
        return FLASH_E_ERROR;
    }
    
    /* init the provided flash */
    _flash_driver_desc.ops.flash_driver_init();
    
    /* copy the provided driver descriptor */
    memcpy(&(flash_internal_handles[i].drv_desc), &_flash_driver_desc, sizeof(FLASH_DRIVER_DESC));
    flash_internal_handles[i].is_used = TRUE;
    
    /* return the handle */
    *handle = (FLASH_HANDLE)(&flash_internal_handles[i]);
    
    return rv;
}


int flash_close(FLASH_HANDLE handle)
{
    int rv = FLASH_E_SUCCESS;
    FLASH_INTERNAL_HANDLE *ihandle = (FLASH_INTERNAL_HANDLE*)handle;
    
    if (handle == NULL)
    {
        TRACE("ERROR: provided handle is NULL");
        return FLASH_E_WRONG_PARAMS;
    }
    /* XXX from that point the handle is trusted */
    
    /* terminate the provided flash and free the handle */
    ihandle->drv_desc.ops.flash_driver_terminate();
    ihandle->is_used = FALSE;
    
    return rv;    
}


int flash_erase(FLASH_HANDLE handle,
                    unsigned char *p_dest,
                    unsigned int   length)
{
    int rv;
    FLASH_INTERNAL_HANDLE *ihandle = (FLASH_INTERNAL_HANDLE*)handle;
    
    if (handle == NULL)
    {
        TRACE("ERROR: provided handle is NULL");
        return FLASH_E_WRONG_PARAMS;
    }
    /* XXX from that point the handle is trusted */

    /* Nothing to do */
    if (length == 0)
    {
        TRACE("WARNING: 0 length provided (nothing done)");
        return FLASH_E_SUCCESS;
    }

    rv = update_flash_with_buffer(ihandle, (unsigned int)p_dest, length, NULL/*no buffer, just erase*/);

    if (rv != FLASH_E_SUCCESS)
    {
        TRACE2("ERROR: could not erase flash at 0x%0X on %u bytes", p_dest, length);
    }
    else
    {
        TRACE2("successfully erased flash at 0x%0X on %u bytes", p_dest, length);
    }

    return rv;
}


int flash_write(FLASH_HANDLE handle,
                unsigned char *p_dest,
                unsigned char *p_src,
                unsigned int   length)
{
    int rv;
    FLASH_INTERNAL_HANDLE *ihandle = (FLASH_INTERNAL_HANDLE*)handle;
    
    if (handle == NULL)
    {
        TRACE("ERROR: provided handle is NULL");
        return FLASH_E_WRONG_PARAMS;
    }
    /* XXX from that point the handle is trusted */

    /* Nothing to do */
    if (length == 0)
    {
        TRACE("WARNING: 0 length provided (nothing done)");
        return FLASH_E_SUCCESS;
    }
    /* Check p_src is NOT in Flash */
    if ( ((unsigned int)p_src >= ihandle->drv_desc.props.flash_base)
            && ((unsigned int)p_src < (ihandle->drv_desc.props.flash_base + ihandle->drv_desc.props.flash_size)) )
    {
        TRACE("ERROR: src pointer is in Flash");
        return FLASH_E_WRONG_PARAMS;
    }

    rv = update_flash_with_buffer(ihandle, (unsigned int)p_dest, length, p_src);

    if (rv != FLASH_E_SUCCESS)
    {
        TRACE3("ERROR: could not write flash at 0x%0X on %u bytes with data from 0x%0X", p_dest, length, p_src);
    }
    else
    {
        TRACE3("successfully written flash at 0x%0X on %u bytes with data from 0x%0X", p_dest, length, p_src);
    }

    return rv;
}


int flash_read(FLASH_HANDLE handle,
               unsigned char *p_dest,
               unsigned char *p_src,
               unsigned int   length)
{
    int rv = FLASH_E_SUCCESS;
    unsigned int size = 0;
    unsigned int base_sector_addr = 0, end_sector_addr = 0, num_sectors = 0, i;
    unsigned int base_offset = 0, base_offset_algn = 0;
    unsigned int end_offset  = 0, end_bytes_to_read_algn  = 0;
    unsigned int p = (unsigned int)p_src;
    unsigned int aligned_p;
    unsigned int sector_mask;
    FLASH_INTERNAL_HANDLE *ihandle = (FLASH_INTERNAL_HANDLE*)handle;
    
    if (handle == NULL)
    {
        TRACE("ERROR: provided handle is NULL");
        return FLASH_E_WRONG_PARAMS;
    }
    /* XXX from that point the handle is trusted */

    sector_mask = get_align_mask(ihandle->drv_desc.props.sector_size);

    TRACE3("attempt to read flash at 0x%0X on %u bytes into 0x%0X", p_src, length, p_dest);

    /* Nothing to do */
    if (length == 0)
    {
        TRACE("WARNING: 0 length provided (nothing done)");
        return FLASH_E_SUCCESS;
    }
    /* Check p_dest is NOT in Flash */
    if ( ((unsigned int)p_dest >= ihandle->drv_desc.props.flash_base)
            && ((unsigned int)p_dest < (ihandle->drv_desc.props.flash_base + ihandle->drv_desc.props.flash_size)) )
    {
        TRACE("ERROR: dest pointer is in Flash");
        return FLASH_E_WRONG_PARAMS;
    }
    /* check input parameters */
    rv = check_addr_range(ihandle, p, length);
    if (rv != FLASH_E_SUCCESS)
    {
        TRACE("ERROR: Invalid parameters");
        return rv;
    }

    /* compute address of 1st and last sector to read */
    base_sector_addr = p & sector_mask;
    end_sector_addr  = (p + length - 1) & sector_mask;

    // the number of sectors to be read
    num_sectors = (end_sector_addr - base_sector_addr)/ihandle->drv_desc.props.sector_size + 1;

    /*
     * NOTE:
     * Flash operates with addresses and offsets aligned on its data quantum.
     * 'xxx_offset' might not be aligned.
     * For things to work properly, align addresses and erase the little extra in the RAM buffer
     */
    aligned_p        = ptr_align(p, ihandle->drv_desc.props.quantum_size);
    // starting offset
    base_offset      = p & ~(sector_mask);
    // quantum aligned, backwards
    base_offset_algn = (aligned_p == p) ? base_offset : ((aligned_p - ihandle->drv_desc.props.quantum_size) & ~(sector_mask));
    // ending offset
    end_offset       = (p + length - 1) & ~(sector_mask);
    // the number of bytes to read in the last sector is thus (end_offset+1)
    // the following is the number of bytes to actually read to be compatible with Flash access requirement
    end_bytes_to_read_algn = ptr_align((unsigned int)(end_offset+1), ihandle->drv_desc.props.quantum_size) % ihandle->drv_desc.props.sector_size; // not a pointer, but same operation

    // NOTE: the 'length' variable is now only used as p_dest's length

    /* deal with 1st and last sector, then copy plain sectors if any */
    if (base_offset)
    {
        /* initialize working buffer with the Flash blank pattern */
        memset(flash_sector_buffer, ihandle->drv_desc.props.virgin_byte, sizeof(flash_sector_buffer));

        /* read last chunk of sector data */
        /*
         * WARNING:
         * Using directly p_dest here is tempting but keep in mind that it may
         * not have enough space to store the additional data due to alignment issues.
         * This would require to split the reading
         */
        rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(base_sector_addr,
                base_offset_algn/*need to respect flash quantum*/,
                flash_sector_buffer,
                ihandle->drv_desc.props.sector_size - base_offset_algn);
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                    ihandle->drv_desc.props.sector_size - base_offset_algn, base_sector_addr + base_offset_algn, rv);
            return FLASH_E_DRIVER_ERROR;
        }
        else
        {
            TRACE2("flash driver read %u bytes at 0x%0X",
                    ihandle->drv_desc.props.sector_size - base_offset_algn, base_sector_addr + base_offset_algn);
        }

        /* copy the data into the destination buffer, skipping any additional data present due to alignment */
        size = MIN(ihandle->drv_desc.props.sector_size - base_offset, length);
        memcpy(p_dest,
               flash_sector_buffer + (base_offset - base_offset_algn),
               size);
        length -= size;
        p_dest += size;

        // first sector has been dealt with
        base_sector_addr += ihandle->drv_desc.props.sector_size;
        num_sectors--;

        if (length == 0)
        {
            goto out;
        }
    }
    if (end_bytes_to_read_algn)
    {
        /* initialize working buffer with the Flash blank pattern */
        memset(flash_sector_buffer, ihandle->drv_desc.props.virgin_byte, sizeof(flash_sector_buffer));

        /* read first chunk of sector data */
        /*
         * WARNING:
         * Using directly p_dest here is tempting but keep in mind that it may
         * not have enough space to store the additional data due to alignment issues.
         * This would require to split the reading
         */
        rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(end_sector_addr,
                0,
                flash_sector_buffer,
                end_bytes_to_read_algn/*need to respect flash quantum*/);
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                    end_bytes_to_read_algn, end_sector_addr, rv);
            return FLASH_E_DRIVER_ERROR;
        }
        else
        {
            TRACE2("flash driver read %u bytes at 0x%0X", end_bytes_to_read_algn, end_sector_addr);
        }

        /* copy the data into the destination buffer, skipping any additional data present due to alignment */
        // NOTE: the whole "end" thing has been computed based on 'length', so (end_offset+1) <= length
        memcpy(p_dest + length - (end_offset+1),
               flash_sector_buffer,
               end_offset+1);

        length -= (end_offset + 1);

        // last sector has been dealt with
        end_sector_addr -= ihandle->drv_desc.props.sector_size;
        num_sectors--;
    }

    for (i = 0; i < num_sectors; ++i, base_sector_addr += ihandle->drv_desc.props.sector_size, p_dest += ihandle->drv_desc.props.sector_size)
    {
        rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(base_sector_addr,
                0,
                p_dest,
                ihandle->drv_desc.props.sector_size);
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            TRACE2("ERROR: flash driver failed at reading sector at 0x%0X with error code 0x%0X",
                    base_sector_addr, rv);
            return FLASH_E_DRIVER_ERROR;
        }
        else
        {
            TRACE1("flash driver read sector at 0x%0X", base_sector_addr);
        }
    }

out:
    return rv;
}



/* local helpers */
static int check_addr_range(FLASH_INTERNAL_HANDLE *ihandle, unsigned int p, unsigned int len)
{
    /* Check input parameters */
    if (p == 0)
    {
        TRACE("ERROR: NULL parameter provided");
        return FLASH_E_WRONG_PARAMS;
    }
    /* Check for address out of range */
    if (ihandle->drv_desc.props.flash_base > p)
    {
        TRACE2("ERROR: address 0x%0X is below flash area (starts at 0x%0X)", p, ihandle->drv_desc.props.flash_base);
        return FLASH_E_UNDERFLOW;
    }
    if ( (ihandle->drv_desc.props.flash_base + ihandle->drv_desc.props.flash_size) < (p + len))
    {
        TRACE2("ERROR: end address 0x%0X is beyond flash area (ends at 0x%0X)",
                p + len,
                ihandle->drv_desc.props.flash_base + ihandle->drv_desc.props.flash_size);
        return FLASH_E_OVERFLOW;
    }
    return FLASH_E_SUCCESS;
}

static int update_flash_with_buffer(FLASH_INTERNAL_HANDLE *ihandle, unsigned int p, unsigned int length, unsigned char *buf)
{
    int rv = FLASH_E_SUCCESS;
    unsigned int size = 0;
    unsigned int base_sector_addr = 0, end_sector_addr = 0, num_sectors = 0, i;
    unsigned int base_offset = 0, base_bytes_to_save = 0, base_bytes_to_save_algn = 0;
    unsigned int end_offset  = 0, end_offset_algn  = 0, end_bytes_to_save = 0;
    unsigned int aligned_end;
    unsigned int sector_mask = get_align_mask(ihandle->drv_desc.props.sector_size);

    /* check input parameters */
    rv = check_addr_range(ihandle, p, length);
    if (rv != FLASH_E_SUCCESS)
    {
        TRACE("ERROR: Invalid parameters");
        return rv;
    }

    /* compute address of 1st and last sector to erase */
    base_sector_addr = p & sector_mask;
    end_sector_addr  = (p + length - 1) & sector_mask;

    // the number of sectors to be erased
    num_sectors = (end_sector_addr - base_sector_addr)/ihandle->drv_desc.props.sector_size + 1;

    /*
     * NOTE:
     * Flash operates with addresses and offsets aligned on its data quantum.
     * 'xxx_offset' might not be aligned.
     * For things to work properly, align addresses and erase the little extra in the RAM buffer
     */
    aligned_end      = ptr_align((p + length - 1), ihandle->drv_desc.props.quantum_size) - ihandle->drv_desc.props.quantum_size; // quantum aligned, backwards
    // starting offset
    base_offset      = p & ~(sector_mask);
    // ending offset
    end_offset       = (p + length - 1) & ~(sector_mask);
    end_offset_algn  = aligned_end & ~(sector_mask);
    // bytes to save
    base_bytes_to_save      = base_offset;/*base_offset is the offset of the first erased/updated cell*/
    base_bytes_to_save_algn = ptr_align((unsigned int)base_bytes_to_save, ihandle->drv_desc.props.quantum_size);//not a pointer but the operation is the same
    end_bytes_to_save       = ihandle->drv_desc.props.sector_size - end_offset - 1/*end_offset is the offset of the last erased/updated cell*/;

    // NOTE: the 'length' variable is now only used as buf's length

    if (num_sectors == 1)
    {
        if (base_bytes_to_save || end_bytes_to_save || buf)
        {
            /* initialize working buffer with the Flash blank pattern */
            memset(flash_sector_buffer, ihandle->drv_desc.props.virgin_byte, sizeof(flash_sector_buffer));
        }
        /* first chunk of sector data must be saved */
        if (base_bytes_to_save)
        {
            rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(base_sector_addr,
                    0/*offset*/,
                    flash_sector_buffer,
                    base_bytes_to_save_algn/*qty of data must match flash quantum*/);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                        base_bytes_to_save_algn, base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE2("flash driver read %u bytes at 0x%0X", base_bytes_to_save_algn, base_sector_addr);
            }
            /* remove any additional data copied due to alignment corrections */
            if (base_bytes_to_save_algn - base_bytes_to_save)
            {
                memset(flash_sector_buffer + base_offset, ihandle->drv_desc.props.virgin_byte, base_bytes_to_save_algn - base_bytes_to_save);
            }
        }
        /* last chunk of sector data must be saved */
        if (end_bytes_to_save)
        {
            rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(end_sector_addr/*== base_sector_addr*/,
                    end_offset_algn,
                    &flash_sector_buffer[end_offset_algn],
                    ihandle->drv_desc.props.sector_size - end_offset_algn);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                        ihandle->drv_desc.props.sector_size - end_offset_algn, end_sector_addr + end_offset_algn, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE2("flash driver read %u bytes at 0x%0X",
                        ihandle->drv_desc.props.sector_size - end_offset_algn, end_sector_addr + end_offset_algn);
            }
            /* remove any additional data copied due to alignment corrections */
            memset(flash_sector_buffer + end_offset_algn, ihandle->drv_desc.props.virgin_byte, end_offset - end_offset_algn + 1);
        }
        /* treat the update buffer, if any */
        if (buf != NULL)
        {
            memcpy(flash_sector_buffer + base_offset, buf, length /*this is the 1-sector case*/);
        }

        /* erase the first sector */
        rv = ihandle->drv_desc.ops.flash_driver_sector_erase(base_sector_addr);
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            TRACE2("ERROR: flash driver failed at erasing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
            return FLASH_E_DRIVER_ERROR;
        }
        else
        {
            TRACE1("flash driver erased sector at 0x%0X", base_sector_addr);
        }

        /* write back any required data */
        if (base_bytes_to_save || end_bytes_to_save || buf)
        {
            /* program back the data that was saved */
            rv = ihandle->drv_desc.ops.flash_driver_sector_write_buf(base_sector_addr,
                    0/*offset*/,
                    flash_sector_buffer,
                    ihandle->drv_desc.props.sector_size);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at writing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver wrote sector at 0x%0X", base_sector_addr);
            }
        }
    }
    else
    {
        /* Prepare the erasure: handle 1st sector if p is not aligned on a sector boundary */
        if (base_bytes_to_save)
        {
            /* initialize working buffer with the Flash blank pattern */
            memset(flash_sector_buffer, ihandle->drv_desc.props.virgin_byte, sizeof(flash_sector_buffer));

            /* first chunk of sector data must be saved */
            rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(base_sector_addr,
                    0/*offset*/,
                    flash_sector_buffer,
                    base_bytes_to_save_algn/*qty of data must match flash quantum*/);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                        base_bytes_to_save_algn, base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE2("flash driver read %u bytes at 0x%0X", base_bytes_to_save_algn, base_sector_addr);
            }
            /* remove any additional data copied due to alignment corrections */
            if (base_bytes_to_save_algn - base_bytes_to_save)
            {
                memset(flash_sector_buffer + base_offset, ihandle->drv_desc.props.virgin_byte, base_bytes_to_save_algn - base_bytes_to_save);
            }
            /* treat the update buffer, if any */
            if (buf != NULL)
            {
                size = ihandle->drv_desc.props.sector_size - base_offset;
                memcpy(flash_sector_buffer + base_offset, buf, size/*multi sector case, so length is greater than this*/);

                length -= size; // multi sector case, so length is greater than this
                buf    += size; // buf is (unsigned char*), pointer arithmetic is OK then (buf++ <==> addr in buf + 1)
            }

            /* erase the first sector */
            rv = ihandle->drv_desc.ops.flash_driver_sector_erase(base_sector_addr);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at erasing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver erased sector at 0x%0X", base_sector_addr);
            }

            /* program back the data that was saved */
            rv = ihandle->drv_desc.ops.flash_driver_sector_write_buf(base_sector_addr,
                    0/*offset*/,
                    flash_sector_buffer,
                    /*NOTE: it is easier to write the whole sector, no quantum size issue*/
                    ihandle->drv_desc.props.sector_size);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at writing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver wrote sector at 0x%0X", base_sector_addr);
            }

            // first sector has been dealt with
            base_sector_addr += ihandle->drv_desc.props.sector_size;
            num_sectors--;
        }
        /* Prepare the erasure: handle last sector if (p+length) is not aligned on a sector boundary */
        if (end_bytes_to_save)
        {
            /* initialize working buffer with the Flash blank pattern */
            memset(flash_sector_buffer, ihandle->drv_desc.props.virgin_byte, sizeof(flash_sector_buffer));

            /* last chunk of sector data must be saved */
            rv = ihandle->drv_desc.ops.flash_driver_sector_read_buf(end_sector_addr,
                    end_offset_algn,
                    &flash_sector_buffer[end_offset_algn],
                    ihandle->drv_desc.props.sector_size - end_offset_algn);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE3("ERROR: flash driver failed at reading %u bytes at 0x%0X with error code 0x%0X",
                        ihandle->drv_desc.props.sector_size - end_offset_algn, end_sector_addr + end_offset_algn, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            {
                TRACE2("flash driver read %u bytes at 0x%0X",
                        ihandle->drv_desc.props.sector_size - end_offset_algn, end_sector_addr + end_offset_algn);
            }
            /* remove any additional data copied due to alignment corrections */
            memset(flash_sector_buffer + end_offset_algn, ihandle->drv_desc.props.virgin_byte, end_offset - end_offset_algn + 1);
            /* treat the update buffer, if any */
            if (buf != NULL)
            {
                size = end_offset + 1;
                memcpy(flash_sector_buffer,
                       (buf + length) - size,
                       size);

                length -= size;
            }

            /* erase the last sector */
            rv = ihandle->drv_desc.ops.flash_driver_sector_erase(end_sector_addr);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at erasing sector at 0x%0X with error code 0x%0X", end_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver erased sector at 0x%0X", end_sector_addr);
            }

            /* program back the data that was saved */
            rv = ihandle->drv_desc.ops.flash_driver_sector_write_buf(end_sector_addr,
                    0/*offset*/,
                    flash_sector_buffer,
                    /*NOTE: it is easier to write the whole sector, no quantum size issue*/
                    ihandle->drv_desc.props.sector_size);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at writing sector at 0x%0X with error code 0x%0X", end_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver wrote sector at 0x%0X", end_sector_addr);
            }

            // last sector has been dealt with
            end_sector_addr -= ihandle->drv_desc.props.sector_size;
            num_sectors--;
        }
        /* now erase sectors in-between, if any */
        /* note: at this point base_sector_addr is the address of the next whole sector to erase
         * and the last sector to erase is also a plain sector
         */
        for (i = 0; i < num_sectors; ++i, base_sector_addr += ihandle->drv_desc.props.sector_size)
        {
            rv = ihandle->drv_desc.ops.flash_driver_sector_erase(base_sector_addr);
            if (rv != FLASH_DRIVER_E_SUCCESS)
            {
                TRACE2("ERROR: flash driver failed at erasing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
                return FLASH_E_DRIVER_ERROR;
            }
            else
            {
                TRACE1("flash driver erased sector at 0x%0X", base_sector_addr);
            }

            /* program update data */
            if (buf)
            {
                rv = ihandle->drv_desc.ops.flash_driver_sector_write_buf(base_sector_addr,
                        0/*offset*/,
                        buf,
                        ihandle->drv_desc.props.sector_size);
                if (rv != FLASH_DRIVER_E_SUCCESS)
                {
                    TRACE2("ERROR: flash driver failed at writing sector at 0x%0X with error code 0x%0X", base_sector_addr, rv);
                    return FLASH_E_DRIVER_ERROR;
                }
                else
                {
                    TRACE1("flash driver wrote sector at 0x%0X", base_sector_addr);
                }
            }
        }
    }

    return rv;
}


unsigned char *TEST_FLASH_VALUE = (unsigned char *)ADDR_FLASH_SECTOR_8;

void testFlash(void)
{
    unsigned char test[1024];
    FLASH_DRIVER_DESC driver_desc;
    FLASH_HANDLE handle;
    unsigned int idx;

    for (idx = 0; idx < sizeof(test); idx++)
    {
        test[idx] = idx;
    }

    flash_init();
    flash_open(&driver_desc, &handle);

//    ssd1306ClearScreen(MAIN_AREA);
//    ssd1306Printf(5, 5, &Font_8x8, "value 0 = %d", TEST_FLASH_VALUE);
//    ssd1306Refresh(MAIN_AREA);
//    HAL_Delay(2000);

    flash_write(handle, TEST_FLASH_VALUE, test, sizeof(test));
    flash_read(handle, test, TEST_FLASH_VALUE, sizeof(test));

//    ssd1306ClearScreen(MAIN_AREA);
//    ssd1306Printf(5, 5, &Font_8x8, "value 1 = %d", test[0]);
//    ssd1306Refresh(MAIN_AREA);
//    HAL_Delay(2000);

    flash_close(handle);
    flash_terminate();
}
