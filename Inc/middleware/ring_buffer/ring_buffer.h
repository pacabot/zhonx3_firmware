/**************************************************************************/
/*!
    @file    ring_buffer.h
    @author  Nathy (PACABOT)
    @date    24/04/2015
    @version  0.0
 */
/**************************************************************************/

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

/* Module Identifier */
#include "config/module_id.h"

typedef struct
{
    // Pointer to an external buffer which will be allocated by the caller
    unsigned char *head;
    // Address of the end of the buffer
    unsigned char *tail;
    // Maximum length of the ring buffer
    unsigned int  max_len;
    // Pointer to the beginning of the available data
    unsigned char *data;
    // Length of available data
    unsigned int  data_len;
} ring_buffer_s;

// Minimum input buffer size
#define RING_BUFFER_MIN_SIZE    20

/*
 * Error Codes
 */
#define RING_BUFFER_E_SUCCESS       0
// Invalid argument
#define RING_BUFFER_E_INVAL         MAKE_ERROR(RING_BUFFER_MODULE_ID, 1)
// No more space into the Ring Buffer
#define RING_BUFFER_E_OVERFLOW      MAKE_ERROR(RING_BUFFER_MODULE_ID, 2)
// Write error
#define RING_BUFFER_E_WRITE_ERROR   MAKE_ERROR(RING_BUFFER_MODULE_ID, 3)
// Read error
#define RING_BUFFER_E_READ_ERROR    MAKE_ERROR(RING_BUFFER_MODULE_ID, 4)
// General error
#define RING_BUFFER_E_ERROR         MAKE_ERROR(RING_BUFFER_MODULE_ID, 5)

/*
 * @brief   Initializes Ring buffer modules
 *
 * @param   input_buffer      Valid pointer toward an allocated external buffer
 * @param   input_buffer_len  Length of input_buffer
 *
 * @return  RING_BUFFER_E_SUCCESS on success,
 *          a negative value otherwise
 */
int RingBuffer_init(unsigned char *input_buffer, unsigned int input_buffer_len);

/*
 * @brief   Writes data into Ring Buffer
 *
 * @param   buffer    Valid pointer toward the buffer to write from
 * @param   length    Length of the buffer to write from
 *
 * @return  Upon successful return, the number of bytes written,
 *          a negative value otherwise
 */
int RingBuffer_write(unsigned char *buffer, unsigned int length);

/*
 * @brief   Reads data from Ring Buffer
 *
 * @param   out_buffer  Valid pointer toward the buffer to write to
 * @param   length      Desired maximum amount of data to read from Ring Buffer
 *
 * @return  Upon successful return, the number of bytes read,
 *          a negative value otherwise
 */
int RingBuffer_get_data(unsigned char *out_buffer, unsigned int length);

/*
 * @brief   Returns the available amount of data (in bytes)
 *
 * @param   none
 *
 * @return  Number of available bytes into the Ring Buffer
 */
unsigned int RingBuffer_get_data_len(void);

#endif // __RING_BUFFER_H__
