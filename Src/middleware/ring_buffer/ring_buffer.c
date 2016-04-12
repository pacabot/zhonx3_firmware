/**************************************************************************/
/*!
 @file    ring_buffer.c
 @author  Nathy (PACABOT)
 @date    24/04/2015
 @version  0.0
 */
/**************************************************************************/

/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Declarations for this module */
#include "middleware/ring_buffer/ring_buffer.h"

// Declare Ring Buffer variable
static ring_buffer_s ring_buffer;

int RingBuffer_init(char *input_buffer, int input_buffer_len)
{
    // Perform sanity checks
    if ((input_buffer == NULL) || (input_buffer_len < RING_BUFFER_MIN_SIZE))
    {
        // Invalid argument
        return RING_BUFFER_E_INVAL;
    }

    // Initialize Ring Buffer structure

    // Register the external buffer and set its maximum length
    ring_buffer.head = input_buffer;
    ring_buffer.max_len = input_buffer_len;
    // Compute the end address of the buffer
    ring_buffer.tail = (ring_buffer.data + ring_buffer.max_len);
    // Initialize the available data length
    ring_buffer.data_len = 0;
    // Initialize the internal pointer
    ring_buffer.data = ring_buffer.head;

    return RING_BUFFER_E_SUCCESS;
}

int RingBuffer_read_byte(char *read_byte)
{
    // Check if there is something to read
    if (ring_buffer.data_len == 0)
    {
        return RING_BUFFER_E_READ_ERROR;
    }

    *read_byte = *ring_buffer.data;

    // Increment data pointer
    ring_buffer.data++;

    // Ensure that the buffer is not accessed out of bounds
    if (ring_buffer.data > ring_buffer.tail)
    {
        ring_buffer.data = ring_buffer.head;
    }

    // Decrement available length
    ring_buffer.data_len--;

    return RING_BUFFER_E_SUCCESS;
}

int RingBuffer_read(char *out_buffer, int length)
{
    int len_until_queue = 0;

    // Sanity checks
    if ((out_buffer == NULL) || (length <= 0))
    {
        return 0;
    }

    // Check if there is something to read
    if (ring_buffer.data_len == 0)
    {
        return 0;
    }

    // Limit length to the available number of bytes
    if (length > ring_buffer.data_len)
    {
        length = ring_buffer.data_len;
    }

    // Calculate the remaining length to reach the buffer queue
    len_until_queue = (ring_buffer.tail - ring_buffer.data);

    // Check how many operations are necessary to read the entire buffer size
    if (length > len_until_queue)
    {
        // 2 Operations are necessary

        // Write first part until buffer queue
        memcpy(out_buffer, ring_buffer.data, len_until_queue);
        // Write second part from buffer head
        memcpy(out_buffer + len_until_queue, ring_buffer.head, length - len_until_queue);

        // Update pointer toward beginning available data
        ring_buffer.data = ring_buffer.head + (length - len_until_queue) + 1;
    }
    else
    {
        // 1 operation is necessary
        memcpy(ring_buffer.data, out_buffer, length);

        // Update pointer toward beginning available data
        ring_buffer.data += (length + 1);
    }

    // Ensure that the buffer is not accessed out of bounds
    if (ring_buffer.data > ring_buffer.tail)
    {
        ring_buffer.data = ring_buffer.head;
    }

    // Update available length
    ring_buffer.data_len -= length;

    return length;
}

int RingBuffer_write_byte(char byte)
{
    int remaining_length = (ring_buffer.max_len - ring_buffer.data_len);

    // Check if the buffer is full
    if (remaining_length <= 0)
    {
        return RING_BUFFER_E_OVERFLOW;
    }

    // Write the byte into the Ring buffer
    *(ring_buffer.data) = byte;

    // Increment the available data length
    ring_buffer.data_len++;

    return RING_BUFFER_E_SUCCESS;
}

int RingBuffer_write(const char *buffer, int length)
{
    int remaining_length = (ring_buffer.max_len - ring_buffer.data_len);
    int len_until_queue = 0;

    // Sanity checks
    if ((buffer == NULL) || (length <= 0))
    {
        return 0;
    }

    // Check if the buffer is full
    if (ring_buffer.data_len >= ring_buffer.max_len)
    {
        return RING_BUFFER_E_OVERFLOW;
    }

    // Check if there is sufficient space
    if (length > remaining_length)
    {
        // Limit length to remaining length
        length = remaining_length;
    }

    // Calculate the remaining length to reach the buffer queue
    len_until_queue = (ring_buffer.tail - ring_buffer.data);

    // Check how many operations are necessary to write the entire buffer
    if (length > len_until_queue)
    {
        // 2 Operations are necessary

        // Write first part until buffer queue
        memcpy(ring_buffer.data, buffer, len_until_queue);
        // Write second part from buffer head
        memcpy(ring_buffer.head, buffer + len_until_queue, length - len_until_queue);
    }
    else
    {
        // 1 operation is necessary
        memcpy(ring_buffer.data, buffer, length);
    }

    // Update the available data length
    ring_buffer.data_len += length;

    // Return the number of bytes written
    return length;
}

int RingBuffer_get_data_len(void)
{
    return ring_buffer.data_len;
}

/*****************************************************************************
 * TEST FUNCTIONS
 *****************************************************************************/

void RingBuffer_test(void)
{
    // Return Value
    int rv;
    // External buffer to register as Ring Buffer
    char ext_buffer[20] = { 0 };
    // Temporary buffers for test purposes
    char read_buffer[40] = { 0 };
    char write_buffer[40] = { 0 };

    // Initialize Ring Buffer module
    rv = RingBuffer_init(ext_buffer, sizeof(ext_buffer));
    if (rv != RING_BUFFER_E_SUCCESS)
    {
        // TODO: Complete this statement
    }

    // Write 20 bytes to the Temporary Write buffer
    strcat(write_buffer, "Test Ring Buffer 20");

    // Write 10 bytes to the Ring Buffer
    rv = RingBuffer_write(write_buffer, 10);
    if (rv != 10)
    {
        // TODO: Complete this statement
    }

    // Read-back 5 first bytes from the Ring buffer
    rv = RingBuffer_read(read_buffer, 5);
    if (rv != 5)
    {
        // TODO: Complete this statement
    }

    // Read-back 5 last bytes from the Ring Buffer
    rv = RingBuffer_read(read_buffer + 5, 5);
    if (rv != 5)
    {
        // TODO: Complete this statement
    }

    // Compare the read byte with initial written data
    rv = memcmp(write_buffer, read_buffer, 10);
    if (rv != 0)
    {
        // Buffers are different, test FAILED
        // TODO: Complete this statement
    }
    else
    {
        // Buffers are identical, test PASSED
        // TODO: Complete this statement
    }
}

