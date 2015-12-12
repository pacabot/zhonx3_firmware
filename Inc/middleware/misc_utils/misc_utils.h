/**************************************************************************/
/*!
    @file    misc_utils.h
    @author  Netanel (PACABOT)
    @date    08/10/2015
    @version 0.1
 */
/**************************************************************************/

#ifndef __MISC_UTILS_H__
#define __MISC_UTILS_H__

#define _inline_ inline

/** @NOTE: alignment MUST be a power of 2 */
unsigned int    ptr_align(unsigned int ptr, size_t alignment);
int          is_ptr_aligned(unsigned int ptr, size_t alignment);
unsigned int    get_align_mask(size_t alignment);

#endif // __MISC_UTILS_H__

