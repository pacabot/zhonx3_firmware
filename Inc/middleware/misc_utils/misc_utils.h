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

#ifndef uintptr_t
    typedef unsigned int uintptr_t;
#endif

#define _inline_ inline

/** @NOTE: alignment MUST be a power of 2 */
_inline_ uintptr_t    ptr_align(uintptr_t ptr, size_t alignment);
_inline_ int          is_ptr_aligned(uintptr_t ptr, size_t alignment);
_inline_ uintptr_t    get_align_mask(size_t alignment);

#endif // __MISC_UTILS_H__

