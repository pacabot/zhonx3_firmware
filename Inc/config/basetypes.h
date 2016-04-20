#ifndef __BASETYPES_H__
#define __BASETYPES_H__

typedef struct
{
    void *p_value;        // Parameter to modify
    void *presetBuffer;   // Preset buffer for the the parameter
    int (*callback)(int param1, void *param2);
} presetParam;

/** This constant defines an empty pointer value */
#ifndef null
# define null	((void *)0)
#endif

#ifndef NULL
# define NULL 	0
#endif

#ifndef ON
# define ON 	1
#endif

#ifndef OFF
# define OFF 	0
#endif

#ifndef true
#define true	1
#endif
#ifndef false
#define false	0
#endif

#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE	0
#endif

/*
 * Utility macros
 */

#ifndef MAX
#define MAX(a, b)	((a)>(b)?(a):(b))
#endif

#ifndef MIN
#define MIN(a, b)	((a)<(b)?(a):(b))
#endif

#ifndef SIGN
#define SIGN(a)	((a)<(0)?(-1):(1))
#endif

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#define reverse_bit(val) ((val) ^ 0x00000001)
#define reverse_bits(val) ((val) ^ 0xffffffff)
#define check_bit(val, mask) ((((val) & (mask)) == (mask)) ? TRUE : FALSE)
#define check_bit_key(val, mask, key) ((((val) & (mask)) == (key)) ? TRUE : FALSE)
#define set_bit(val, mask) ((val) |= (mask))
#define reset_bit(val, mask) ((val) &= (reverse_bits(mask)))
#define get_bit(val, mask) ((val) & (mask))

/** Timeout constant indicating an infinite time */
#define FOREVER     0xFFFFFFFFL

/** Timeout constant indicating a 0 timeout */
#define IMMEDIATE   0x00000000L

#endif // __BASETYPES_H__
