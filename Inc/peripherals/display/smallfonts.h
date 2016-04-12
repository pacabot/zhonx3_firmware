/**************************************************************************/
/*! 
 @file     smallfonts.h
 @author   PLF (PACABOT)
 @date     26 March 2011
 @version  0.10
 */
/**************************************************************************/
#ifndef __SMALLFONTS_H_
#define __SMALLFONTS_H_

/* Current version by ZHONX */
/* Last Updated: 22 November 2012 */

typedef struct
{
    unsigned char u8Width; /* Character width for storage         */
    unsigned char u8Height; /* Character height for storage        */
    unsigned char u8FirstChar; /* The first character available       */
    unsigned char u8LastChar; /* The last character available        */
    const unsigned char *au8FontTable; /* Font table start address in memory  */
} FONT_DEF;

extern const FONT_DEF Font_3x6;
extern const FONT_DEF Font_5x8;
extern const FONT_DEF Font_7x8;
extern const FONT_DEF Font_8x8;
extern const FONT_DEF Font_8x8Thin;

extern const unsigned char au8Font3x6[];
extern const unsigned char au8Font5x8[];
extern const unsigned char au8Font7x8[];
extern const unsigned char au8Font8x8[];
extern const unsigned char au8Font8x8Thin[];

#endif
