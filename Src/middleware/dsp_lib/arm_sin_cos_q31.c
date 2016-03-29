/* ----------------------------------------------------------------------    
 * Copyright (C) 2010-2013 ARM Limited. All rights reserved.    
 *    
 * $Date:        17. January 2013
 * $Revision: 	V1.4.1
 *    
 * Project: 	    CMSIS DSP Library    
 * Title:		arm_sin_cos_q31.c    
 *    
 * Description:	Cosine & Sine calculation for Q31 values.   
 *    
 * Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
 *  
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met:
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the 
 *     distribution.
 *   - Neither the name of ARM LIMITED nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.    
 * -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupController    
 */

/**    
 * @addtogroup SinCos    
 * @{    
 */

/**    
 * \par    
 * Sine Table is generated from following loop    
 * <pre>for(i = 0; i < 360; i++)    
 * {    
 *    sinTable[i]= sin((i-180) * PI/180.0);    
 * } </pre>   
 * Convert above coefficients to fixed point 1.31 format.    
 */

static const int32_t sinTableQ31[360] = {

0x0,
                                          0xfdc41e9b, 0xfb8869ce, 0xf94d0e2e, 0xf7123849, 0xf4d814a4, 0xf29ecfb2,
                                          0xf06695da, 0xee2f9369, 0xebf9f498, 0xe9c5e582, 0xe7939223, 0xe5632654,
                                          0xe334cdc9, 0xe108b40d, 0xdedf047d, 0xdcb7ea46, 0xda939061, 0xd8722192,
                                          0xd653c860, 0xd438af17, 0xd220ffc0, 0xd00ce422, 0xcdfc85bb, 0xcbf00dbe,
                                          0xc9e7a512, 0xc7e3744b, 0xc5e3a3a9, 0xc3e85b18, 0xc1f1c224, 0xc0000000,
                                          0xbe133b7c, 0xbc2b9b05, 0xba4944a2, 0xb86c5df0, 0xb6950c1e, 0xb4c373ee,
                                          0xb2f7b9af, 0xb1320139, 0xaf726def, 0xadb922b7, 0xac0641fb, 0xaa59eda4,
                                          0xa8b4471a, 0xa7156f3c, 0xa57d8666, 0xa3ecac65, 0xa263007d, 0xa0e0a15f,
                                          0x9f65ad2d, 0x9df24175, 0x9c867b2c, 0x9b2276b0, 0x99c64fc5, 0x98722192,
                                          0x9726069c, 0x95e218c9, 0x94a6715d, 0x937328f5, 0x92485786, 0x9126145f,
                                          0x900c7621, 0x8efb92c2, 0x8df37f8b, 0x8cf45113, 0x8bfe1b3f, 0x8b10f144,
                                          0x8a2ce59f, 0x89520a1a, 0x88806fc4, 0x87b826f7, 0x86f93f50, 0x8643c7b3,
                                          0x8597ce46, 0x84f56073, 0x845c8ae3, 0x83cd5982, 0x8347d77b, 0x82cc0f36,
                                          0x825a0a5b, 0x81f1d1ce, 0x81936daf, 0x813ee55b, 0x80f43f69, 0x80b381ac,
                                          0x807cb130, 0x804fd23a, 0x802ce84c, 0x8013f61d, 0x8004fda0, 0x80000000,
                                          0x8004fda0, 0x8013f61d, 0x802ce84c, 0x804fd23a, 0x807cb130, 0x80b381ac,
                                          0x80f43f69, 0x813ee55b, 0x81936daf, 0x81f1d1ce, 0x825a0a5b, 0x82cc0f36,
                                          0x8347d77b, 0x83cd5982, 0x845c8ae3, 0x84f56073, 0x8597ce46, 0x8643c7b3,
                                          0x86f93f50, 0x87b826f7, 0x88806fc4, 0x89520a1a, 0x8a2ce59f, 0x8b10f144,
                                          0x8bfe1b3f, 0x8cf45113, 0x8df37f8b, 0x8efb92c2, 0x900c7621, 0x9126145f,
                                          0x92485786, 0x937328f5, 0x94a6715d, 0x95e218c9, 0x9726069c, 0x98722192,
                                          0x99c64fc5, 0x9b2276b0, 0x9c867b2c, 0x9df24175, 0x9f65ad2d, 0xa0e0a15f,
                                          0xa263007d, 0xa3ecac65, 0xa57d8666, 0xa7156f3c, 0xa8b4471a, 0xaa59eda4,
                                          0xac0641fb, 0xadb922b7, 0xaf726def, 0xb1320139, 0xb2f7b9af, 0xb4c373ee,
                                          0xb6950c1e, 0xb86c5df0, 0xba4944a2, 0xbc2b9b05, 0xbe133b7c, 0xc0000000,
                                          0xc1f1c224, 0xc3e85b18, 0xc5e3a3a9, 0xc7e3744b, 0xc9e7a512, 0xcbf00dbe,
                                          0xcdfc85bb, 0xd00ce422, 0xd220ffc0, 0xd438af17, 0xd653c860, 0xd8722192,
                                          0xda939061, 0xdcb7ea46, 0xdedf047d, 0xe108b40d, 0xe334cdc9, 0xe5632654,
                                          0xe7939223, 0xe9c5e582, 0xebf9f498, 0xee2f9369, 0xf06695da, 0xf29ecfb2,
                                          0xf4d814a4, 0xf7123849, 0xf94d0e2e, 0xfb8869ce, 0xfdc41e9b, 0x0, 0x23be165,
                                          0x4779632, 0x6b2f1d2, 0x8edc7b7, 0xb27eb5c, 0xd61304e, 0xf996a26, 0x11d06c97,
                                          0x14060b68, 0x163a1a7e, 0x186c6ddd, 0x1a9cd9ac, 0x1ccb3237, 0x1ef74bf3,
                                          0x2120fb83, 0x234815ba, 0x256c6f9f, 0x278dde6e, 0x29ac37a0, 0x2bc750e9,
                                          0x2ddf0040, 0x2ff31bde, 0x32037a45, 0x340ff242, 0x36185aee, 0x381c8bb5,
                                          0x3a1c5c57, 0x3c17a4e8, 0x3e0e3ddc, 0x40000000, 0x41ecc484, 0x43d464fb,
                                          0x45b6bb5e, 0x4793a210, 0x496af3e2, 0x4b3c8c12, 0x4d084651, 0x4ecdfec7,
                                          0x508d9211, 0x5246dd49, 0x53f9be05, 0x55a6125c, 0x574bb8e6, 0x58ea90c4,
                                          0x5a82799a, 0x5c13539b, 0x5d9cff83, 0x5f1f5ea1, 0x609a52d3, 0x620dbe8b,
                                          0x637984d4, 0x64dd8950, 0x6639b03b, 0x678dde6e, 0x68d9f964, 0x6a1de737,
                                          0x6b598ea3, 0x6c8cd70b, 0x6db7a87a, 0x6ed9eba1, 0x6ff389df, 0x71046d3e,
                                          0x720c8075, 0x730baeed, 0x7401e4c1, 0x74ef0ebc, 0x75d31a61, 0x76adf5e6,
                                          0x777f903c, 0x7847d909, 0x7906c0b0, 0x79bc384d, 0x7a6831ba, 0x7b0a9f8d,
                                          0x7ba3751d, 0x7c32a67e, 0x7cb82885, 0x7d33f0ca, 0x7da5f5a5, 0x7e0e2e32,
                                          0x7e6c9251, 0x7ec11aa5, 0x7f0bc097, 0x7f4c7e54, 0x7f834ed0, 0x7fb02dc6,
                                          0x7fd317b4, 0x7fec09e3, 0x7ffb0260, 0x7fffffff, 0x7ffb0260, 0x7fec09e3,
                                          0x7fd317b4, 0x7fb02dc6, 0x7f834ed0, 0x7f4c7e54, 0x7f0bc097, 0x7ec11aa5,
                                          0x7e6c9251, 0x7e0e2e32, 0x7da5f5a5, 0x7d33f0ca, 0x7cb82885, 0x7c32a67e,
                                          0x7ba3751d, 0x7b0a9f8d, 0x7a6831ba, 0x79bc384d, 0x7906c0b0, 0x7847d909,
                                          0x777f903c, 0x76adf5e6, 0x75d31a61, 0x74ef0ebc, 0x7401e4c1, 0x730baeed,
                                          0x720c8075, 0x71046d3e, 0x6ff389df, 0x6ed9eba1, 0x6db7a87a, 0x6c8cd70b,
                                          0x6b598ea3, 0x6a1de737, 0x68d9f964, 0x678dde6e, 0x6639b03b, 0x64dd8950,
                                          0x637984d4, 0x620dbe8b, 0x609a52d3, 0x5f1f5ea1, 0x5d9cff83, 0x5c13539b,
                                          0x5a82799a, 0x58ea90c4, 0x574bb8e6, 0x55a6125c, 0x53f9be05, 0x5246dd49,
                                          0x508d9211, 0x4ecdfec7, 0x4d084651, 0x4b3c8c12, 0x496af3e2, 0x4793a210,
                                          0x45b6bb5e, 0x43d464fb, 0x41ecc484, 0x40000000, 0x3e0e3ddc, 0x3c17a4e8,
                                          0x3a1c5c57, 0x381c8bb5, 0x36185aee, 0x340ff242, 0x32037a45, 0x2ff31bde,
                                          0x2ddf0040, 0x2bc750e9, 0x29ac37a0, 0x278dde6e, 0x256c6f9f, 0x234815ba,
                                          0x2120fb83, 0x1ef74bf3, 0x1ccb3237, 0x1a9cd9ac, 0x186c6ddd, 0x163a1a7e,
                                          0x14060b68, 0x11d06c97, 0xf996a26, 0xd61304e, 0xb27eb5c, 0x8edc7b7, 0x6b2f1d2,
                                          0x4779632, 0x23be165,

};

/**    
 * \par    
 * Cosine Table is generated from following loop    
 * <pre>for(i = 0; i < 360; i++)    
 * {    
 *    cosTable[i]= cos((i-180) * PI/180.0);    
 * } </pre>   
 * \par    
 * Convert above coefficients to fixed point 1.31 format.    
 */
static const int32_t cosTableQ31[360] = { 0x80000000, 0x8004fda0, 0x8013f61d, 0x802ce84c, 0x804fd23a, 0x807cb130,
                                          0x80b381ac, 0x80f43f69, 0x813ee55b, 0x81936daf, 0x81f1d1ce, 0x825a0a5b,
                                          0x82cc0f36, 0x8347d77b, 0x83cd5982, 0x845c8ae3, 0x84f56073, 0x8597ce46,
                                          0x8643c7b3, 0x86f93f50, 0x87b826f7, 0x88806fc4, 0x89520a1a, 0x8a2ce59f,
                                          0x8b10f144, 0x8bfe1b3f, 0x8cf45113, 0x8df37f8b, 0x8efb92c2, 0x900c7621,
                                          0x9126145f, 0x92485786, 0x937328f5, 0x94a6715d, 0x95e218c9, 0x9726069c,
                                          0x98722192, 0x99c64fc5, 0x9b2276b0, 0x9c867b2c, 0x9df24175, 0x9f65ad2d,
                                          0xa0e0a15f, 0xa263007d, 0xa3ecac65, 0xa57d8666, 0xa7156f3c, 0xa8b4471a,
                                          0xaa59eda4, 0xac0641fb, 0xadb922b7, 0xaf726def, 0xb1320139, 0xb2f7b9af,
                                          0xb4c373ee, 0xb6950c1e, 0xb86c5df0, 0xba4944a2, 0xbc2b9b05, 0xbe133b7c,
                                          0xc0000000, 0xc1f1c224, 0xc3e85b18, 0xc5e3a3a9, 0xc7e3744b, 0xc9e7a512,
                                          0xcbf00dbe, 0xcdfc85bb, 0xd00ce422, 0xd220ffc0, 0xd438af17, 0xd653c860,
                                          0xd8722192, 0xda939061, 0xdcb7ea46, 0xdedf047d, 0xe108b40d, 0xe334cdc9,
                                          0xe5632654, 0xe7939223, 0xe9c5e582, 0xebf9f498, 0xee2f9369, 0xf06695da,
                                          0xf29ecfb2, 0xf4d814a4, 0xf7123849, 0xf94d0e2e, 0xfb8869ce, 0xfdc41e9b, 0x0,
                                          0x23be165, 0x4779632, 0x6b2f1d2, 0x8edc7b7, 0xb27eb5c, 0xd61304e, 0xf996a26,
                                          0x11d06c97, 0x14060b68, 0x163a1a7e, 0x186c6ddd, 0x1a9cd9ac, 0x1ccb3237,
                                          0x1ef74bf3, 0x2120fb83, 0x234815ba, 0x256c6f9f, 0x278dde6e, 0x29ac37a0,
                                          0x2bc750e9, 0x2ddf0040, 0x2ff31bde, 0x32037a45, 0x340ff242, 0x36185aee,
                                          0x381c8bb5, 0x3a1c5c57, 0x3c17a4e8, 0x3e0e3ddc, 0x40000000, 0x41ecc484,
                                          0x43d464fb, 0x45b6bb5e, 0x4793a210, 0x496af3e2, 0x4b3c8c12, 0x4d084651,
                                          0x4ecdfec7, 0x508d9211, 0x5246dd49, 0x53f9be05, 0x55a6125c, 0x574bb8e6,
                                          0x58ea90c4, 0x5a82799a, 0x5c13539b, 0x5d9cff83, 0x5f1f5ea1, 0x609a52d3,
                                          0x620dbe8b, 0x637984d4, 0x64dd8950, 0x6639b03b, 0x678dde6e, 0x68d9f964,
                                          0x6a1de737, 0x6b598ea3, 0x6c8cd70b, 0x6db7a87a, 0x6ed9eba1, 0x6ff389df,
                                          0x71046d3e, 0x720c8075, 0x730baeed, 0x7401e4c1, 0x74ef0ebc, 0x75d31a61,
                                          0x76adf5e6, 0x777f903c, 0x7847d909, 0x7906c0b0, 0x79bc384d, 0x7a6831ba,
                                          0x7b0a9f8d, 0x7ba3751d, 0x7c32a67e, 0x7cb82885, 0x7d33f0ca, 0x7da5f5a5,
                                          0x7e0e2e32, 0x7e6c9251, 0x7ec11aa5, 0x7f0bc097, 0x7f4c7e54, 0x7f834ed0,
                                          0x7fb02dc6, 0x7fd317b4, 0x7fec09e3, 0x7ffb0260, 0x7fffffff, 0x7ffb0260,
                                          0x7fec09e3, 0x7fd317b4, 0x7fb02dc6, 0x7f834ed0, 0x7f4c7e54, 0x7f0bc097,
                                          0x7ec11aa5, 0x7e6c9251, 0x7e0e2e32, 0x7da5f5a5, 0x7d33f0ca, 0x7cb82885,
                                          0x7c32a67e, 0x7ba3751d, 0x7b0a9f8d, 0x7a6831ba, 0x79bc384d, 0x7906c0b0,
                                          0x7847d909, 0x777f903c, 0x76adf5e6, 0x75d31a61, 0x74ef0ebc, 0x7401e4c1,
                                          0x730baeed, 0x720c8075, 0x71046d3e, 0x6ff389df, 0x6ed9eba1, 0x6db7a87a,
                                          0x6c8cd70b, 0x6b598ea3, 0x6a1de737, 0x68d9f964, 0x678dde6e, 0x6639b03b,
                                          0x64dd8950, 0x637984d4, 0x620dbe8b, 0x609a52d3, 0x5f1f5ea1, 0x5d9cff83,
                                          0x5c13539b, 0x5a82799a, 0x58ea90c4, 0x574bb8e6, 0x55a6125c, 0x53f9be05,
                                          0x5246dd49, 0x508d9211, 0x4ecdfec7, 0x4d084651, 0x4b3c8c12, 0x496af3e2,
                                          0x4793a210, 0x45b6bb5e, 0x43d464fb, 0x41ecc484, 0x40000000, 0x3e0e3ddc,
                                          0x3c17a4e8, 0x3a1c5c57, 0x381c8bb5, 0x36185aee, 0x340ff242, 0x32037a45,
                                          0x2ff31bde, 0x2ddf0040, 0x2bc750e9, 0x29ac37a0, 0x278dde6e, 0x256c6f9f,
                                          0x234815ba, 0x2120fb83, 0x1ef74bf3, 0x1ccb3237, 0x1a9cd9ac, 0x186c6ddd,
                                          0x163a1a7e, 0x14060b68, 0x11d06c97, 0xf996a26, 0xd61304e, 0xb27eb5c,
                                          0x8edc7b7, 0x6b2f1d2, 0x4779632, 0x23be165, 0x0, 0xfdc41e9b, 0xfb8869ce,
                                          0xf94d0e2e, 0xf7123849, 0xf4d814a4, 0xf29ecfb2, 0xf06695da, 0xee2f9369,
                                          0xebf9f498, 0xe9c5e582, 0xe7939223, 0xe5632654, 0xe334cdc9, 0xe108b40d,
                                          0xdedf047d, 0xdcb7ea46, 0xda939061, 0xd8722192, 0xd653c860, 0xd438af17,
                                          0xd220ffc0, 0xd00ce422, 0xcdfc85bb, 0xcbf00dbe, 0xc9e7a512, 0xc7e3744b,
                                          0xc5e3a3a9, 0xc3e85b18, 0xc1f1c224, 0xc0000000, 0xbe133b7c, 0xbc2b9b05,
                                          0xba4944a2, 0xb86c5df0, 0xb6950c1e, 0xb4c373ee, 0xb2f7b9af, 0xb1320139,
                                          0xaf726def, 0xadb922b7, 0xac0641fb, 0xaa59eda4, 0xa8b4471a, 0xa7156f3c,
                                          0xa57d8666, 0xa3ecac65, 0xa263007d, 0xa0e0a15f, 0x9f65ad2d, 0x9df24175,
                                          0x9c867b2c, 0x9b2276b0, 0x99c64fc5, 0x98722192, 0x9726069c, 0x95e218c9,
                                          0x94a6715d, 0x937328f5, 0x92485786, 0x9126145f, 0x900c7621, 0x8efb92c2,
                                          0x8df37f8b, 0x8cf45113, 0x8bfe1b3f, 0x8b10f144, 0x8a2ce59f, 0x89520a1a,
                                          0x88806fc4, 0x87b826f7, 0x86f93f50, 0x8643c7b3, 0x8597ce46, 0x84f56073,
                                          0x845c8ae3, 0x83cd5982, 0x8347d77b, 0x82cc0f36, 0x825a0a5b, 0x81f1d1ce,
                                          0x81936daf, 0x813ee55b, 0x80f43f69, 0x80b381ac, 0x807cb130, 0x804fd23a,
                                          0x802ce84c, 0x8013f61d, 0x8004fda0,

};

/**    
 * @brief  Q31 sin_cos function.   
 * @param[in]  theta    scaled input value in degrees    
 * @param[out] *pSinVal points to the processed sine output.    
 * @param[out] *pCosVal points to the processed cosine output.    
 * @return none.   
 *    
 * The Q31 input value is in the range [-1 0.999999] and is mapped to a degree value in the range [-180 179].   
 *    
 */

void arm_sin_cos_q31(q31_t theta, q31_t * pSinVal, q31_t * pCosVal)
{
    q31_t x0; /* Nearest input value */
    q31_t y0, y1; /* Nearest output values */
    q31_t xSpacing = INPUT_SPACING; /* Spaing between inputs */
    uint32_t i; /* Index */
    q31_t oneByXSpacing; /* 1/ xSpacing value */
    q31_t out; /* temporary variable */
    uint32_t sign_bits; /* No.of sign bits */
    uint32_t firstX = 0x80000000; /* First X value */

    /* Calculation of index */
    i = ((uint32_t) theta - firstX) / (uint32_t) xSpacing;

    /* Checking min and max index of table */
    if (i >= 359)
    {
        i = 358;
    }

    /* Calculation of first nearest input value */
    x0 = (q31_t) firstX + ((q31_t) i * xSpacing);

    /* Reading nearest sine output values from table */
    y0 = sinTableQ31[i];
    y1 = sinTableQ31[i + 1u];

    /* Calculation of 1/(x1-x0) */
    /* (x1-x0) is xSpacing which is fixed value */
    sign_bits = 8u;
    oneByXSpacing = 0x5A000000;

    /* Calculation of (theta - x0)/(x1-x0) */
    out = (((q31_t) (((q63_t) (theta - x0) * oneByXSpacing) >> 32)) << sign_bits);

    /* Calculation of y0 + (y1 - y0) * ((theta - x0)/(x1-x0)) */
    *pSinVal = __QADD(y0, ((q31_t) (((q63_t) (y1 - y0) * out) >> 30)));

    /* Reading nearest cosine output values from table */
    y0 = cosTableQ31[i];
    y1 = cosTableQ31[i + 1u];

    /* Calculation of y0 + (y1 - y0) * ((theta - x0)/(x1-x0)) */
    *pCosVal = __QADD(y0, ((q31_t) (((q63_t) (y1 - y0) * out) >> 30)));

}

/**    
 * @} end of SinCos group    
 */
