/*---------------------------------------------------------------------------
 *
 *      pictures.h
 *
 *---------------------------------------------------------------------------*/

#ifndef __PICTURES_H__
#define __PICTURES_H__

// TODO: Re-declare correctly these global variables

//w = 13 h = 7
const unsigned char battery[13] =
{
		0XFE,0X82,0X82,0X82,0X82,0X82,0X82,0X82,0X82,0X82,0X82,0XFE,0X38
};

//w = 13 h = 7
const unsigned char batterySationPowered0n[13] =
{
		0XFE,0X82,0X82,0X02,0X92,0X7A,0XBC,0X92,0X80,0X82,0X82,0XFE,0X38
};

//w = 13 h = 7
const unsigned char batterySationPowered0ff[13] =
{
		0XFE,0X82,0X82,0X02,0X02,0X02,0X80,0X80,0X80,0X82,0X82,0XFE,0X38
};


static const unsigned char pic_battery_level[] =
{
		0XFC,0X84,0X84,0X84,0X84,0X84,0XFC,0X30
};

//w = 128 h = 40
static const unsigned char Pacabot_bmp[] =
{
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,0X07,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,
		0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XFF,0XFF,0XFF,0XFE,0XFE,0X00,0X00,0XC6,
		0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,
		0XC7,0X07,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0XE6,0XE6,0XE7,0XE7,0XE7,
		0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,
		0XE7,0XE7,0XFF,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0XC6,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,
		0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XE7,0XC7,0X07,
		0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,0X07,0X07,0X07,0X07,0X07,0X07,0X06,0X00,
		0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X79,0X79,0X79,0X79,0X79,0X79,0X79,
		0X79,0X79,0X79,0X79,0X79,0X79,0X79,0X79,0XF9,0XF9,0XF9,0XE0,0XE0,0X00,0X00,0XFF,
		0XFF,0XFF,0XFF,0XFF,0XFF,0X01,0X01,0XF9,0XF9,0XF9,0XF9,0X79,0X79,0X79,0X79,0X79,
		0X79,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
		0XFF,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,
		0XF9,0XF9,0XF9,0XF9,0XF9,0XF9,0XE0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
		0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,
		0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFE,0XFE,0XFE,0XFE,0XCE,0XCE,0XCE,
		0XCE,0XCE,0XCE,0XCE,0XCE,0XCE,0XC0,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFF,
		0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0XC0,0XCF,0XCF,0XCF,0XCF,0XCE,0XCE,0XCE,0XCE,0XCE,
		0XCE,0XCE,0XCE,0XCF,0XCF,0XCF,0XCF,0XCF,0XC7,0X00,0X00,0XCF,0XCF,0XCF,0XCF,0XCF,
		0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,0XCF,
		0XCF,0XCF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
		0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,
		0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
		0X00,0X00,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X01,0X01,0X01,0X01,0X01,0X01,0X01,
		0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X00,0X00,0X00,
		0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,
		0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X01,0X01,0X01,
		0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,
		0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X00,0X00,0X01,0X01,0X01,0X01,0X01,0X01,
		0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00,
		0X00,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};

#endif // __PICTURES_H__
