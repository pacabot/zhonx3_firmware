/**************************************************************************/
/*!
    @file     tone.h
    @author  BM (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __TONE_H__
#define __TONE_H__

/* Module Identifier */
#define TONE_DRIVER_MODULE_ID  14

/* Error codes */
#define TONE_DRIVER_E_SUCCESS  0
#define TONE_DRIVER_E_ERROR    MAKE_ERROR(TONE_DRIVER_MODULE_ID, 1)

#define c	261
#define d   294
#define e   329
#define f   349
#define g   391
#define gS  415
#define a   440
#define aS  455
#define b   466
#define cH  523
#define cSH 554
#define dH  587
#define dSH 622
#define eH  659
#define fH  698
#define fSH 740
#define gH  784
#define gSH 830
#define aH  880

#define C2 	131
#define C2d 139
#define D2 	147
#define D2d 156
#define E2 	165
#define F2 	175
#define F2d 185
#define G2 	196
#define G2d 208
#define A2 	220
#define A2d 233
#define B2 	247

#define C3 	262
#define C3d 278
#define D3 	294
#define D3d 312
#define E3 	330
#define F3 	350
#define F3d 370
#define G3 	393
#define G3d 416
#define A3 	441
#define A3d 467
#define B3 	494

/*************************************************
 * Public Constants
 *************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

void tonesplayer(int *note, int *duration, int size, int tempo);
void tone(int note, int duration);
void toneSetVolulme(int volume);
void toneTest(void);

#endif

// double   1 la ronde vaut 16
// Au clair de la lune
//  int music[11]={C3,C3,C3,D3,E3,D3,C3,E3,D3,D3,C3};
//  int duree[11]={2,2,2,2,4,4,2,2,2,2,8};


// Marseillaise
//  int music[18]={G2,G2,G2,C3,C3,D3,D3,G3,E3,C3,C3,E3,C3,A2,F3,D3,B2,D3};
//  int duree[18]={1,2,1,4,4,4,4,7,1,3,1,3,1,4,8,3,1,8}

// Joyeux anniversaire
//  int music[25]={G2,G2,A2,G2,C3,B2,G2,G2,A2,G2,D3,C3,G2,G2,G3,E3,C3,B2,A2,F3,F3,E3,C3,D3,C3};
//  int duree[25]={2,2,4,4,4,8,2,2,4,4,4,8,2,2,4,4,4,4,8,2,2,4,4,4,8};
