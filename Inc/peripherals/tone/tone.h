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

#define C2 	131	// DO
#define C2H 139	// DO#
#define D2 	147	// RE
#define D2H 156	// RE#
#define E2 	165	// MI
#define F2 	175	// FA
#define F2H 185	// FA#
#define G2 	196	// SOL
#define G2H 208	// SOL#
#define A2 	220	// LA
#define A2H 233	// LA#
#define B2 	247	// SI

#define C3 	262	// DO
#define C3H 278	// DO#
#define D3 	294	// RE
#define D3H 312	// RE#
#define E3 	330	// MI
#define F3 	350	// FA
#define F3H 370	// FA#
#define G3 	393	// SOL
#define G3H 416	// SOL#
#define A3 	440	// LA
#define A3H 467	// LA#
#define B3 	494	// SI
#define C4  524 // DO

// ------------------------------------------------------------------------------------------------------------------------
// tonesplayer
// note[size]     : tableau de fréquence (notation américaine A = LA, B= SI, C= DO,... A3 = LA 440Hz)
// duration[size) : tableau de durée (nombre de double croche) 1=double croche, 2= croche, 4=noire, 8=blanche, 16=ronde...)
// size           : nombre de note dans le morceau
// tempo		  : 60= 60 noire à la minute
// ------------------------------------------------------------------------------------------------------------------------
void tonesplayer(int *note, int *duration, int size, int tempo);

void toneInit(void);
void toneItMode(int note, int duration);
void tone_IT(void);
void toneStart(int note);
void toneStop(void);
void tone(int note, int duration);
void toneSetVolulme(int volume);
void toneTest(void);

#endif

// Au clair de la lune
//  int music[11]={C3,C3,C3,D3,E3,D3,C3,E3,D3,D3,C3};
//  int duree[11]={2 ,2, 2, 2, 4, 4, 2, 2, 2, 2, 8};

// Marseillaise
//  int music[18]={G2,G2,G2,C3,C3,D3,D3,G3,E3,C3,C3,E3,C3,A2,F3,D3,B2,D3};
//  int duree[18]={1, 2, 1, 4, 4, 4, 4, 7, 1, 3, 1, 3, 1, 4, 8, 3, 1, 8}

// Joyeux anniversaire
//  int music[25]={G2,G2,A2,G2,C3,B2,G2,G2,A2,G2,D3,C3,G2,G2,G3,E3,C3,B2,A2,F3,F3,E3,C3,D3,C3};
//  int duree[25]={2, 2, 4, 4, 4, 8, 2, 2, 4 ,4, 4, 8, 2, 2, 4, 4, 4, 4, 8, 2, 2, 4, 4, 4, 8};
