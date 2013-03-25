
#include <p18f2455.h>
#include <i2c.h>
#include <delays.h>
#include <stdio.h>           //needed for printf, may remove to save space

#ifndef COMMON_H
#define	COMMON_H
#define TONE_LOW_BYTE 0x0f

//#define A_TONE_l 0x3F
//#define Ash_TONE_l 0x40
//
//#define B_TONE_l 0x4F
//#define C_TONE_l 0x50
//#define Csh_TONE_l 0x5F

#define D_TONE_l 0x00
#define Dsh_TONE_l 0x0F

#define E_TONE_l 0x10

#define F_TONE_l 0x1F
#define Fsh_TONE_l 0x20

#define G_TONE_l 0x2F
#define Gsh_TONE_l 0x30



#define A_TONE_h 0x3F
#define Ash_TONE_h 0x40

#define B_TONE_h 0x4F
#define C_TONE_h 0x50
#define Csh_TONE_h 0x5F

#define D_TONE_h 0x60
#define Dsh_TONE_h 0x6F

#define E_TONE_h 0x70

#define F_TONE_h 0x7F
#define Fsh_TONE_h 0x80

#define G_TONE_h 0x8F
#define Gsh_TONE_h 0x90

struct song_desc {
    volatile unsigned char note_length;
    volatile unsigned char song_index;
    volatile unsigned char *song;       //point this to song array
};


void led_setup(void);

void interrupt_setup(void);

void i2c_setup(void);

void setup(void);


#endif	/* COMMON_H */

