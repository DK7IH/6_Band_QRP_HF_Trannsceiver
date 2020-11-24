///////////////////////////////////////////////////////////////////
/*               6-Band QRP HF Transceiver "Midi6"               */ 
/*  ************************************************************ */
/*  Mikrocontroller:  ATMEL AVR ATmega128, 16 MHz                */
/*  LCD: CP11003  												 */
/*  DDS used: AD9951, AD9834                                     */  
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Autor:            Peter Rachow (DK7IH)                       */
/*  Letzte Aenderung: 2020-01-11                                 */
///////////////////////////////////////////////////////////////////
//8 bit parallel LCD Version

//MIC connector from front (Mini DIN)
//          [   ]
//
//     (1)        (2)
//
//        (3)  (4)
//
// 1=PTT
// 2=MIC
// 3,4=GND

// UART 3 pole connector
//             ----------- 
// <AAA|BBB|CCC
//             ----------- A TX out|B  RX in  | C GND
//
// MUC: grey: RX, green: TX

//Line orange, white: GND, blue rx, yellow TX

  ///////////////////////
 //       PORTS       //
///////////////////////

  ///////////////
 //O U T P U T//
///////////////

//LCD
//   Data lines
//PC0      violet
//PC1      blue
//PC2      green
//PC3      brown
//PC4      yellow
//PC5      grey
//PC6      white
//PC7      pink / orange
//   Ctrl lines
//PA4       blue    RS
//PA5       brown   WR
//PA6       violet  RD
//PA7       green   RES

//BCD driver for band relays
//PA0  white
//PA1  green
//PA2  grey

//PE3 LCD Backlight PWM
//PA3 TX/RX relay out

//DDS1  AD9951
//      PD4: IO_UD      //green
//      PD5: SDIO       //white
//      PD6: SCLK       //blue
//      PD7: RESETPIN   //violet

//PB3: RX ATT 

//DDS2  AD9834
//      PB4: FQ_UD       //green
//      PB5: SDATA       //white
//      PB6: W_CLK       //blue

//PB7 Tone oscillator

//Tone
//PG3, PG4
//AGC
//PG0, PG1

  /////////////
 //I N P U T//
/////////////

 //PD2  ROT ENC
 //PD3
//ADC
//PF0/ADC0: Keys           yellow
//PF1/ADC1: PWR-Meter      violet 
//PF2/ADC2: PA Temperature brown
//PF3/ADC3: Voltage        green
//PF4/ADC4: S-Meter		   blue

//PG2 PTT in

  ///////////
 // T W I //
///////////
//PD0: SCL yellow
//PD1: SDA green

  ///////////////////////
 //    E E P R O M    //
///////////////////////
// Bytes
// 0:63:    Memory frequencies 160m    Formula: memplace = band * 64 + memory# * 4
// 64:127:  Memory frequencies 80m
// 128:191: Memory frequencies 40m
// 192:255: Memory frequencies 20m
// 256:319: Memory frequencies 15m
// 320:383: Memory frequencies 10m

//VFO frequencies
// 384:387: Freq VFO A 160m memplace=96  Formula: memplace = band * 2 + 96
// 388:391: Freq VFO B 160m memplace=97
// 392:395: Freq VFO A 80m memplace=98
// 396:399: Freq VFO B 80m memplace=99
// 400:403: Freq VFO A 40m memplace=100
// 404:407: Freq VFO B 40m memplace=101
// 408:411: Freq VFO A 20m memplace=102
// 412:415: Freq VFO B 20m memplace=103
// 416:419: Freq VFO A 15m memplace=104
// 420:423: Freq VFO B 15m memplace=105
// 424:427: Freq VFO A 10m memplace=106
// 428:431: Freq VFO B 10m memplace=107


// 432:435: Reserved
// 436:439: Reserved

// 440: Last band used
// 441: Last VFO in use
// 442: Last memory selected
// 450: Scan threshold (0:80)

// 480: TONE set
// 481: AGC set
// 482: Backlight set
// 483: ATT set

// 484, 485: TX preset 160m
// 486, 487: TX preset 80m
// 488, 489: TX preset 40m
// 490, 491: TX preset 20m
// 492, 493: TX preset 15m
// 494, 495: TX preset 10m

//512:515: f.Lo.LSB memplace=128
//516:519: f.Lo.USB memplace=129

//Scan edge frquencies
/*
550	553	Scanfreq0: 160
554	557	Scanfreq1: 160
558	561	Scanfreq0: 80
562	565	Scanfreq1: 80
566	569	Scanfreq0: 40
570	573	Scanfreq1: 40
574	577	Scanfreq0: 20
578	581	Scanfreq1: 20
582	585	Scanfreq0: 15
586	589	Scanfreq1: 15
590	593	Scanfreq0: 10
594	597	Scanfreq1: 10
*/

#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define FOSC 16000000// Clock Speed
#define BAUD 2400
#define UARTBAUDSET FOSC/16/BAUD-1

  ///////////////////
 //  LCD-Display  //
///////////////////
//LCD ILI9341
//Parallel output (8 bit) line defines
#define LCDDATAPORT PORTC
#define LCDDATADDR  DDRC
#define LCDCTRLPORT PORTA
#define LCDCTRLDDR  DDRA
#define LCDRS  16 //CMD (low) or DATA (high)				blue
#define LCDWR  32  //Write operation indicator (actice low)	brown
#define LCDRD  64  //Read indicator (actice low)	        vioelt
#define LCDRES 128 //Reset (active low)						green

//Code defines for basic LCD write procedure
#define LCD_CMD   0
#define LCD_DATA  1

//LCD resolution
#define LCD_WIDTH   320
#define LCD_HEIGHT  240	

//Menu
#define MENUITEMS 11

//Sample colors
#define WHITE        0xFFFF
#define SILVER1      0xC618
#define SILVER2      0xA510
#define BLACK0       0x0000
#define BLACK1       0x0004
#define GRAY         0x8410
#define LIGHT_GRAY   0xC618
#define DARK_GRAY    0x628A
#define LIGHT_GREEN  0x07E0
#define LIGHT_RED    0xF800
#define RED          0xF800
#define LIGHT_BLUE   0x03FF
#define BLUE         0x001F
#define DARK_BLUE1   0x0002
#define DARK_BLUE2   0x0008
#define MAROON1      0x8000
#define MAROON2      0x7800
#define FUCHSIA      0xF81F		
#define PURPLE1      0x8010
#define PURPLE2      0x780F
#define LIME         0x07E0
#define GREEN        0x0400
#define YELLOW       0xFFE0
#define OLIVE1       0x8400
#define OLIVE2       0x7BE0
#define NAVY1        0x0010
#define NAVY2        0x000F
#define AQUA         0x07FF
#define TEAL         0x0410
#define MAGENTA      0xF81F
#define CYAN         0x07FF
#define DARK_CYAN    0x03EF
#define ORANGE       0xFCA0
#define BROWN        0x8200
#define LIGHT_BROWN  0xF5F0
#define VIOLET       0x9199
#define LIGHT_VIOLET 0xF00F
#define PINK         0xF97F
#define GOLD         0xA508

//Font dimensions
#define FONTWIDTH  12 
#define FONTHEIGHT 16

//Font data 12x16 vert. MSB 
//Based on work by Benedikt K. published on
//https://www.mikrocontroller.net/topic/54860 THANKS!
const char xchar[][24] PROGMEM={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x00
{0x00,0x00,0x03,0xF0,0x0C,0x0C,0x10,0x02,0x11,0x32,0x22,0x31,0x22,0x01,0x22,0x31,0x11,0x32,0x10,0x02,0x0C,0x0C,0x03,0xF0},	// 0x01
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1F,0xFE,0x1E,0xCE,0x3D,0xCF,0x3D,0xFF,0x3D,0xCF,0x1E,0xCE,0x1F,0xFE,0x0F,0xFC,0x03,0xF0},	// 0x02
{0x00,0x00,0x00,0x00,0x00,0xF0,0x01,0xF8,0x03,0xF8,0x07,0xF0,0x0F,0xE0,0x07,0xF0,0x03,0xF8,0x01,0xF8,0x00,0xF0,0x00,0x00},	// 0x03
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x0F,0xF8,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00},	// 0x04
{0x00,0x00,0x03,0x80,0x07,0xC0,0x07,0xC0,0x13,0xB8,0x1B,0xFC,0x1F,0xFC,0x1B,0xFC,0x13,0xB8,0x07,0xC0,0x07,0xC0,0x03,0x80},	// 0x05
{0x00,0x00,0x00,0x00,0x03,0x80,0x07,0xC0,0x17,0xE0,0x1B,0xF0,0x1F,0xFC,0x1B,0xF0,0x17,0xE0,0x07,0xC0,0x03,0x80,0x00,0x00},	// 0x06
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x07
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x08
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x0A
{0x00,0x00,0x03,0x80,0x07,0xC0,0x0C,0x60,0x08,0x20,0x08,0x20,0x0C,0x60,0x07,0xC8,0x03,0xA8,0x00,0x18,0x00,0x78,0x00,0x00},	// 0x0B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x08,0xF8,0x09,0x8C,0x3F,0x04,0x3F,0x04,0x09,0x8C,0x08,0xF8,0x00,0x70,0x00,0x00},	// 0x0C
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x0D
{0x00,0x00,0x06,0x00,0x0F,0x00,0x0F,0x00,0x07,0xFF,0x00,0x33,0x30,0x66,0x78,0xCC,0x79,0x98,0x3F,0xF0,0x00,0x00,0x00,0x00},	// 0x0E
{0x00,0x00,0x00,0x80,0x09,0xC8,0x07,0xF0,0x06,0x30,0x0C,0x18,0x3C,0x1E,0x0C,0x18,0x06,0x30,0x07,0xF0,0x09,0xC8,0x00,0x80},	// 0x0F
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xFC,0x0F,0xF8,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x10
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x0F,0xF8,0x1F,0xFC,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x11
{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x10,0x0C,0x18,0x1C,0x1C,0x3F,0xFE,0x1C,0x1C,0x0C,0x18,0x04,0x10,0x00,0x00,0x00,0x00},	// 0x12
{0x00,0x00,0x00,0x00,0x00,0x00,0x37,0xFE,0x37,0xFE,0x00,0x00,0x00,0x00,0x37,0xFE,0x37,0xFE,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x13
{0x00,0x00,0x00,0x38,0x00,0x7C,0x00,0xC6,0x00,0x82,0x3F,0xFE,0x3F,0xFE,0x00,0x02,0x3F,0xFE,0x3F,0xFE,0x00,0x02,0x00,0x00},	// 0x14
{0x00,0x00,0x00,0x00,0x08,0xDC,0x19,0xFE,0x11,0x22,0x11,0x22,0x11,0x22,0x11,0x22,0x1F,0xE6,0x0E,0xC4,0x00,0x00,0x00,0x00},	// 0x15
{0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x00,0x00},	// 0x16
{0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x08,0x4C,0x0C,0x5C,0x0E,0x7F,0xFF,0x5C,0x0E,0x4C,0x0C,0x44,0x08,0x00,0x00,0x00,0x00},	// 0x17
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x18,0x00,0x1C,0x3F,0xFE,0x00,0x1C,0x00,0x18,0x00,0x10,0x00,0x00,0x00,0x00},	// 0x18
{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x0C,0x00,0x1C,0x00,0x3F,0xFE,0x1C,0x00,0x0C,0x00,0x04,0x00,0x00,0x00,0x00,0x00},	// 0x19
{0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00},	// 0x1A
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00},	// 0x1B
{0x00,0x00,0x3F,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x00,0x00},	// 0x1C
{0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x00,0x80,0x00,0x80,0x00,0x80,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80},	// 0x1D
{0x00,0x00,0x04,0x00,0x06,0x00,0x07,0x00,0x07,0x80,0x07,0xC0,0x07,0xE0,0x07,0xC0,0x07,0x80,0x07,0x00,0x06,0x00,0x04,0x00},	// 0x1E
{0x00,0x00,0x00,0x20,0x00,0x60,0x00,0xE0,0x01,0xE0,0x03,0xE0,0x07,0xE0,0x03,0xE0,0x01,0xE0,0x00,0xE0,0x00,0x60,0x00,0x20},	// 0x1F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0x33,0xFF,0x33,0xFF,0x00,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x21
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x22
{0x00,0x00,0x02,0x00,0x1E,0x10,0x1F,0x90,0x03,0xF0,0x02,0x7E,0x1E,0x1E,0x1F,0x90,0x03,0xF0,0x02,0x7E,0x00,0x1E,0x00,0x10},	// 0x23
{0x00,0x00,0x00,0x00,0x04,0x78,0x0C,0xFC,0x0C,0xCC,0x3F,0xFF,0x3F,0xFF,0x0C,0xCC,0x0F,0xCC,0x07,0x88,0x00,0x00,0x00,0x00},	// 0x24
{0x00,0x00,0x30,0x00,0x38,0x38,0x1C,0x38,0x0E,0x38,0x07,0x00,0x03,0x80,0x01,0xC0,0x38,0xE0,0x38,0x70,0x38,0x38,0x00,0x1C},	// 0x25
{0x00,0x00,0x00,0x00,0x1F,0x00,0x3F,0xB8,0x31,0xFC,0x21,0xC6,0x37,0xE2,0x1E,0x3E,0x1C,0x1C,0x36,0x00,0x22,0x00,0x00,0x00},	// 0x26
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x27,0x00,0x3F,0x00,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x27
{0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1F,0xFE,0x38,0x07,0x20,0x01,0x20,0x01,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x28
{0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x01,0x20,0x01,0x38,0x07,0x1F,0xFE,0x0F,0xFC,0x03,0xF0,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x29
{0x00,0x00,0x00,0x00,0x0C,0x98,0x0E,0xB8,0x03,0xE0,0x0F,0xF8,0x0F,0xF8,0x03,0xE0,0x0E,0xB8,0x0C,0x98,0x00,0x00,0x00,0x00},	// 0x2A
{0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x0F,0xF0,0x0F,0xF0,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00,0x00,0x00},	// 0x2B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB8,0x00,0xF8,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2C
{0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00,0x00,0x00},	// 0x2D
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2E
{0x00,0x00,0x18,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E},	// 0x2F
{0x00,0x00,0x07,0xF8,0x1F,0xFE,0x1E,0x06,0x33,0x03,0x31,0x83,0x30,0xC3,0x30,0x63,0x30,0x33,0x18,0x1E,0x1F,0xFE,0x07,0xF8},	// 0x30
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x0C,0x30,0x0C,0x30,0x0E,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00},	// 0x31
{0x00,0x00,0x30,0x1C,0x38,0x1E,0x3C,0x07,0x3E,0x03,0x37,0x03,0x33,0x83,0x31,0xC3,0x30,0xE3,0x30,0x77,0x30,0x3E,0x30,0x1C},	// 0x32
{0x00,0x00,0x0C,0x0C,0x1C,0x0E,0x38,0x07,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xE7,0x1F,0x7E,0x0E,0x3C},	// 0x33
{0x00,0x00,0x03,0xC0,0x03,0xE0,0x03,0x70,0x03,0x38,0x03,0x1C,0x03,0x0E,0x03,0x07,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x03,0x00},	// 0x34
{0x00,0x00,0x0C,0x3F,0x1C,0x7F,0x38,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x38,0xE3,0x1F,0xC3,0x0F,0x83},	// 0x35
{0x00,0x00,0x0F,0xC0,0x1F,0xF0,0x39,0xF8,0x30,0xDC,0x30,0xCE,0x30,0xC7,0x30,0xC3,0x30,0xC3,0x39,0xC3,0x1F,0x80,0x0F,0x00},	// 0x36
{0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x30,0x03,0x3C,0x03,0x0F,0x03,0x03,0xC3,0x00,0xF3,0x00,0x3F,0x00,0x0F,0x00,0x03},	// 0x37
{0x00,0x00,0x0F,0x00,0x1F,0xBC,0x39,0xFE,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00},	// 0x38
{0x00,0x00,0x00,0x3C,0x00,0x7E,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x38,0xC3,0x1C,0xC3,0x0E,0xC3,0x07,0xE7,0x03,0xFE,0x00,0xFC},	// 0x39
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x3A
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9C,0x70,0xFC,0x70,0x7C,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x3B
{0x00,0x00,0x00,0x00,0x00,0xC0,0x01,0xE0,0x03,0xF0,0x07,0x38,0x0E,0x1C,0x1C,0x0E,0x38,0x07,0x30,0x03,0x00,0x00,0x00,0x00},	// 0x3C
{0x00,0x00,0x00,0x00,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x00,0x00},	// 0x3D
{0x00,0x00,0x00,0x00,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0E,0x1C,0x07,0x38,0x03,0xF0,0x01,0xE0,0x00,0xC0,0x00,0x00,0x00,0x00},	// 0x3E
{0x00,0x00,0x00,0x1C,0x00,0x1E,0x00,0x07,0x00,0x03,0x37,0x83,0x37,0xC3,0x00,0xE3,0x00,0x77,0x00,0x3E,0x00,0x1C,0x00,0x00},	// 0x3F
{0x00,0x00,0x0F,0xF8,0x1F,0xFE,0x18,0x07,0x33,0xF3,0x37,0xFB,0x36,0x1B,0x37,0xFB,0x37,0xFB,0x36,0x07,0x03,0xFE,0x01,0xF8},	// 0x40
{0x00,0x00,0x38,0x00,0x3F,0x00,0x07,0xE0,0x06,0xFC,0x06,0x1F,0x06,0x1F,0x06,0xFC,0x07,0xE0,0x3F,0x00,0x38,0x00,0x00,0x00},	// 0x41
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00,0x00,0x00},	// 0x42
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0C,0x0C,0x00,0x00},	// 0x43
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00},	// 0x44
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0x03,0x30,0x03,0x00,0x00},	// 0x45
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x03,0x00,0x00},	// 0x46
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x3F,0xC7,0x3F,0xC6,0x00,0x00},	// 0x47
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x48
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x49
{0x00,0x00,0x0E,0x00,0x1E,0x00,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00},	// 0x4A
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x01,0xE0,0x03,0xF0,0x07,0x38,0x0E,0x1C,0x1C,0x0E,0x38,0x07,0x30,0x03,0x00,0x00},	// 0x4B
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00},	// 0x4C
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x1E,0x00,0x78,0x01,0xE0,0x01,0xE0,0x00,0x78,0x00,0x1E,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x4D
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x0E,0x00,0x38,0x00,0xF0,0x03,0xC0,0x07,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x4E
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00},	// 0x4F
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0xC7,0x00,0xFE,0x00,0x7C,0x00,0x00},	// 0x50
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x36,0x03,0x3E,0x07,0x1C,0x0E,0x3F,0xFC,0x33,0xF0,0x00,0x00},	// 0x51
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x03,0x83,0x07,0x83,0x0F,0x83,0x1D,0xC7,0x38,0xFE,0x30,0x7C,0x00,0x00},	// 0x52
{0x00,0x00,0x0C,0x3C,0x1C,0x7E,0x38,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xC7,0x1F,0x8E,0x0F,0x0C,0x00,0x00},	// 0x53
{0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00},	// 0x54
{0x00,0x00,0x07,0xFF,0x1F,0xFF,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00},	// 0x55
{0x00,0x00,0x00,0x07,0x00,0x3F,0x01,0xF8,0x0F,0xC0,0x3E,0x00,0x3E,0x00,0x0F,0xC0,0x01,0xF8,0x00,0x3F,0x00,0x07,0x00,0x00},	// 0x56
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x1C,0x00,0x06,0x00,0x03,0x80,0x03,0x80,0x06,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x57
{0x00,0x00,0x30,0x03,0x3C,0x0F,0x0E,0x1C,0x03,0x30,0x01,0xE0,0x01,0xE0,0x03,0x30,0x0E,0x1C,0x3C,0x0F,0x30,0x03,0x00,0x00},	// 0x58
{0x00,0x00,0x00,0x03,0x00,0x0F,0x00,0x3C,0x00,0xF0,0x3F,0xC0,0x3F,0xC0,0x00,0xF0,0x00,0x3C,0x00,0x0F,0x00,0x03,0x00,0x00},	// 0x59
{0x00,0x00,0x30,0x03,0x3C,0x03,0x3E,0x03,0x33,0x03,0x31,0xC3,0x30,0xE3,0x30,0x33,0x30,0x1F,0x30,0x0F,0x30,0x03,0x00,0x00},	// 0x5A
{0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x5B
{0x00,0x00,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x18,0x00},	// 0x5C
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x5D
{0x00,0x00,0x00,0x60,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0x60},	// 0x5E
{0x00,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00},	// 0x5F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x7E,0x00,0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x60
{0x00,0x00,0x1C,0x00,0x3E,0x40,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0xE0,0x3F,0xC0,0x00,0x00},	// 0x61
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00},	// 0x62
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x18,0xC0,0x08,0x80,0x00,0x00},	// 0x63
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0xE0,0x30,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x64
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x3B,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x13,0xC0,0x01,0x80,0x00,0x00},	// 0x65
{0x00,0x00,0x00,0xC0,0x00,0xC0,0x3F,0xFC,0x3F,0xFE,0x00,0xC7,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x66
{0x00,0x00,0x03,0x80,0xC7,0xC0,0xCE,0xE0,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xE6,0x60,0x7F,0xE0,0x3F,0xE0,0x00,0x00},	// 0x67
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,0x00,0x00},	// 0x68
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x60,0x3F,0xEC,0x3F,0xEC,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x69
{0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0xC0,0x60,0xFF,0xEC,0x7F,0xEC,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x6A
{0x00,0x00,0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x38,0x60,0x30,0x00,0x00,0x00,0x00,0x00},	// 0x6B
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x6C
{0x00,0x00,0x3F,0xE0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00},	// 0x6D
{0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00},	// 0x6E
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00},	// 0x6F
{0x00,0x00,0xFF,0xE0,0xFF,0xE0,0x0C,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x1C,0xE0,0x0F,0xC0,0x07,0x80,0x00,0x00},	// 0x70
{0x00,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x0C,0x60,0xFF,0xE0,0xFF,0xE0,0x00,0x00},	// 0x71
{0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0x00},	// 0x72
{0x00,0x00,0x11,0xC0,0x33,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0x60,0x1E,0x40,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x73
{0x00,0x00,0x00,0x60,0x00,0x60,0x1F,0xFE,0x3F,0xFE,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x74
{0x00,0x00,0x0F,0xE0,0x1F,0xE0,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x18,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x00},	// 0x75
{0x00,0x00,0x00,0x60,0x01,0xE0,0x07,0x80,0x1E,0x00,0x38,0x00,0x38,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00},	// 0x76
{0x00,0x00,0x07,0xE0,0x1F,0xE0,0x38,0x00,0x1C,0x00,0x0F,0xE0,0x0F,0xE0,0x1C,0x00,0x38,0x00,0x1F,0xE0,0x07,0xE0,0x00,0x00},	// 0x77
{0x00,0x00,0x30,0x60,0x38,0xE0,0x1D,0xC0,0x0F,0x80,0x07,0x00,0x0F,0x80,0x1D,0xC0,0x38,0xE0,0x30,0x60,0x00,0x00,0x00,0x00},	// 0x78
{0x00,0x00,0x00,0x00,0x00,0x60,0x81,0xE0,0xE7,0x80,0x7E,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00,0x00,0x00},	// 0x79
{0x00,0x00,0x30,0x60,0x38,0x60,0x3C,0x60,0x36,0x60,0x33,0x60,0x31,0xE0,0x30,0xE0,0x30,0x60,0x30,0x20,0x00,0x00,0x00,0x00},	// 0x7A
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x1F,0xFC,0x3F,0x7E,0x70,0x07,0x60,0x03,0x60,0x03,0x60,0x03,0x00,0x00,0x00,0x00},	// 0x7B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xBF,0x3F,0xBF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x7C
{0x00,0x00,0x00,0x00,0x60,0x03,0x60,0x03,0x60,0x03,0x70,0x07,0x3F,0x7E,0x1F,0xFC,0x01,0xC0,0x00,0x80,0x00,0x00,0x00,0x00},	// 0x7D
{0x00,0x00,0x00,0x10,0x00,0x18,0x00,0x0C,0x00,0x04,0x00,0x0C,0x00,0x18,0x00,0x10,0x00,0x18,0x00,0x0C,0x00,0x04,0x00,0x00},	// 0x7E
{0x00,0x00,0x0F,0x00,0x0F,0x80,0x0C,0xC0,0x0C,0x60,0x0C,0x30,0x0C,0x30,0x0C,0x60,0x0C,0xC0,0x0F,0x80,0x0F,0x00,0x00,0x00},	// 0x7F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0xFC,0x00,0xCC,0x00,0xCC,0x00,0xFC,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x80 °-sign
};
//LCD additional data
int bcolor = BLACK0; //Standard Background Color
  
  ///////////
 //  ADC  //
////////////
#define ADC_KEY_PORT PORTF

  ///////////
 //  DDS  //
////////////
// SPI DDS1 (AD9951)
#define DDS1_PORT PORTD
#define DDS1_DDR DDRD
#define DDS1_IO_UD 16      //green
#define DDS1_SDIO 32       //white
#define DDS1_SCLK 64       //blue
#define DDS1_RESETPIN 128  //violet

///////////////////////
//  SPI DDS2  AD9834 //
///////////////////////
#define DDS2_PORT PORTB
#define DDS2_DDR DDRB
#define DDS2_FSYNC 16 //green
#define DDS2_SDATA 32 //white
#define DDS2_SCLK 64  //blue

  /////////////////////
 //   BAND RELAYS   //
/////////////////////
#define RELAY_PORT PORTA
#define RELAY_0 PA0 //grey
#define RELAY_1 PA1 //green
#define RELAY_2 PA2 //white

//S-Meter
#define SMAX 240
#define SMETERPOSITION 62

int main(void);

  /////////////////
 //   L  C  D   //
/////////////////

//ILI9341 LCD Basic Functions
void lcd_send(int, int);                           //Write a byte of data via SPI
void lcd_set_xy(int, int);
void lcd_cls(int);
void lcd_draw_pixel(int);
void lcd_putchar(int, int, int, int, int, int);
void lcd_putstring(int, int, char*, int, int, int);
void lcd_putnumber(int, int, unsigned long, int, int, int, int);
void lcd_init(void);
void lcd_setbacklight(int);

//Extended LCD functions
void show_all_data(unsigned long, unsigned long, int, int, int, int, int, unsigned long, int, int, int, int);
void show_frequency1(unsigned long, int, int);
void show_frequency2(int, int, unsigned long, int, int, int);
void show_mem_number(int);
void show_mem_freq(unsigned long, int);
void show_sideband(int, int);
void show_voltage(int);
void show_band(int);
void show_temp(int);
void show_vfo(int, int);
void show_split(int, int);
void show_scan_status(int, int);
void show_tone(int, int);
void show_txrx(int);
void show_agc(int, int);
void show_att(int, int);
void draw_meter_scale(int, int);
void reset_smax(void);
void smeter(int, int);
void clear_smeter(int);
void show_msg(char*, int);//STRING FUNCTIONS
int int2asc(unsigned long, int, char*, int);
//int strlen(char *s);
int calcx(int col);
int calcy(int row);

//Frequency generation
void dds1_send_bit(int);
void dds1_send_word(unsigned int);
void dds1_send_byte(unsigned int);
void set_frequency1(unsigned long frequency);

void dds1_send_bit(int);
void dds2_start(void);
void dds2_stop(void);
void dds2_send_bit(int sbit);
void set_frequency2(unsigned long fx);

void set_lo_freq(int);
//ADC
int get_s_value(void);
int get_keys(void);
int get_adc(int);
int get_ptt(void);

//MISC
int calc_tuningfactor(void);
void tx_test(void);
void tune(void);
void set_dualtone_oscillator(int);

//Graphics
void drawbox(int, int, int, int, int);
void draw_hor_line(int, int, int, int);
void draw_vert_line(int, int, int, int);

//Menu
int menu0_get_xp(int);
int menu0_get_yp(int);
long menu0(long, int, int);
unsigned long menu1(int, unsigned long, int, int);
void print_menu_head(char*, int);
void print_menu_item(int, int, int);
void print_menu_item_list(int, int);
int navigate_thru_item_list(int, int, int, int, int);
void print_menu_help(int, int, int, int);

//Scanning & VFO
unsigned long scan(int);
void set_scan_threshold(void);
unsigned long set_scan_frequency(int, unsigned long);

//Tone and AGC
void set_tone(int);
void set_agc(int);
void set_att(int);
void set_band(int);

//BAcklight
int adjustbacklight(void);

//TWI - I²C
void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t);

//DAC and TX preset
void mcp4725_set_value(int);
void tx_preset_adjust(void);
void store_tx_preset(int, int);
int load_tx_preset(int);

//EEPROM
//Loading
int load_last_band(void);
unsigned long load_frequency0(int);
unsigned long load_frequency1(int);
int load_last_mem(void);
int load_last_vfo(void);
unsigned long recall_mem_freq(int);
//Storing
void store_last_band(int);
void store_frequency0(unsigned long, int);
void store_frequency1(unsigned long, int);
void store_last_vfo(int);
void store_last_mem(int);
int save_mem_freq(unsigned long, int);

void rcv_mem_frequencies(void);

//Checking
int is_mem_freq_ok(unsigned long,int);

//UART
void usart_init(int);
int usart_receive(void);
void usart_transmit(unsigned char data);

  /////////////
 //Variables//
/////////////
//Seconds counting
unsigned long runseconds10 = 0; 

//S-Meter temporary max. value
int smaxold = 0;

//Freq data
//LO
#define MAXMODES 2

//Radio basics
int txrx = 0;

//Interfrequency options
#define IFOPTION 0

#if (IFOPTION == 0) //9MHz Filter 9XMF24D (box73.de)
    #define INTERFREQUENCY 9000000
    #define F_LO_LSB 8998130
    #define F_LO_USB 9001420
#endif  
  
#if (IFOPTION == 1)  //10.695MHz Filter 10M04DS (ex CB TRX "President Jackson")
    #define INTERFREQUENCY 10695000 //fLSB 10692100, fUSB 10697700
    #define F_LO_LSB 10691880
    #define F_LO_USB 10697580
#endif  

#if (IFOPTION == 2) //10.7MHz Filter 10MXF24D (box73.de)
    #define INTERFREQUENCY 10700000 
    #define F_LO_LSB 10697630
    #define F_LO_USB 10702000
#endif  

#if (IFOPTION == 3) //Ladderfilter 9.830 MHz low profile xtals
    #define INTERFREQUENCY 9830000
    #define F_LO_LSB 9828320
    #define F_LO_USB 9831000
#endif  

#if (IFOPTION == 4) //Ladderfilter 9.832 MHz high profile xtals "NARVA"
    #define INTERFREQUENCY 9830000
    #define F_LO_LSB 9830960
    #define F_LO_USB 9834930
#endif  

#if (IFOPTION == 5)     //Ladderfilter 10 MHz high profile xtals
    #define INTERFREQUENCY 10000000
    #define F_LO_LSB 9994720
    #define F_LO_USB 9999840
#endif  

unsigned long f_lo[] = {F_LO_LSB, F_LO_USB}; //LSB, USB

int sideband = 0;  //Current sideband in use LSB=0, USB=1
int cur_band;

//VFO
#define MAXVFOS 10
unsigned long f_vfo[2];
int vfo_s[2];
int split = 0; //0=off, 1=TXA RXB, 2=TXB RXA

//Data for 6 bands
int std_sideband [] = {0, 0, 0, 1, 1, 1};                                    //Standard sideband for each rf band
unsigned long c_freq[] =  {1950000, 3650000, 7120000, 14180000, 21290000, 28500000};  //Center frequency
unsigned long band_f0[] = {1810000, 3500000, 7000000, 14000000, 21000000, 28000000};  //Edge frequency I
unsigned long band_f1[] = {2000000, 3800000, 7200000, 14350000, 21450000, 29700000};  //Edge frequency II

//Frequency memories
#define MAXMEM 15
int last_memplace = 0;

//Scanning
int s_threshold = 30;
unsigned long scanfreq[2];

//Encoder & tuning
int laststate = 0; //Last state of rotary encoder
int tuningknob = 0;
int tuningcount = 0;

//Tone and AGC
int curtone;
int curagc;
int curatt;

//TX amplifier preset values
int tx_preset[6] = {0, 0, 0, 0, 0, 0};

//METER
int smax = 0;
unsigned long runseconds10s = 0;

//Menu n=items-1
int menu_items[MENUITEMS] =  {5, 1, 3, 1, 3, 3, 1, 3, 2, 1, 4}; 

//Backlight
int blight = 128;

  /////////////////////////////////////
 //  Functions for ILI9341 control  //
/////////////////////////////////////
//Parallel write data or command to LCD
void lcd_send(int dc, int val)
{		
	if(!dc) //Cmd (0) or Data(1)?
	{
	    LCDCTRLPORT &= ~(LCDRS);  //Cmd=0
	}
	else
	{
	    LCDCTRLPORT |= LCDRS;     //Data=1
	}
	LCDCTRLPORT |= LCDRD;
	
	LCDDATAPORT = val;
	LCDCTRLPORT &= ~(LCDWR); //Write operation
	LCDCTRLPORT |= LCDWR;
}	

//Init LCD to vertical alignement and 16-bit color mode
void lcd_init(void)
{
	lcd_send(LCD_CMD,  0xCF);
	lcd_send(LCD_DATA, 0x00);
	lcd_send(LCD_DATA, 0xc1);
	lcd_send(LCD_DATA, 0x30);
	
	lcd_send(LCD_CMD,  0xed); 
    lcd_send(LCD_DATA, 0x64);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0x12);
    lcd_send(LCD_DATA, 0x81);

	lcd_send(LCD_CMD,  0xcb); 
    lcd_send(LCD_DATA, 0x39);
    lcd_send(LCD_DATA, 0x2c);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x34);
    lcd_send(LCD_DATA, 0x02);

    lcd_send(LCD_CMD,  0xea); 
	lcd_send(LCD_DATA, 0x00);
	lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0xe8); 
	lcd_send(LCD_DATA, 0x85);
	lcd_send(LCD_DATA, 0x10);
	lcd_send(LCD_DATA, 0x79);
	
    lcd_send(LCD_CMD,  0xC0); // Power control
    lcd_send(LCD_DATA, 0x23); // VRH[5:0]

    lcd_send(LCD_CMD,  0xC1); // Power control
    lcd_send(LCD_DATA, 0x10); // SAP[2:0];BT[3:0]

    lcd_send(LCD_CMD,  0xC5); // VCM control
    lcd_send(LCD_DATA, 0x3e);
    lcd_send(LCD_DATA, 0x28);

    lcd_send(LCD_CMD,  0xC7); // VCM control2
    lcd_send(LCD_DATA, 0x86);

    lcd_send(LCD_CMD,  0x36); // Memory Access Control
    lcd_send(LCD_DATA, 0x88); // C8

    lcd_send(LCD_CMD,  0x3A);
    lcd_send(LCD_DATA, 0x55);

    lcd_send(LCD_CMD,  0xB1);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x18);

    lcd_send(LCD_CMD,  0xB6); // Display Function Control
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x82);
    lcd_send(LCD_DATA, 0x27);

    lcd_send(LCD_CMD,  0xF2); // 3Gamma Function Disable
    lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0x26); // Gamma curve selected
    lcd_send(LCD_DATA, 0x01);

    lcd_send(LCD_CMD,  0xE0); // Set Gamma
    lcd_send(LCD_DATA, 0x0F);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0x2B);
    lcd_send(LCD_DATA, 0x0C);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x4E);
    lcd_send(LCD_DATA, 0xF1);
    lcd_send(LCD_DATA, 0x37);
    lcd_send(LCD_DATA, 0x07);
    lcd_send(LCD_DATA, 0x10);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x09);
    lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0xE1); // Set Gamma
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x14);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0x11);
    lcd_send(LCD_DATA, 0x07);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0xC1);
    lcd_send(LCD_DATA, 0x48);
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x0F);
    lcd_send(LCD_DATA, 0x0C);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0x36);
    lcd_send(LCD_DATA, 0x0F);

    lcd_send(LCD_CMD,  0x11); // Sleep out
    _delay_ms(120);
    lcd_send(LCD_CMD, 0x2c);  
        
    lcd_send(LCD_CMD, 0x29); // Display on 
    lcd_send(LCD_CMD, 0x2c);	
	
}	

void lcd_set_xy(int x, int y)
{
	//X
	lcd_send(LCD_CMD, 0x2B);
    lcd_send(LCD_DATA, x >> 8);
    lcd_send(LCD_DATA, x & 0xFF);
    lcd_send(LCD_CMD, 0x2c);

    //Y 
    lcd_send(LCD_CMD, 0x2A);
    lcd_send(LCD_DATA, y >> 8);
    lcd_send(LCD_DATA, y & 0xFF);
    lcd_send(LCD_CMD, 0x2c);
}
 
void lcd_draw_pixel(int color)
{
    lcd_send(LCD_DATA, color >> 8);
    lcd_send(LCD_DATA, color & 0xFF);
}

void lcd_cls(int bcolor)
{
	int x;
	unsigned char y;
	
	lcd_set_xy(0, 0);
	for(x = 0; x < LCD_WIDTH; x++)
	{
        for(y = 0; y < LCD_HEIGHT; y++)
        {
			lcd_draw_pixel(bcolor);
		}	
	}
}		

//Write character from font set to destination on screen
void lcd_putchar(int x, int y, int c, int size, int fcolor, int bcolor)
{
    int x0;
    int t0, t1, t2, t3, u;
       
    x0 = x;
    for(t0 = 0; t0 < FONTWIDTH * 2; t0 += 2)
    { 
		for(t1 = 0; t1 < size; t1++)
		{
		    //u = xchar[c][t0 + 1] + (xchar[c][t0] << 8);
		    u = pgm_read_byte(&xchar[c][t0 + 1]) + (pgm_read_byte(&xchar[c][t0]) << 8);
		    lcd_set_xy(x0, y);
		    for(t2 = 16; t2 >= 0; t2--)
		    {
			    if(u & (1 << t2))
			    {
				    for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(fcolor);
		            }
		        }    
		        else
		        {
		            for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(bcolor);
		            }
		        }
		    }
		    x0++;
		}    
	}	
}	

//Print String to LCD
void lcd_putstring(int x, int y, char *text, int size, int fc, int bc)
{
	int t1 = 0, x0;
	
	x0 = x;
	while(text[t1])
	{
		lcd_putchar(x0, y, text[t1], size, fc, bc);
		x0 += (size * FONTWIDTH);
		t1++;
	}
}		

//Convert a number to a string and print it
//col, row: Coordinates, Num: int or unsigned long to be displayed
//dec: Set position of decimal separator
//
//inv: Set to 1 if inverted charactor is required
void lcd_putnumber(int x, int y, unsigned long num, int dec, int lsize, int fc, int bc)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(x, y, s, lsize, fc, bc);
	    free(s);
	}	
	else
	{
		lcd_putstring(x, y, "Error", lsize, fc, bc);
	}	
}

//Set backlight
void lcd_setbacklight(int duty_cycle)
{
	OCR3A = duty_cycle;
}	

  ///////////////////////////////////////
  // Data Display Functions for Radio  //
 ///////////////////////////////////////
void show_all_data(unsigned long f0, unsigned long f1, int refresh, int s, int scan_s, int vfo, int splt, unsigned long splt_freq, int splt_invert, int mtr_scale, int memplace, int tr)
{
	int linecolor = GRAY;
			
	show_frequency1(f0, refresh, bcolor); 
	show_frequency2(8, 9, f1, bcolor, 100, 1); 
	show_sideband(s, 0);
	//show_scan_status(scan_s);
	show_mem_number(memplace);
	show_vfo(vfo, 0);
	
	show_voltage(bcolor);
	show_temp(bcolor);
	show_split(splt, bcolor);
	//show_split_freq(splt_freq, splt_invert);
	
	draw_meter_scale(mtr_scale, bcolor);
	show_tone(curtone, bcolor);
	show_agc(curagc, bcolor);
	show_band(cur_band);
    show_txrx(tr);
    show_att(curatt, bcolor);	
	load_tx_preset(cur_band);
	
	draw_hor_line(0, LCD_WIDTH, 20, linecolor);
	draw_hor_line(0, LCD_WIDTH, 75, linecolor);
	draw_hor_line(0, LCD_WIDTH, 150, linecolor);
	draw_hor_line(0, LCD_WIDTH, 200, linecolor);
	draw_vert_line(65, 150, LCD_HEIGHT, linecolor);
	draw_vert_line(140, 150, LCD_HEIGHT, linecolor);
	draw_vert_line(240, 150, LCD_HEIGHT, linecolor);
	draw_vert_line(260, 20, 75, linecolor);
}    
 
//Calc x and y position of character
int calcx(int col)
{
	return col * FONTWIDTH;
}

int calcy(int row)
{
	return (14 - row) * FONTHEIGHT;
}

//Current frequency (double letter height)
void show_frequency1(unsigned long f, int refresh, int bc)
{
	char *buf;
	int t1, t2, x;
	int x0 = 7, y0 = 8; 
	int fc = 0;
	
	if(is_mem_freq_ok(f, cur_band))
	{
		fc = YELLOW;
	}
	else	
	{
		fc = LIGHT_RED;
	}	
	
	if(f < 10000000)
	{
		x0 = 8;
	}	
	
	if(f <= 0)
	{
		for(x = 80; x < LCD_WIDTH; x++)
		{
			lcd_set_xy(x, calcy(y0));
			for(t2 = 0; t2 < FONTHEIGHT * 2 + 2; t2++)
			{
				lcd_draw_pixel(bc);
			}	
		}
		//lcd_putstring(calcx(x0), calcy(y0), "#####.#",  2, YELLOW, bcolor);
		return;
	}
		
	buf = malloc(10);
	
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
	
	if(f)
	{
	    int2asc(f / 100, 1, buf, 9);
	}
	else
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			buf[t1] = 32;
		}
	}		
	
	//Display buffer (but only the letters that have changed)
	for(t1 = 0; *(buf + t1); t1++)
	{
		lcd_putchar(calcx(x0 + t1 * 2), calcy(y0), *(buf + t1), 2, fc, bc);  
	}	
	
	lcd_putstring(calcx(22), calcy(8), "kHz", 1, fc, bc);
		
	free(buf);
}

//Memeory frequency on selection memplace in menu
void show_frequency2(int x, int y, unsigned long f, int bc, int x10, int digits)
{
	int fcolor = WHITE;
		
	if(!f)
	{
		lcd_putstring(calcx(x), calcy(y) - 1, "-------", 1, fcolor, bc);
	}
	else
	{	
        lcd_putstring(calcx(x), calcy(y) - 1, "       ", 1, fcolor, bc);
	    lcd_putnumber(calcx(x), calcy(y) - 1, f / x10, digits, 1, fcolor, bc);
	}    
}	

void show_band(int b)
{
	int xpos = 0, ypos = 8;
	char *bnd[] = {"160m", "80m ", "40m ", "20m ", "15m ", "10m"};
	int fc[] = {LIGHT_GREEN, LIGHT_BLUE, LIGHT_BROWN, YELLOW, LIGHT_GRAY, LIGHT_VIOLET};
	lcd_putstring(calcx(xpos), calcy(ypos)- 5, bnd[b], 1, fc[b], bcolor);	
}	

//VFO
void show_vfo(int nvfo, int bcolor)
{
	int xpos = 0, ypos = 1;
	
	lcd_putstring(calcx(xpos), calcy(ypos), "VFO", 1, LIGHT_GRAY, bcolor);			
	lcd_putchar(calcx(xpos + 3), calcy(ypos), nvfo + 65, 1, LIGHT_GRAY, bcolor);  
	
}

void show_split(int sp_status, int bcolor)
{
	int xpos = 0, ypos = 7;
	char *splitstr = "SPLIT";
		
	//Write string to position
	if(sp_status)
	{
	    lcd_putstring(calcx(xpos), calcy(ypos), splitstr, 1, WHITE, bcolor);
	}    
	else
	{
	    lcd_putstring(calcx(xpos), calcy(ypos), splitstr, 1, DARK_GRAY, bcolor);
	}    
}


//Scan operation
void show_scan_status(int status, int bc)
{
	int xpos = 16, ypos = 1;
	
	switch(status)
	{
		case 0: lcd_putstring(calcx(xpos), calcy(ypos), "SCAN", 1, WHITE, bcolor);
		        break;
		case 1: lcd_putstring(calcx(xpos), calcy(ypos), "VFO ", 1, WHITE, bcolor);
		        break;
		case 2: lcd_putstring(calcx(xpos), calcy(ypos), "VFO*", 1, WHITE, bcolor);
		        break;        
		case 3: lcd_putstring(calcx(xpos), calcy(ypos), "BND ", 1, WHITE, bcolor);
		        break;
		case 4: lcd_putstring(calcx(xpos), calcy(ypos), "BND*", 1, WHITE, bcolor);
		        break;                
	}
}	

//Current sideband display
void show_sideband(int sb, int bc)
{
	int xpos = 21, ypos = 1;
	char *sidebandstr[] = {"LSB", "USB"};
		
	//Write string to position
	lcd_putstring(calcx(xpos), calcy(ypos), sidebandstr[sb], 1, YELLOW, bc);
}


//Battery voltage display
void show_voltage(int bc)
{
    char *buf;
	int t1;
	int xpos = 21, ypos = 3;	
	int fc = LIGHT_BLUE;
		
	int adc_v;
    double v1;		
    	 
	//Measure current voltage	
	v1 = (double) get_adc(3) * 5 / 1024 * 5 * 10;
	adc_v = (int) v1;
   	
   	if(v1 < 110)
   	{
		fc = RED;
	}
	
	if(v1 > 150)
   	{
		fc = ORANGE;
	}
	
	buf = malloc(10);
	
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
    
    //Display value
    lcd_putstring(calcx(xpos), calcy(ypos), "     ", 1, 0, bc);
    int2asc(adc_v, 1, buf, 6);
    lcd_putstring(calcx(xpos), calcy(ypos), buf, 1, fc, bc);
	lcd_putchar(calcx(xpos + strlen(buf)), calcy(ypos), 'V', 1, fc, bc);
	
	//Free mem
	free(buf);
}

//Temperature display
void show_temp(int bc)
{
    char *buf;
	int t1, fc;
	int xpos = 21, ypos = 4;	
		
	int adc_t;
    double r1, temp1;		
    	
	//Measure current voltage	
	r1 = 2000.0 / (1024.0 / get_adc(2) - 1);
	temp1 = (double) (10.0 * (r1 - 1630) / 17.62); 
	
    adc_t = (int) temp1;
   	
	lcd_putstring(calcx(xpos), calcy(ypos), "     ", 1, WHITE, bc);
	buf = malloc(10);
	
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
    
    if(adc_t < 300)
    {
		fc = LIGHT_BLUE;
	}	
	if(adc_t >= 300 && adc_t < 600)
    {
		fc = LIGHT_GREEN;
	}
	
	if(adc_t >= 600)
    {
		fc = LIGHT_RED;
	}	
	
    //Display value
    //int2asc(get_adc(2), -1, buf, 6); //TEST
    int2asc(adc_t, 1, buf, 6);
    lcd_putstring(calcx(xpos), calcy(ypos) - 1, buf, 1, fc, bc);
	lcd_putchar(calcx(xpos + strlen(buf)), calcy(ypos) - 1, 0x80, 1, fc, bc);
	
	//Free mem
	free(buf);
}

//AGC speed
void show_agc(int agc, int bc)
{
	int xpos = 6, ypos = 3;
	lcd_putstring(calcx(xpos), calcy(ypos), "AGC  ", 1, GRAY, DARK_BLUE2);				
	char *agcstr[] = {"FAST ", "NORM ", "SLOW ", "XSLOW"}; 
	lcd_putstring(calcx(xpos), calcy(ypos + 1) - 1, agcstr[agc], 1, GREEN, DARK_BLUE2);			
}

//Tone pitch
void show_tone(int tone, int bc)
{
	int xpos = 0, ypos = 3;
	lcd_putstring(calcx(xpos), calcy(ypos), "TONE ", 1, GRAY, DARK_BLUE2);				
	char *tstr[] = {"HIGH", "NORM", "LOW ", "XLOW"};
	lcd_putstring(calcx(xpos), calcy(ypos + 1) - 1, tstr[tone], 1, RED, DARK_BLUE2);				
}

//Tone pitch
void show_att(int att, int bc)
{
	int xpos = 12, ypos = 3;
	lcd_putstring(calcx(xpos), calcy(ypos), "ATT ", 1, GRAY, DARK_BLUE2);				
	char *attstr[] = {"OFF ", "ON  "}; 
	lcd_putstring(calcx(xpos), calcy(ypos + 1) - 1, attstr[att], 1, GREEN, DARK_BLUE2);			
}



void show_txrx(int tx)
{
	int x = 22, y = 12;
	if(tx)
	{
		lcd_putstring(calcx(x), calcy(y - 1), " TX ", 1, LIGHT_RED, WHITE);
		lcd_putstring(calcx(x), calcy(y), " RX ", 1, DARK_GRAY, BLACK0);
	}
	else
	{
		lcd_putstring(calcx(x), calcy(y - 1), " TX ", 1, DARK_GRAY, BLACK0);
		lcd_putstring(calcx(x), calcy(y), " RX ", 1, LIGHT_GREEN, DARK_GRAY);
	}
		
}

//Draw TX or RX meter scale
void draw_meter_scale(int scaletype, int bc)
{
	int x = 0, y = 12;
	
	if(!scaletype)
	{
		lcd_putstring(calcx(x), calcy(y), "S1 3 5 7 9 +10 +20dB", 1, WHITE, bcolor);
	}
	else
	{
		lcd_putstring(calcx(x), calcy(y), "P 1 2  4  6 8 10 20W", 1, WHITE, bcolor);
	}
}		

//S-Meter bargraph
void smeter(int value, int bc)
{
	int v, t1, t2;
	int x = 0, y = SMETERPOSITION; //Position on screen
	int fc = WHITE;
	v = value;
	
	if(v > SMAX)
	{
		v = SMAX;
	}	
	
	//Draw bar
	for(t1 = 0; t1 < v; t1 += 4)
	{
		lcd_set_xy(x + t1, y);
		if(t1 > 60)
		{
			fc = YELLOW;
		}		
		
		if(t1 > 120)
		{
			fc = ORANGE;
		}	
				
		if(t1 > 160)
		{
			fc = LIGHT_RED;
		}
		
		for(t2 = 0; t2 < 8; t2++)
		{
			lcd_draw_pixel(fc);
		}	
	}		
		
	//Clear meter approx. to max. value
	for(t1 = (v / 4) * 4; t1 < smaxold - 6; t1 += 4)
	{ 
	    lcd_set_xy(x + t1, y);
	    
		for(t2 = 0; t2 < 8; t2++)
		{
		    lcd_draw_pixel(bc);
		}	
	}    
			
	if(v > smaxold)
	{
		smaxold = v;
	}	
	
}

void clear_smeter(int bc)
{
	int t1, t2;
	int x = 0, y = SMETERPOSITION; //Position omn screen
	
	//Clear meter approx. to max. value
	for(t1 = 0; t1 < SMAX; t1 += 4)
	{ 
	    lcd_set_xy(x + t1, y);
		for(t2 = 0; t2 < 8; t2++)
		{
		    lcd_draw_pixel(bc);
		}	
	}   
}
	
//Reset max value of s meter
void reset_smax(void)
{
	int t1, t2, s;
	int x = 0, y = SMETERPOSITION; //Position omn screen
	
	s = get_s_value();
			
	//Clear meter approx. to max. value
	for(t1 = SMAX; t1 > s; t1--)
	{ 
	    lcd_set_xy(x + t1, y);
		for(t2 = 0; t2 < 8; t2++)
		{
		    lcd_draw_pixel(bcolor);
		}	
	}   
	
	smax = 0;	
	smaxold = 0;
}	

void show_msg(char *msg, int bc)
{
	int x = 0, y = 14, t1, t2;
	
	if(!strlen(msg))
	{
		for(t1 = 0; t1 < LCD_WIDTH; t1++)
		{
			lcd_set_xy(t1, calcy(y));
			for(t2 = 0; t2 < FONTHEIGHT; t2++)
			{
				lcd_draw_pixel(bc);
			}	
		}	
		return;
	}
		
	lcd_putstring(calcx(x), calcy(y), msg, 1, LIGHT_GRAY, bc);
}	

//Show memory place by number
void show_mem_number(int mem_addr)
{
	int xpos = 6, ypos = 1;
    	
	if(mem_addr == -1)
	{
		lcd_putstring(calcx(xpos + FONTWIDTH * 2), calcy(ypos), "--", 1, LIGHT_GRAY, bcolor);
		return;
	}	
	
	//Show number of mem place
	lcd_putstring(calcx(xpos), calcy(ypos), "M", 1, WHITE, bcolor);
		
	if(mem_addr < 10)
	{
	     lcd_putnumber(calcx(xpos + 1), calcy(ypos), 0, -1, 1, LIGHT_GREEN, bcolor);
	     lcd_putnumber(calcx(xpos + 2), calcy(ypos), mem_addr, -1, 1,LIGHT_GREEN, bcolor);								
	}
	else
	{
		lcd_putnumber(calcx(xpos + 1), calcy(ypos), mem_addr, -1, 1, LIGHT_GREEN, bcolor);				
	}
}

void show_mem_freq(unsigned long f, int bc)
{
	int xpos = 12, ypos = 1;
	int fcolor = LIGHT_GREEN;
           
	if(f)
	{
	    lcd_putnumber(calcx(xpos), calcy(ypos), f / 100, 1, 1, fcolor, bc);			
	} 
	else  
    {
	    lcd_putstring(calcx(xpos), calcy(ypos), " ----- ", 1, fcolor, bc);			
	} 
}	

  //////////////////////
 // STRING FUNCTIONS //
//////////////////////
//INT 2 ASC
int int2asc(unsigned long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    unsigned long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//STRLEN
/*
int strlen(char *s)
{
   int t1 = 0;

   while(*(s + t1++));

   return (t1 - 1);
}
*/
  /////////////////
 //   GRAPHICS  //
/////////////////
void draw_hor_line(int x0, int x1, int y, int fc)
{
	int t1;

	//Hor. lines
	for(t1 = x0; t1 < x1; t1++)
	{
		lcd_set_xy(t1, y);
		lcd_draw_pixel(fc);
		lcd_draw_pixel(fc);
		lcd_set_xy(t1, y);
		lcd_draw_pixel(fc);
		lcd_draw_pixel(fc);
	}
}	

void draw_vert_line(int x0, int y0, int y1, int fc)
{
	int t1;

	//Vert. lines
	lcd_set_xy(x0, y0);
	for(t1 = y0; t1 < y1; t1++)
	{
		lcd_draw_pixel(fc);
	}
}	


void drawbox(int x0, int y0, int x1, int y1, int fc)
{
	int t1;
		
	//Hor. lines
	for(t1 = x0; t1 < x1; t1++)
	{
		lcd_set_xy(t1, LCD_HEIGHT - y0);
		lcd_draw_pixel(fc);
		lcd_draw_pixel(fc);
		lcd_set_xy(t1, LCD_HEIGHT - y1);
		lcd_draw_pixel(fc);
		lcd_draw_pixel(fc);
	}
	
	//Vert. lines
	lcd_set_xy(x0, LCD_HEIGHT - y1);
	for(t1 = y0; t1 < y1; t1++)
	{
		lcd_draw_pixel(fc);
	}
	
	lcd_set_xy(x0 + 1, LCD_HEIGHT - y1);
	for(t1 = y0; t1 < y1; t1++)
	{
		lcd_draw_pixel(fc);
	}
	
	lcd_set_xy(x1, LCD_HEIGHT - y1);
	for(t1 = y0; t1 < y1; t1++)
	{
		lcd_draw_pixel(fc);
	}
	
	lcd_set_xy(x1 + 1, LCD_HEIGHT - y1);
	for(t1 = y0; t1 < y1; t1++)
	{
		lcd_draw_pixel(fc);
	}
	
}

  /////////////////
 //   T  W  I   //
/////////////////
void twi_init(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
    //enable TWI
    TWCR = (1<<TWEN);
}

  /////////////////
 //   MCP4725   //
/////////////////
//Send comand to MCP4725
void mcp4725_set_value(int value)
{
   twi_start();
   twi_write(0xC0); //Device address
   twi_write(64);       		    	
   twi_write(value >> 4); //8 MSBs
   twi_write((value & 0x0F) << 4); //4LSBs
   twi_stop();			
		
} 

void tx_preset_adjust(void)
{
	int key = 0, t1;
	int v1 = tx_preset[cur_band];
	char *tmpstr;
	
	tmpstr = malloc(12);
	for(t1 = 0; t1 < 10; t1++)
	{
		tmpstr[t1] = 0;
	}	
	
	PORTA |= 8; //TX on
  
	show_msg("      TX Preset", bcolor);
	int2asc(v1, -1, tmpstr, 8);
    show_msg(tmpstr, bcolor);
    
	while(get_keys());
	key = get_keys();
			
	while(!key)
	{
		if(tuningknob >= 1)  //Turn CW
		{
		    if(v1 < 4090)
		    {
				v1 += 5;
			}
						
		    tuningknob = 0;
		    int2asc(v1, -1, tmpstr, 8);
		    show_msg("    ", bcolor);
		    show_msg(tmpstr, bcolor);
		    mcp4725_set_value(v1);
		} 

		if(tuningknob <= -1) //Turn CCW
		{    
		    if(v1 > 5)
		    {
				v1 -= 5;
			}
			
		    tuningknob = 0;
		    int2asc(v1, -1, tmpstr, 8);
		    show_msg("    ", bcolor);
		    show_msg(tmpstr, bcolor);
			mcp4725_set_value(v1);
		}	
		key = get_keys();
	}	
	
	PORTA &= ~(8); 		//TX off
	
	if(key == 2)
	{
		tx_preset[cur_band] = v1;
		store_tx_preset(v1, cur_band);
	}	
	
	free(tmpstr);
}	

void store_tx_preset(int value, int band)
{
	//MSB first
    int adr = 484 + band * 2;
    
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)adr++, (value >> 8) & 0x0f);
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)adr, value & 0xff);
    show_msg("TX preset stored.", bcolor);
}	

int load_tx_preset(int band)
{
	int x = 20, y = 14;
	char *buffer;
	
	//MSB first
	 int adr = 484 + band * 2;
    int v = 0;
    
    while(!eeprom_is_ready());
    v = eeprom_read_byte((uint8_t*)adr++) << 8;
    while(!eeprom_is_ready());
    v += eeprom_read_byte((uint8_t*)adr);
    show_msg("TX preset loaded:", bcolor);
    buffer = malloc(16);
    int2asc(v, -1, buffer, 15);
    lcd_putstring(calcx(x), calcy(y), buffer, 1, YELLOW, bcolor);
    free(buffer);
    return v;
}	

void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

  ///////////////////
 //   A   D   C   //
///////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	
	int adc_val = 0;
	
	ADMUX = (1<<REFS0) + adc_channel;     // Kanal adcmode aktivieren
    _delay_ms(3);
	
    ADCSRA |= (1<<ADSC);
    //while(ADCSRA & (1<<ADSC));
	_delay_ms(3);
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	return adc_val;
	
}	

//Read keys via ADC0
int get_keys(void)
{

    int key_value[] = {39, 143, 280};
        	
    int t1;
    int adcval = get_adc(0);
        
    //TEST display of ADC value 
    //lcd_putstring(0, 5, "....", 1, DARK_BLUE2, WHITE);    
    //lcd_putnumber(0, 5, adcval, -1, 1, DARK_BLUE2, WHITE);    
    	
    for(t1 = 0; t1 < 3; t1++)
    {
        if(adcval > key_value[t1] - 5 && adcval < key_value[t1] + 5)
        {
             return t1 + 1;
        }
    }
    return 0;
}

//Check PTT Pin PG2
int get_ptt(void)
{
	if(!(PING & (1 << PG2)))
	{ 
		return 1;
	}	
	else
	{
		return 0;
	}	
	
}		

//Determine s-value
//No sig=3V, Full sig=0V
int get_s_value(void)
{
	int s = 620 - get_adc(4);
	
	return ((s >> 2) + (s >> 3));  
}	
  ////////////////////////
 //  HARDWARE SETTINGS //
////////////////////////
void set_tone(int tone_value)
{
	//Reset PG3 and PG4
	PORTG &= ~(8);   //PG3 Lo
	PORTG &= ~(16);  //PG4 Lo
	
	PORTG |= tone_value << 3; // !!!
	
	cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)480, tone_value);
    sei();
}

void set_agc(int agc_value)
{
	//Reset PG0 and PG1
	PORTG &= ~(1);  //PG0 Lo
	PORTG &= ~(2);  //PG1 Lo
	
	PORTG |= agc_value;
	
	cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)481, agc_value);
    sei();
}	

void set_att(int att_value)
{
	//Reset PG3 and PG4
	
	if(att_value)
	{
	    PORTB |= (1 << PB3);
	}
	else
	{
	    PORTB &= ~(1 << PB3);
	}
	    
	cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)483, att_value);
    sei();
}


//Set relays for spec. band
void set_band(int b)
{
	PORTA &= ~(0x01);  //Reset PA0:PA2
	PORTA &= ~(0x02);  
	PORTA &= ~(0x04);  
	PORTA |= b + 1;
}

void set_dualtone_oscillator(int state)
{
	if(state)
	{
		PORTB |= (1 << 7);
	}
	else
	{
		PORTB &= ~(1 << 7);
	}		
}	

  /////////////////////////////
 //       DDS Functions     //
/////////////////////////////

////////////////////////
//    SPI for DDS 1   //
////////////////////////
void dds1_send_bit(int sbit)
{
    DDS1_PORT &= ~(DDS1_SCLK);  //SCLK lo
    	
    //Bit setzen oder löschen
	if(sbit)
	{
		DDS1_PORT|= DDS1_SDIO;  //SDATA  set
	}
	else
	{
		DDS1_PORT &= ~(DDS1_SDIO);  //SDATA  erase
	}
	
    DDS1_PORT |= DDS1_SCLK; //SCLK hi
	
}

void dds1_send_byte(unsigned int sbyte)
{
    int t1, x = (1 << 7);
	
	for(t1 = 0; t1 < 8; t1++)
	{
	    dds1_send_bit(sbyte & x);	
		x >>= 1;
	}	
}

void dds1_send_word(unsigned int sword)
{
    int t1, x = (1 << 15);
	
	for(t1 = 0; t1 < 16; t1++)
	{
	    dds1_send_bit(sword & x);	
		x >>= 1;
	}	
}

//SET frequency AD9951 DDS
//f.clock = 400MHz
void set_frequency1(unsigned long frequency)
{
    unsigned long f;
    unsigned long fword;
    int t1, t2, shiftbyte = 24, resultbyte, x;
    unsigned long comparebyte = 0xFF000000;
	
	f = frequency + 3000; //Offset because of inaccuracy of crystal oscillator
		 
	if(!sideband)//Calculate correct offset from center frequency in display for each sideband
	{
	     fword = (unsigned long) (f + INTERFREQUENCY - 5000) * 10.73741824; //USB
	}    
	else
    {
	     fword = (unsigned long) (f + INTERFREQUENCY) * 10.73741824; //LSB
	}    
	
    //Start transfer to DDS
    DDS1_PORT &= ~(DDS1_IO_UD); //DDS1_IO_UD lo
    
	//Send instruction bit to set fequency by frequency tuning word
	x = (1 << 7);
	for(t1 = 0; t1 < 8; t1++)
	{
	    dds1_send_bit(0x04 & x);	
		x >>= 1;
	}	
	
    //Calculate and transfer the 4 bytes of the tuning word to DDS
    //Start with msb
    for(t1 = 0; t1 < 4; t1++)
    {
        resultbyte = (fword & comparebyte) >> shiftbyte;
        comparebyte >>= 8;
        shiftbyte -= 8;       
        x = (1 << 7);
        for(t2 = 0; t2 < 8; t2++)
	    {
	        dds1_send_bit(resultbyte & x);	
		    x >>= 1;
	    }	
    }    
	
	//End transfer sequence
    DDS1_PORT|= (DDS1_IO_UD); //DDS1_IO_UD hi 
}

  ////////////////////////
 //  SPI DDS 2 AD9834  //
////////////////////////
void dds2_start(void)
{
	DDS2_PORT |= DDS2_SCLK;      //SCLK hi
    DDS2_PORT &= ~(DDS2_FSYNC);  //FSYNC lo
}

void dds2_stop(void)
{
	DDS2_PORT |= DDS2_FSYNC; //FSYNC hi
}

void dds2_send_bit(int sbit)
{
    if(sbit)
	{
		DDS2_PORT |= DDS2_SDATA;  //SDATA hi
	}
	else
	{
		DDS2_PORT &= ~(DDS2_SDATA);  //SDATA lo
	}
	
	DDS2_PORT |= DDS2_SCLK;     //SCLK hi
    DDS2_PORT &= ~(DDS2_SCLK);  //SCLK lo
}

void set_frequency2(unsigned long f)
{

    double fword0;
    long fword1, x;
    int l[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int m[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, t1;
    
    int fclk = 100;
    double fact;
    
    //fact = 268435456 / fClk
    switch(fclk)
    {
		  
		case 50:  fact = 5.36870912; //50MHz
		          break;        
		case 75:  fact = 3.579139413; //75MHz
		          break;         
		case 100: fact = 2.68435456; //100MHz
		          break;                   
		case 110: fact = 2.440302361; //110MHz
		          break;         
		default:  fact = 3.579139413;        
	}	          
		         
    fword0 = (double) (fact  * f); 
        
    fword1 = (long) fword0;

    //Transfer frequency word to byte array
    x = (1 << 13);      //2^13
    for(t1 = 2; t1 < 16; t1++)
    {
		if(fword1 & x)
	    {
			l[t1] = 1;
	    }
	    x >>= 1;
    }
    
    x = (1L << 27);  //2^27
    for(t1 = 2; t1 < 16; t1++)
    {
	    if(fword1 & x)
	    {
	        m[t1] = 1;
	    }
	    x >>= 1;
    }
    ////////////////////////////////////////
    
    //Transfer to DDS
    //Send start command
    dds2_start();
    for(t1 = 15; t1 >= 0; t1--)
    {
       dds2_send_bit(0x2000 & (1 << t1));
    }
    dds2_stop();
        
    //Transfer frequency word	
    //L-WORD
    dds2_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       dds2_send_bit(l[t1]);
    }
    dds2_stop();
	
	//M-WORD
	dds2_start();
    for(t1 = 0; t1 < 16; t1++) 
    {
       dds2_send_bit(m[t1]);
    }
    dds2_stop();
}


void set_lo_freq(int sb)
{
	int key;
	unsigned long f = f_lo[sb];
	int fcolor = WHITE;
	
	lcd_cls(bcolor);
		
	lcd_putstring(calcx(2), calcy(1), "Set LO FREQ ", 1, fcolor, bcolor);
	if(!sb)
	{
		lcd_putstring(calcx(2), calcy(3), "LSB", 1, fcolor, bcolor);
	}
	else	
	{
		lcd_putstring(calcx(2), calcy(3), "USB", 1, fcolor, bcolor);
	}
	
	print_menu_help(2, 8, LIGHT_BLUE, bcolor);
		
	key = get_keys();
	show_frequency2(8, 3, f, bcolor, 1, 3);
	
	while(key == 0)
	{
		if(tuningknob >= 1) //Turn CW
		{
			f += 10;
		    tuningknob = 0;
	        show_frequency2(8, 3, f, bcolor, 1, 3);
	        set_frequency2(f);
		}

		if(tuningknob <= -1)  //Turn CCW
		{    
		    f -= 10;
		    tuningknob = 0;
		    show_frequency2(8, 3, f, bcolor, 1, 3);
		    set_frequency2(f);
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		f_lo[sb] = f; //Confirm
		
		store_frequency1(f, 128 + sb); //Save frequency to memplace 128 (LSB) or 129 (USB)
	}	
	else
	{
		set_frequency2(f_lo[sb]); //Abort and restore old data
	}	
}	


  /////////////////////////
 //   E  E  P  R  O  M  //
/////////////////////////
//Store MEM Frequency
void store_frequency0(unsigned long f, int mem)
{
    int start_adr = cur_band * 64 + mem * 4;
    
    store_frequency1(f, start_adr);
	
}

//Store any other frequency by start byte address
void store_frequency1(unsigned long f, int start_adr)
{
    unsigned long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
	cli();
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, hmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 1, hlsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 2, lmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 3, llsb);
    
    sei();	
	
}

//Load a frequency from memory by memplace
unsigned long load_frequency0(int mem)
{
    int start_adr = cur_band * 64 + mem * 4;
		
    return load_frequency1(start_adr);
    //rf = (unsigned long) (hmsb << 24) + (unsigned long) (hlsb << 16) + (unsigned long) (lmsb << 8) + llsb;
}

//Load any other frequency by start byte address
unsigned long load_frequency1(int start_adr)
{
    unsigned char hmsb, lmsb, hlsb, llsb;
    		
    cli();
    hmsb = eeprom_read_byte((uint8_t*)start_adr);
    hlsb = eeprom_read_byte((uint8_t*)start_adr + 1);
    lmsb = eeprom_read_byte((uint8_t*)start_adr + 2);
    llsb = eeprom_read_byte((uint8_t*)start_adr + 3);
	sei();
	
    return (unsigned long) 16777216 * hmsb + (unsigned long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
    //rf = (unsigned long) (hmsb << 24) + (unsigned long) (hlsb << 16) + (unsigned long) (lmsb << 8) + llsb;
}

//Load las memory used
int load_last_mem(void)
{
	return eeprom_read_byte((uint8_t*)442);
}

//Check if freq is in 20m-band
int is_mem_freq_ok(unsigned long f, int cband)
{
	
	if(f >= band_f0[cband] && f <= band_f1[cband])
	{
	    return 1;
	}	
	else
	{
	    return 0;
	}		
}	

//Load 6 by 16 mem frequencies via uasrt0
void rcv_mem_frequencies(void)
{
	int c = 0, r, t1;
	int key;
	char *sbuf;
	char dbuf[384];
	int maxbytes = 384;
	
	sbuf = malloc(32);
	
	while(get_keys());
	
	//Empty RX buffer
	r = usart_receive();
	while(r > -1)
	{
		r = usart_receive();
	}
	show_msg("Ready.", bcolor);
		
	while(c < maxbytes && !key)
	{
		r = usart_receive();
		if(r > -1)
		{
			dbuf[c++] = r;
		}
		//key = get_keys();
	}
	
	int2asc(c, -1, sbuf, 8);
	strcat(sbuf, " Bytes received.");
	show_msg(sbuf, bcolor);
	free(sbuf);
	_delay_ms(1000);
		
	//Store data in EEPROM	
	for(t1 = 0; t1 < maxbytes; t1++)
	{
	    eeprom_write_byte((uint8_t*)t1, dbuf[t1]);
	}    
	show_msg("Data stored.", bcolor);
}			
	

//Store last BAND used
void store_last_band(int bandnum)
{
    
	cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)440, bandnum);
    sei();	
}

//Store last VFO used
void store_last_vfo(int vfonum)
{
    
	cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)441, vfonum);
    sei();	
}

//Load last VFO stored
int load_last_band(void)
{
    int bandnum;
    
    cli();
    bandnum = eeprom_read_byte((uint8_t*)440);
    sei();
	
	if(bandnum >= 0 && bandnum <= 7)
	{
		return bandnum;
	}
	else
	{
		return -1;
	}	
}

//Load last VFO stored
int load_last_vfo(void)
{
    int vfonum;
    
    cli();
    vfonum = eeprom_read_byte((uint8_t*)441);
    sei();
	
	if(vfonum >= 0 && vfonum <= 15)
	{
		return vfonum;
	}
	else
	{
		return -1;
	}	
}

//Store last used memplace
void store_last_mem(int mem)
{
	while(!eeprom_is_ready());
	eeprom_write_byte((uint8_t*)127, mem);
}

//Recall AND display a frequency from memory
//and let user select
//Returns freq in bits 0:27 and mem_number in bits 28:31
unsigned long recall_mem_freq(int mem_addr)
{
	int key;
	int fcolor = WHITE;
	
	lcd_cls(bcolor);
	lcd_putstring(calcx(0), calcy(3), "RECALL",  1, fcolor, bcolor);
	
	//Load initial freq
	if(is_mem_freq_ok(load_frequency0(mem_addr), cur_band))
	{
		show_mem_number(mem_addr);
	    set_frequency1(load_frequency0(mem_addr));
	    show_frequency1(load_frequency0(mem_addr), 1, bcolor);
	}  
	else  
	{
		show_mem_number(mem_addr);
		show_frequency1(0, 1, bcolor);
	}	
	
	key = 0;
	while(!key)
	{
		if(tuningknob <= -1)  
		{    
		    if(mem_addr > 0)
		    {
			    mem_addr--;
			}
			else     
			{
			    mem_addr = MAXMEM;
			}	 
			
			tuningknob = 0;
			
			show_mem_number(mem_addr);
			if(is_mem_freq_ok(load_frequency0(mem_addr), cur_band))
			{
			    set_frequency1(load_frequency0(mem_addr));
			    show_frequency1(load_frequency0(mem_addr), 1, bcolor);
			}    
			else
			{
				show_frequency1(0, 0, bcolor);
			}	
	    }
		
		if(tuningknob >= 1)
		{    
		    if(mem_addr < MAXMEM)
		    {
			    mem_addr++;
			}
			else     
			{
			    mem_addr = 0;
			}	 
			
			tuningknob = 0;
			
			show_mem_number(mem_addr);
			if(is_mem_freq_ok(load_frequency0(mem_addr), cur_band))
			{
			    set_frequency1(load_frequency0(mem_addr));
			    show_frequency1(load_frequency0(mem_addr), 1, bcolor); 
            }    
            else
			{
				show_frequency1(0, 0, bcolor);
			}	
	    }
	    
	    key = get_keys();
	}	
	            
	switch(key)
	{
	    case 2: if(is_mem_freq_ok(load_frequency0(mem_addr), cur_band))
	            {
	                store_last_mem(mem_addr);
	                return(load_frequency0(mem_addr) + ((unsigned long)(mem_addr & 0x0F) << 28));
	            }    
				break;
	}	
								
    while(get_keys());
    
    return 0;
}	

//Save a frequenca to a user selected memplace
int save_mem_freq(unsigned long f, int mem)
{
    int mem_addr = mem;
    unsigned long mem_freq;
	int key;
	int fcolor = WHITE;
	
	lcd_cls(bcolor);
	lcd_putstring(0, 0, "STORE", 1, fcolor, bcolor);
	
	//Load initial mem
	show_mem_number(mem_addr);
	if(is_mem_freq_ok(load_frequency0(mem_addr), cur_band))
	{
	    set_frequency1(load_frequency0(mem_addr));
	}    
	show_frequency1(f, 1, bcolor);
			
	key = 0;
	while(!key)
	{
		if(tuningknob <= -1)
		{    
		    if(mem_addr > 0)
		    {
			    mem_addr--;
			}
			else     
			{
			    mem_addr = MAXMEM;
			}	 
			
			tuningknob = 0;
			
			show_mem_number(mem_addr);
			mem_freq = load_frequency0(mem_addr);
			if(is_mem_freq_ok(mem_freq, cur_band))
			{
				show_mem_freq(mem_freq, bcolor);
			    set_frequency1(mem_freq);
			    set_frequency2(f_lo[sideband]);
			}    
			else
			{
				show_mem_freq(0, bcolor);
			}	
	    }
		
		if(tuningknob >= 1)  
		{    
		    if(mem_addr < MAXMEM)
		    {
			    mem_addr++;
			}
			else     
			{
			    mem_addr = 0;
			}	 
			
			tuningknob = 0;
			
			show_mem_number(mem_addr);
			mem_freq = load_frequency0(mem_addr);
			if(is_mem_freq_ok(mem_freq, cur_band))
			{
				show_mem_freq(mem_freq, bcolor);
			    set_frequency1(mem_freq);
			    set_frequency2(f_lo[sideband]);
			}    
			else
			{
				show_mem_freq(0, bcolor);
			}
	    }
	    
	    key = get_keys();
	}	
	            
	switch(key)
	{
	    case 2:     store_last_mem(mem_addr);
	                store_frequency0(f, mem_addr);
	                return mem_addr;
	            	break;
	}	
								
    while(get_keys());
    
    return -1;
}	

  //////////
 // MENU //
//////////
void print_menu_head(char *head_str0, int m_items)
{	
    int xpos0 = 1;
	int ypos0 = 1;
	int fcolor = WHITE;
		
	lcd_cls(bcolor);
	
	drawbox(132, 24, 238, FONTHEIGHT * m_items + 56, WHITE);
	
	lcd_putstring(calcx(xpos0), calcy(ypos0 + 1), head_str0,  1, fcolor, bcolor);
		
	fcolor = LIGHT_GRAY;
	print_menu_help(xpos0, ypos0 + 8, fcolor, bcolor);
	
}

void print_menu_help(int xpos, int ypos, int fc, int bc)
{
    lcd_putstring(calcx(xpos), calcy(ypos), "(K1) Next", 1, fc, bc);
	lcd_putstring(calcx(xpos), calcy(ypos + 1), "(K2) OK", 1, fc, bc);
	lcd_putstring(calcx(xpos), calcy(ypos + 2), "(K3) Quit Menu", 1,  fc, bc);
}

void print_menu_item(int m, int i, int invert)
{
	char *menu_str[MENUITEMS][10] =    {{"160m   ", "80m    ", "40m    ", "20m    ", "15m    ", "10m    "},
		                                {"LSB    ", "USB    ", "       ", "       ", "       ", "       "},
		                                {"VFO A  ", "VFO B  ", "A=B    ", "B=A    ", "       ", "       "},
		                                {"OFF    ", "ON     ", "       ", "       ", "       ", "       "}, 
		                                {"HIGH   ", "NORM   ", "LOW    ", "XLOW   ", "       ", "       "}, 
		                                {"FAST   ", "NORM   ", "SLOW   ", "XSLOW  ", "       ", "       "}, 
	                                    {"STORE  ", "RECALL ", "       ", "       ", "       ", "       "},
	                                    {"MEMORY ", "BAND   ", "LIMITS ", "THRESH ", "       ", "       "}, 
	                                    {"TXA RXB", "TXB RXA", "OFF    ", "       ", "       ", "       "},
	                                    {"LO LSB ", "LO USB ", "       ", "       ", "       ", "       "},
	                                    {"B-LIGHT", "TX TEST", "TX TUNE", "TX PRES", "GET MEM", "       "}};
	int xpos1 = 12;
	int fc, bc;
	
	if(invert)
	{
		fc = DARK_BLUE2;
		bc = WHITE;
	}
	else	
	{
		fc = WHITE;
		bc = DARK_BLUE2;
	}
	
	lcd_putstring(calcx(xpos1), calcy(i + 2), menu_str[m][i], 1, fc, bc);
}
	
//Print the itemlist or single item
void print_menu_item_list(int m, int item)
{
	
    int t1;
    
    //Print item list for menu
	for(t1 = 0; t1 < menu_items[m] + 1; t1++)
	{
		if(item == t1)
		{
			print_menu_item(m, t1, 1);    
		}
		else
		{	
	        print_menu_item(m, t1, 0);       
	    }	
	}	
}

//Returns menu_pos if OK or -1 if aborted
int navigate_thru_item_list(int m, int maxitems, int curitem, int cvfo, int cband)
{
	int menu_pos = curitem;
	
	//print_menu_item_list(m, menu_pos, 1);     //Write current entry in REVERSE color
	
	int key = get_keys();
	
    while(key == 0)
	{
		if(tuningknob >= 1)  //Turn CW
		{
			print_menu_item(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos < maxitems)
		    {
				menu_pos++;
			}
			else	
			{
				menu_pos = 0;
			}
			print_menu_item(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}

		if(tuningknob <= -1) //Turn CCW
		{    
		    print_menu_item(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else	
			{
				menu_pos = maxitems;
			}
			print_menu_item(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}	
		
		//Make settings audible
		switch(m)
		{
			case 0: break; //Band set: No preview available
			case 1: set_frequency2(f_lo[menu_pos]);
			        break; 
			case 2: if(is_mem_freq_ok(f_vfo[menu_pos], cband))
			        {
				        set_frequency1(f_vfo[menu_pos]);
				        lcd_putnumber(calcx(2), calcy(7), f_vfo[menu_pos] / 100, 1, 1, LIGHT_BLUE, bcolor);
				        //void lcd_putnumber(int x, int y, unsigned long num, int dec, int lsize, int fc, int bc)
				    }    
			        break; //VFO set: No preview available
			case 3: set_att(menu_pos); //ATT     
			        break;
			case 4: set_tone(menu_pos); //Tone       
			        break;
			case 5: set_agc(menu_pos); //AGC
			        break;
			default: break;                
		}	         
				
		key = get_keys();
	}
		
	while(get_keys());
	set_frequency1(f_vfo[cvfo]);
	switch(key)
	{   case 1: return -1;       //Next menu!
		        break;
	    case 2: return menu_pos; //OK
	            break;
	    case 3: return -3;       //Quit menu!
	            break;
	}
	
	return -1;
}	

//Calculate coordinates
int menu0_get_xp(int x)
{
	return calcx(x * 12 + 2);
}	

int menu0_get_yp(int y)
{
	return calcy(y + 4);
}	

//Preselection menu
long menu0(long f, int c_vfo, int c_band)
{
	int x, y, c = 0;
	int key = 0;
	
	char menu_str[MENUITEMS][10] = {"BAND    ", "SIDEBAND", "VFO     ", "ATT     ", "TONE    ", "AGC     ", "MEMORIES", "SCAN    ", "SPLIT   ", "LO ADJST", "SPECIAL "};
	
	while(get_keys());
	
	lcd_cls(bcolor);
	
	lcd_putstring(calcx(0), calcy(1), "       MENU SELECT       ", 1, DARK_BLUE1, LIGHT_GRAY);
	
	//Draw init screen
	for(y = 0; y < 6; y++)
	{
		for(x = 0; x < 2; x++)
		{
			if(c < MENUITEMS)
			{
			    lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, WHITE, DARK_BLUE2);
			    c++;
			}    
		}
	}
	
	c = 0;
	y = c / 2;
	x = c - (y * 2);
	
	drawbox(10, 46, 275, 176, WHITE);
	
	lcd_putstring(calcx(1), calcy(12), "(K2) OK" "(K3) Quit Menu", 1,  LIGHT_GRAY, bcolor);
	
	//Highlight 1st item
	lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, DARK_BLUE2, WHITE);
		
	//Select item
	while(!key)
	{
		
		if(tuningknob >= 1)  
		{
			if(c < MENUITEMS - 1)
			{   
				y = c / 2;
			    x = c - (y * 2);
			    lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, WHITE, DARK_BLUE2);
				
				c++; 
		        
		        y = c / 2;
			    x = c - (y * 2);
			    lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, DARK_BLUE2, WHITE);
			}   
			tuningknob = 0;  
		}	
		
		if(tuningknob <= -1)  
		{   
			if(c > 0)
			{    
				y = c / 2;
			    x = c - (y * 2);
			    lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, WHITE, DARK_BLUE2);
			    
			    c--;
		        
		        y = c / 2;
			    x = c - (y * 2);
			    lcd_putstring(menu0_get_xp(x), menu0_get_yp(y), menu_str[c], 1, DARK_BLUE2, WHITE);
			}    
			tuningknob = 0; 
		}	
		
		key = get_keys();
		
		switch(key)
		{
			case 0: break;
			case 2: return menu1(c, f, c_vfo, c_band);    
			        break;
			default:return -2;
		}	        
	}
	
	while(get_keys());		
	
	return -2; 
}	
	

			
unsigned long menu1(int menu, unsigned long f, int c_vfo, int c_band)
{
	//                              0       1       2      3       4       5      6      7        8        9        10
	char menu_str[MENUITEMS][10] = {"BAND", "SIDE", "VFO", "ATT ", "TONE", "AGC", "MEM", "SCAN", "SPLIT", "LO ADJ", "XTRA"};
	
	int result = 0;
		
	while(get_keys());    
	print_menu_head(menu_str[menu], menu_items[menu]);	//Head outline of menu
	
	//Navigate thru item list	
	switch(menu)
	{           //Print item list in full with diff. preset values
	    case 0: print_menu_item_list(menu, cur_band);  //BAND
	            result = navigate_thru_item_list(menu, menu_items[menu], cur_band, c_vfo, c_band);
	            break; 
	    case 1: print_menu_item_list(menu, sideband);  //Sideband
	            result = navigate_thru_item_list(menu, menu_items[menu], sideband, c_vfo, c_band);
	            break; 
	    case 2: print_menu_item_list(menu, c_vfo);  //VFO
	            result = navigate_thru_item_list(menu, menu_items[menu], c_vfo, c_vfo, c_band);
	            break; 
	    case 3: print_menu_item_list(menu, curatt);  //ATT
	            result = navigate_thru_item_list(menu, menu_items[menu], curatt, c_vfo, c_band);
	            break;         
	    case 4: print_menu_item_list(menu, curtone); //Tone
	            result = navigate_thru_item_list(menu, menu_items[menu], curtone, c_vfo, c_band);
	            break;
	    case 5: print_menu_item_list(menu, curagc);  //AGC
	            result = navigate_thru_item_list(menu, menu_items[menu], curagc, c_vfo, c_band);
                break;           
	    default:print_menu_item_list(menu, 0);         //All other without preset value
	            result = navigate_thru_item_list(menu, menu_items[menu], 0, c_vfo, c_band);
	}  
	
	//lcd_putnumber(0, 0, result, -1, 1, LIGHT_RED, bcolor);
	//_delay_ms(1000);
			
	if(result > -1)
	{
	    return(menu * 10 + result);
	}
	else
	{
		//Restore old settings
	    set_frequency2(f_lo[sideband]);
	    set_att(curatt);
	    set_frequency1(f_vfo[c_vfo]);
	    
	    switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
    
    return -2;
    
}    	

   //////////////////////////
  // BACKLIGHT  Settings  //
 //////////////////////////
int adjustbacklight(void)
{
	int key;
	int val = blight;
	key = get_keys();
	
	lcd_cls(bcolor);
	
	lcd_putstring(calcx(2), calcy(2), "Backlight Set", 1, YELLOW, bcolor);
	lcd_putnumber(calcx(2), calcy(4), val, -1, 1, WHITE, bcolor);	
	print_menu_help(2, 8, LIGHT_GREEN, bcolor);
	
	while(key == 0)
	{
		//Set light by changing PWM duty cycle
        if(tuningknob >= 1)
		{
		    if(val < 255)
		    {
				val += 1;
			}	
			lcd_setbacklight(val);
			lcd_putstring(calcx(2), calcy(4), ".  ", 1, YELLOW, bcolor);
			lcd_putnumber(calcx(2), calcy(4), val, -1, 1, WHITE, bcolor);
			tuningknob = 0;
		}

		if(tuningknob <= -1)  
		{    
		    if(val > 0)
		    {
				val -= 1;
			}

			lcd_setbacklight(val);
			lcd_putstring(calcx(2), calcy(4), ".  ", 1, YELLOW, bcolor);
			lcd_putnumber(calcx(2), calcy(4), val, -1, 1, WHITE, bcolor);
			tuningknob = 0;
		}			    
		key = get_keys();
	}	
	
	if(key == 2)
	{
		cli();
        while(!eeprom_is_ready());
        eeprom_write_byte((uint8_t*)482, val);
        sei();
        blight = val;
    }
    else
    {
		lcd_setbacklight(blight);
	}	    
	return val;
}

//Set TX on the 6 bands consecutively
void tx_test(void)
{
	int t1;
	int key = 0;
		
	while(get_keys());
	PORTA |= 8;
	show_txrx(1);
	while(key != 1)
	{
	    for(t1 = 0; t1 < 6; t1++)
	    {
			if(!t1)
			{
				lcd_cls(bcolor);
	            show_msg("Transmitter test mode", bcolor);
	        }
		    set_band(t1);
		    set_frequency1(c_freq[t1]);
		    show_frequency1(c_freq[t1], 1, bcolor);
		    key = 0;
		    while(!key)
		    {
			    key = get_keys();
		    }
		    while(get_keys());
		    
		    if(key == 1)
		    {
				PORTA &= ~(8); 	
	            show_txrx(0);
	            while(get_keys());
	            return;
	        }       
		}    
		while(get_keys());
	}			
	PORTA &= ~(8); 	
	show_txrx(0);
	while(get_keys());
}	

//Switch TX with dual tone oscillator
void tune(void)
{	
	int key = 0;
	
	PORTA |= 8; //TX on
	set_dualtone_oscillator(1);
	show_txrx(1);
	
	while(!key)
	{
		key = get_keys();
	}
		
	PORTA &= ~(8); //TX off
	set_dualtone_oscillator(0);
	show_txrx(0);    
}		
	
   ////////////////////////
  //     SCAN Settings  //
 ////////////////////////
//Scans a frequency range defined by 2 edge frequencies
//Scan==0: scan memories, scan=1: scan band
unsigned long scan(int mode)
{
    int t1 = 0;
    unsigned long f, df, runsecsold = 0;
    int key = 0;
    int sval;
        
    int scan_skip[MAXMEM];
    
    lcd_cls(bcolor);
    
    for(t1 = 0; t1 < MAXMEM; t1++)
    {
		scan_skip[t1] = 0;
	}
	tuningknob = 0;
	
    while(get_keys());
    
    if(!mode)
    {
		show_msg("Scanning Memories...", bcolor);
		key = 0;
        while(!key) //Scan memories
	    {
		    t1 = 0;
		    while(t1 < MAXMEM && !key)
		    {
			    f = load_frequency0(t1);
				if(is_mem_freq_ok(f, cur_band) && !scan_skip[t1])
				{
					set_frequency1(f);
				    show_frequency1(f, 1, bcolor);
				    show_mem_number(t1);
				    				    
				    sval = get_s_value(); //ADC voltage on ADC2 SVAL
				    smeter(sval, bcolor); //S-Meter
				    				    
				    while(sval > s_threshold && !key)
				    {
		 	            runsecsold = runseconds10;
		 	            key = get_keys();
		 	            while(runseconds10 < runsecsold + 1 && !key)
		 	            {
							key = get_keys();
					    }	
		 	            sval = get_s_value();
		 	            smeter(sval, bcolor); //S-Meter
		 	            
		 	            if(get_ptt()) //PTT active
			            {
				            key = 2;
				            while(get_ptt());
			            }
		 	        }
					
				    runsecsold = runseconds10;
				    while(runseconds10 < runsecsold + 20 && !key)
			        {
						key = get_keys();
						sval = get_s_value();
						smeter(sval, bcolor); //S-Meter
						if(get_ptt()) //PTT active
			            {
				            key = 2;
				            while(get_ptt());
			            }
					}	
			    } 
			    else  
			    {
					show_mem_number(t1);
					show_frequency1(0, 0, bcolor);
					key = get_keys();
				}	
				
				while(get_keys());
				
				if(tuningknob)
				{
					scan_skip[t1] = 1;
					tuningknob = 0;
				}	
				t1++;
				reset_smax();
			}
		}
		t1--;
				
		while(get_keys());
				
		if(key == 2)
		{
			return(t1); //Set this memory frequency as new operating QRG
		}
		else
		{
			return(-1);
		}	
	}
	else  //Scan band
	{	
		show_msg("Scanning Band...", bcolor);				
	    while(!key) 
	    {
			//Load edge frequencies
			for(t1 = 0; t1 < 2; t1++)
			{
	            scanfreq[t1] = load_frequency1(cur_band * 2 + 550 + t1 * 4);
				if(!is_mem_freq_ok(scanfreq[t1], cur_band))
				{
					if(!t1)
					{
					    scanfreq[t1] = band_f0[cur_band];
					}
					else    
					{
					    scanfreq[t1] = band_f1[cur_band];
					}
				}	
			}					
								
		    df = 0;
		    set_frequency1(scanfreq[0] + df);
		    show_frequency1(scanfreq[0], 1, bcolor);
		    
		    while((scanfreq[0] + df <= scanfreq[1]) && !key)
		    {
				df += 100;
		        set_frequency1(scanfreq[0] + df);
		        show_frequency1(scanfreq[0] + df, 0, bcolor);
						    
			    sval = get_s_value(); //ADC voltage on ADC2 SVAL
			    smeter(sval, bcolor); //S-Meter
				
				key = get_keys();
				
		 	    while((sval > s_threshold) && !key)
				{
					runsecsold = runseconds10;
		 	        key = get_keys();
		 	        while(runseconds10 < runsecsold + 1 && !key)
		 	        {
						key = get_keys();
					}	
		 	        sval = get_s_value();
		 	        smeter(sval, bcolor); //S-Meter
		 	    }
		 	    
			    if(get_ptt()) //PTT active
			    {
				    key = 2;
				    while(get_ptt());
			    }
			}
		}  
								
		while(get_keys());
				
		if(key == 2)
		{
			show_msg("QRG selected", bcolor);				
			return(scanfreq[0] + df); //Set this memory frequency as new operating QRG
		}
		else
		{
			show_msg("Stopped.", bcolor);				
			return(-1);
		}			
	}				    
	return(-1);
}	

unsigned long set_scan_frequency(int fpos, unsigned long f0)
{
	int xpos0 = 0;
	int ypos0 = 0;
    int key = 0;
    unsigned long f1 = f0;
    int fcolor = WHITE;
    
    lcd_putstring(xpos0, ypos0, "SET SCAN FREQ", 1, fcolor, bcolor);
    if(!fpos)
    {
        lcd_putstring(xpos0, ypos0 + 1, "1st FREQUENCY", 1, fcolor, bcolor);
    }   
    else
    {
        lcd_putstring(xpos0, ypos0 + 1, "2nd FREQUENCY", 1, fcolor, bcolor);
    }   
        
    show_frequency1(f1, 1, bcolor);
        	
    while(!key)
    {
        if(tuningknob >= 1) //Turn CW
		{
			f1 += calc_tuningfactor();
			set_frequency1(f1);
			show_frequency1(f1, 0, bcolor);
            tuningknob = 0;
		}

		if(tuningknob <= -1)  //Turn CCW
		{    
			f1 -= calc_tuningfactor();
			show_frequency1(f1, 0, bcolor);
            set_frequency1(f1);
            tuningknob = 0;
		}		
		key = get_keys();
	}
	
	while(get_keys());
	
	if(key == 2)
	{
		store_frequency1(f1, cur_band * 2 + 550 + fpos * 4);
		scanfreq[fpos] = f1;
		return f1;
	}	
	
	return 0;
	
}

//Scans 16 memories
void set_scan_threshold(void)
{
	int xpos0 = 0;
	int ypos0 = 0;
    int key = 0;
    int thresh = s_threshold;
    int fcolor = WHITE;
    
    lcd_cls(bcolor);
    smeter(thresh, bcolor);
    draw_meter_scale(0, bcolor);
        
    lcd_putstring(xpos0, ypos0, " SCAN THRESH ", 1, fcolor, bcolor);
        
    lcd_putstring(xpos0, ypos0 + 2, "  ", 1, fcolor, bcolor);
    lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 1, WHITE, bcolor);
    
    	
    while(!key)
    {
         if(tuningknob >= 1)//Turn CW
		{
			if(thresh < 200)
			{
				thresh++;
			}
			smeter(thresh, bcolor);
	
            lcd_putstring(xpos0, ypos0 + 2, "   ", 1, fcolor, bcolor);
            lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 1, fcolor, bcolor);
    
			tuningknob = 0;
		}

		if(tuningknob <= -1)  //Turn CCW
		{    
			if(thresh > 0)
			{
				thresh--;
			}
			smeter(thresh, bcolor);
			 
            lcd_putstring(xpos0, ypos0 + 2, "   ", 1, fcolor, bcolor);
            lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 1, fcolor, bcolor);
            tuningknob = 0;
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		s_threshold = thresh;
		eeprom_write_byte((uint8_t*)129, s_threshold);
	}	
	
}	


  //////////////////////////
 //  INTERRUPT HANDLERS  //
//////////////////////////
//Rotary encoder
ISR(INT2_vect)
{ 
    tuningknob = ((PIND >> 2) & 0x03) - 2;           // Read PD2 and PD3 and convert to 1 or -1 
	tuningcount++;
}

ISR(TIMER1_COMPA_vect)
{
	runseconds10++; 
    tuningcount = 0;
}

int calc_tuningfactor(void)
{
	return (tuningcount * tuningcount  * tuningcount);
}	

  ///////////////////
 //// U A R T   ////
///////////////////
void usart_init(int baudrate)
{
		
    /* Set baud rate */
	UBRR0H = (unsigned char)(baudrate >> 8);
	UBRR0L = (unsigned char)baudrate;
	
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN)|(1<<TXEN);
	
	/* Set frame format: 8data, 1stop bit, NO parity */
	UCSR0C = (1 << UCSZ00)|(1 << UCSZ01);
		
}

void usart_transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while(!(UCSR0A & (1<<UDRE)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

int usart_receive(void)
{
	/* Wait for data to be received */
	if(UCSR0A & (1<<RXC))
	{
		/* Get and return received data from buffer */
	    return UDR0;
	}
	else
	{
		return -1;
	}	    
}
	
int main(void)
{
	int t1;
	
	curagc = 1;
	curtone = 1;
	int key;
	unsigned long rval = 0;
	
	int adcval;
	
	unsigned long runseconds10s = 0;     //Ms counter for displaying S-Value 
	unsigned long runseconds10speak = 0; //Ms counter for resetting peak value of S-Meter
	unsigned long runseconds10msg = 0;   //Ms for holding message displayed
	unsigned long runseconds10volts = 0; //Ms for volatge check
	
	int sval = 0;
	int cur_vfo = 0, alt_vfo = 1;
	
	unsigned long freq_temp0 = 0;      
	unsigned long freq_temp1 = 0;    
		
    LCDCTRLDDR = 0xF0; //LCD CTRL PA4:PA7 blue, brown, violet, green
    LCDDATADDR = 0xFF; //LCD DATA PC0:PC7
    
    int uchar;
    
    _delay_ms(100);
    
    //Relays for band set PA0, PA1, PA2
    DDRA |= 0x07;
    
    //PA3 TX relay output
    DDRA |= 0x08;
    PORTA &= ~(8); 		 //Set to RX mode
    
    //DDS1
    DDS1_DDR = 0xF0; //DDS1 on PD4:PD7
    
    //DDS2
    DDS2_DDR = 0xF0; //DDS2 on PB4:PB7

    //PB0 for ATT
    DDRB |= (1 << 3);
    
    DDRG = 0x1B; //Set: AGC (PG0:PG1), Tone (PG3:PG4)
    
    //ADC0 Pullup resistor
    ADC_KEY_PORT = 0x01; //ADC        
    
    //ROTARY ENCODER
    PORTD |= (1 << PD2) | (1 << PD3);//INPUT: Pullup resistors for PD2 and PD3 rotary encoder
    
    PORTG |= 4; //Pullup for TX/RX indicator
        
    //Timer 1 as counter for 10th seconds
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1 << CS10) | (1 << CS12) | (1<<WGM12);   // Prescaler = 1/1024 based on system clock 16 MHz
                                                       // 15625 incs/sec
                                                       // and enable reset of counter register
	OCR1AH = (1562 >> 8);                             //Load compare values to registers
    OCR1AL = (1562 & 0x00FF);
	TIMSK |= (1<<OCIE1A);
		
	// Timer 3 PWM for display light
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM30); // 8-bit PWM phase-correct
    TCCR3B |= (1<<CS30); // No prescale
    DDRE |= (1<<3);
    	
	//Interrupt definitions for rotary encoder  
	EIMSK = (1<<INT2); // | (1<<INT3));  //enable encoder pins as interrupt source
	EICRA = (1<<ISC20)|(1<<ISC21);                      // enable pin change interupts  
    
    //Reset DDS1 (AD9951)
    DDS1_PORT |= (DDS1_RESETPIN); 
    _delay_ms(100); 
	DDS1_PORT &= ~(DDS1_RESETPIN);          
    _delay_ms(100); 
	DDS1_PORT |= (DDS1_RESETPIN);     
         
    //Start DDS2
    /*
    DDS2_PORT &= ~(DDS2_RESETPIN);  //Bit erase   
    _delay_ms(100);       //wait for > 20ns i. e. 1ms minimum time with _delay_s()
    DDS2_PORT |= (DDS2_RESETPIN);       //Bit set
    _delay_ms(100);       //wait for > 20ns i. e. 1ms minimum time with _delay_s()
	DDS2_PORT &= ~(DDS2_RESETPIN);  //Bit erase   
	*/
	     
	//Display init
	LCDCTRLPORT |= LCDRES;     //1
    _delay_ms(5);
	LCDCTRLPORT &= ~(LCDRES);  //0
	_delay_ms(5);
    LCDCTRLPORT |= LCDRES;     //1
    _delay_ms(5);

	lcd_init();
	_delay_ms(100);
	lcd_cls(bcolor);
	_delay_ms(100);
	
	//UART init
	usart_init(UARTBAUDSET);

    //ADC config and ADC init
    ADCSRA = (1<<ADPS0) | (1<<ADPS1) | (1<<ADEN); //Prescaler 64 and ADC on
	get_adc(0); //One dummy conversion
        	    
	//Load start values
	//Load last band used
	cur_band = load_last_band();
	if(cur_band == -1)
	{
		cur_band = 2; //Set 40m as default
	}
	set_band(cur_band);
	    
	sideband = std_sideband[cur_band];
		
	//Load VFO data and VFO number
    cur_vfo = load_last_vfo();
    if(cur_vfo < 0 || cur_vfo > 1)
    {
		cur_vfo = 0;
		alt_vfo = 1;
	}
	
	if(cur_vfo)
	{
		alt_vfo = 0;
	}
	else
	{
		alt_vfo = 1;
	}	
	    
	//Load valid frequency if possible int0 2 VFOs
	for(t1 = 0; t1 < 2; t1++)
	{   
	    freq_temp0 = load_frequency0(cur_band + 96 + t1); 
        //Check if freq is OK
        if(is_mem_freq_ok(freq_temp0, cur_band))
        {
	         f_vfo[t1] = freq_temp0;
	    }
	    else
	    {
	        f_vfo[t1] = c_freq[cur_band];
	    }
	}    
	    
    //Load last stored freqeuncy
    //Check if memory place is not a random number anywhere in EEPROM
    last_memplace = load_last_mem();
    if(last_memplace < 0 || last_memplace > 15)
    {
		last_memplace = 0;
	}
	
	//Load scan threshold
    s_threshold = eeprom_read_byte((uint8_t*)129);          	
    if(s_threshold < 0 || s_threshold > 255)
    {
		s_threshold = 100;
	}
	
	//Load scan edge frequencies
	scanfreq[0] = load_frequency0(108);
	if(!is_mem_freq_ok(scanfreq[0], cur_band))
	{
		scanfreq[0] = band_f0[cur_band];
		scanfreq[1] = band_f1[cur_band];
	}	
	
	//Split default setting TXA RXB
	vfo_s[0] = 0;
	vfo_s[1] = 1;
    
    if(is_mem_freq_ok(load_frequency0(last_memplace), cur_band))
	{
		show_mem_freq(load_frequency0(last_memplace), bcolor);
	}
	else
	{	
		show_mem_freq(0, bcolor);	
	}	
          
	//Load sets
	// 480: TONE set
    // 481: AGC set
	curtone = eeprom_read_byte((uint8_t*)480);
	if(curtone < 0 ||curtone > 3)
	{
		curtone = 1;
	}	
	show_tone(curtone, bcolor);
	set_tone(curtone);
	
	curagc = eeprom_read_byte((uint8_t*)481);
	if(curagc < 0 || curagc > 3)
	{
		curagc = 2;
	}
	show_agc(curagc, bcolor);
	set_agc(curagc);
	
	curatt = eeprom_read_byte((uint8_t*)483);
    if(curatt < 0 || curatt > 1)
    {
		curatt = 0;
	}	
    set_att(curatt);
    
    show_all_data(f_vfo[cur_vfo], f_vfo[alt_vfo], 0, sideband, 0, cur_vfo, 0, 0, 0, 0, last_memplace, txrx);
	
	//Load LO frequencies if available
    for(t1 = 0; t1 < 2; t1++)
    {
		f_lo[t1] = load_frequency1(t1 + 128);
		if((f_lo[t1] < F_LO_LSB - 2000) || (f_lo[t1] > F_LO_USB + 2000))
		{
			if(!t1)
			{
			    f_lo[t1] = F_LO_LSB;
			}   
			else
			{
			    f_lo[t1] = F_LO_USB;
			}   
		}
	}		
			
    for(t1 = 0; t1 < 5; t1++)
    {
		set_frequency1(f_vfo[cur_vfo]);
		set_frequency2(f_lo[sideband]);
        _delay_ms(10);
    }

    blight = eeprom_read_byte((uint8_t*)482);
    if(blight < 0 || blight > 255)
    {
		blight = 128;
	}	
    lcd_setbacklight(blight);
    
    lcd_putnumber(calcx(0), calcy(14), f_lo[sideband], -1, 1, WHITE, bcolor);
    
    //Init TWI
    twi_init();
        
    //Load TX preset values
    for(t1 = 0; t1 < 6; t1++)
    {
		tx_preset[t1] = load_tx_preset(t1);
	}	
	//Load TX preset
	mcp4725_set_value(load_tx_preset(cur_band)); 
    
    sei();
        
    for(;;) 
	{
		//TUNING		
		if(tuningknob >= 1 && !txrx)
		{    
		    f_vfo[cur_vfo] += calc_tuningfactor();  
		    set_frequency1(f_vfo[cur_vfo]);
		    tuningknob = 0;
		    show_frequency1(f_vfo[cur_vfo], 0, bcolor);
		}
		
		if(tuningknob <= -1 && !txrx)  
		{
		    f_vfo[cur_vfo] -= calc_tuningfactor();  
		    set_frequency1(f_vfo[cur_vfo]);
		    tuningknob = 0;
			show_frequency1(f_vfo[cur_vfo], 0, bcolor);
		}
				
		//MENU
		key = get_keys();	
			
        switch(key)
		{
		    case 1:	rval = menu0(f_vfo[cur_vfo], cur_vfo, cur_band);
		            key = 0;
			        
			        //Band change
			        if(rval >= 0 && rval < 6)
			        {
						//last_freq[cur_band] = f_vfo[cur_vfo];
			            cur_band = rval;
			            set_band(cur_band);
			            
			            //Load VFOs A and B with last stored frequencies
	                    for(t1 = 0; t1 < 2; t1++)    
	                    {
                            freq_temp0 = load_frequency0(cur_band + 96 + t1); 

                            //Check if freq is OK
                            if(is_mem_freq_ok(freq_temp0, cur_band))
                            {
		                         f_vfo[t1] = freq_temp0; 
		                    }
		                    else
		                    {
		                         f_vfo[t1] = c_freq[cur_band];
		                    }    
		                }    
        			            
			            set_frequency1(f_vfo[cur_vfo]);
			            sideband = std_sideband[cur_band];
                        set_frequency2(f_lo[sideband]);
                        show_frequency1(f_vfo[cur_vfo], 1, bcolor);
                        show_sideband(sideband, 0);
	                    store_last_band(cur_band);
	                    last_memplace = 0;
	                    freq_temp0 = load_frequency0(last_memplace);
	                    
	                    //Load TX preset and show
	                    mcp4725_set_value(load_tx_preset(cur_band));
			        }   
			        
			        //SIDEBAND
			        //////////
			        if(rval == 10 || rval == 11) //LSB or USB
			        {
			            sideband = rval - 10;
			            set_frequency1(f_vfo[cur_vfo]);
			            set_frequency2(f_lo[sideband]);
			        }    
			        ///////
			        //VFO
			        /////
			        if(rval == 20 || rval == 21) //VFO A or VFO B
			        {
						freq_temp0 = f_vfo[cur_vfo];
						alt_vfo = cur_vfo;
						cur_vfo = rval - 20;
						if(!is_mem_freq_ok(f_vfo[cur_vfo], cur_band))
						{
						     f_vfo[cur_vfo] = c_freq[cur_band];
						}	     
				        set_frequency1(f_vfo[cur_vfo]);
				        set_frequency2(f_lo[sideband]);
			            show_frequency1(f_vfo[cur_vfo], 1, bcolor);
			        }    
			        
			        if(rval == 22)
			        {
						f_vfo[0] = f_vfo[1]; //VFO A = VFO B
				    }
				    
				    if(rval == 23)
			        {
						f_vfo[1] = f_vfo[0]; //VFO B = VFO A
				    }
				    
				    //ATT, TONE and AGC
				    if(rval >= 30 && rval <= 32)
				    {
						curatt = rval - 30;
						show_att(curatt, bcolor);
						set_att(curatt);
					}
					
				    if(rval >= 40 && rval <= 44)
				    {
						curtone = rval - 40;
						show_tone(curtone, bcolor);
						set_tone(curtone);
					}
					
					if(rval >= 50 && rval <= 54)
				    {
						curagc = rval - 50;
						show_agc(curagc, bcolor);
						set_agc(curagc);
					}
						
				    /////////////////////
				    //MEMORY, SCAN, SPLIT
				    /////////////////////
				    switch(rval)
				    {
				        case 60:t1 = save_mem_freq(f_vfo[cur_vfo], last_memplace); //Store
					            if(t1 > -1)
					            {
									store_frequency0(f_vfo[cur_vfo], 16 + cur_vfo);
									last_memplace = t1;
									store_last_mem(t1);
					    		}    
					    		show_frequency1(f_vfo[cur_vfo], 0, bcolor);
					            set_frequency1(f_vfo[cur_vfo]);
					            set_frequency2(f_lo[sideband]);
					            break;
					            
					    case 61:freq_temp1 = recall_mem_freq(0);     //Recall QRG  
					            if(is_mem_freq_ok(freq_temp1 & 0x0FFFFFFF, cur_band) ) //Separate freq from mem_place
					            {
									last_memplace = (freq_temp1 >> 28) & 0x0F;
								    f_vfo[cur_vfo] = freq_temp1  & 0x0FFFFFFF;
									set_frequency1(f_vfo[cur_vfo]);
									show_frequency1(f_vfo[cur_vfo], 1, bcolor);
									show_mem_freq(f_vfo[cur_vfo], bcolor);
									freq_temp1 &= 0x0FFFFFFF;
								}	
								else
								{
								    set_frequency1(f_vfo[cur_vfo]);
								    show_frequency1(f_vfo[cur_vfo], 1, bcolor);
								}	
								break;
					
				        case 70:t1 = scan(0); //Scan MEMs
				                if(t1 > 0)
				                {
					                freq_temp0 = load_frequency0(t1);
					                if(is_mem_freq_ok(freq_temp0, cur_band))
					                {
									    f_vfo[cur_vfo] = freq_temp0;
								    }	
								 }   
								 set_frequency1(f_vfo[cur_vfo]);
								 set_frequency2(f_lo[sideband]);
					            break;
					    
					    case 71:freq_temp0 = scan(1); //Scan BAND
					            if(is_mem_freq_ok(freq_temp0, cur_band))
					            {
									f_vfo[cur_vfo] = freq_temp0;
								}	
								set_frequency1(f_vfo[cur_vfo]);
								set_frequency2(f_lo[sideband]);
								break;
					                  
					    case 72:lcd_cls(bcolor); //Set scan frequencies
					            //Load from EEPROM
					            for(t1 = 0; t1 < 2; t1++)
					            {
					                scanfreq[t1] = load_frequency1(cur_band * 2 + 550 + t1 * 4);
					                if(!is_mem_freq_ok(scanfreq[t1], cur_band))
					                {
										if(!t1)
										{
									        scanfreq[t1] = band_f0[cur_band] + 100;
									    }
									    else    
									    {
									        scanfreq[t1] = band_f1[cur_band] - 100;
									    }
								    }	
								}
								
					            scanfreq[0] = set_scan_frequency(0, scanfreq[0]);
					            scanfreq[1] = set_scan_frequency(1, scanfreq[1]); 
					            
					            if(scanfreq[1] < scanfreq[0]) //Change freq order if f0 > f1
					            {
									freq_temp0 = scanfreq[1];
									scanfreq[1] = scanfreq[0];
									scanfreq[0] = freq_temp0;
								}
								
								//Store in EEPROM
								for(t1 = 0; t1 < 2; t1++)
								{
								    store_frequency1(cur_band * 2 + 550 + t1 * 4, scanfreq[t1]);
								}
								
					            break;
					                
					    case 73:set_scan_threshold();            
					            break;
					    
					    //SPLIT mode        
					    case 80: split = 1;
					             vfo_s[0] = 0;  //TXA RXB
					             vfo_s[1] = 1;
					             break;               
					             
					    case 81: split = 2;
					             vfo_s[0] = 1;  //TXA RXB
					             vfo_s[1] = 0;
					             break;               
					             
					    case 82: split = 0;
					             show_split(0, bcolor);
					             break;   
					    
					    case 90: set_lo_freq(0);
					             break;         
					             
					    case 91: set_lo_freq(1);
					             break;         
					             
					    case 100: adjustbacklight();
					             break;          
					    
					    case 101: tx_test();
					             set_frequency1(f_vfo[cur_vfo]);
					             show_frequency1(f_vfo[cur_vfo], 1, bcolor);
					             set_band(cur_band);
					             txrx = 0;
					             break;                   
					    case 102: tune();         
					             break;
					    case 103: tx_preset_adjust();
					             break;         
					    case 104: rcv_mem_frequencies();
					              break;
				    }
				       
				    lcd_cls(bcolor);    
			        show_all_data(f_vfo[cur_vfo], f_vfo[alt_vfo], 1, sideband, 0, cur_vfo, 0, 0, 0, 0, last_memplace, txrx);
			        show_mem_freq(freq_temp1, bcolor);
			           
			        while(get_keys());
			        break;
			        
			case 2: store_frequency0(f_vfo[0], cur_band + 96);
			        store_frequency0(f_vfo[1], cur_band + 97);
			        store_last_band(cur_band);
			        store_last_vfo(cur_vfo);
			        while(get_keys());
			        show_msg("Frequency data saved.", bcolor);
			        runseconds10msg = runseconds10;
			        break;
			        
			case 3: while(get_keys());
                    rval = menu1(10, f_vfo[cur_vfo], cur_vfo, cur_band);
                    switch(rval)
				    {
				        case 100: adjustbacklight();
					             break;          
					    
					    case 101: tx_test();
					             set_frequency1(f_vfo[cur_vfo]);
					             show_frequency1(f_vfo[cur_vfo], 1, bcolor);
					             set_band(cur_band);
					             txrx = 0;
					             break;                   
					    case 102: tune();         
					             break;
					    case 103: tx_preset_adjust();
					             break;   
			        }
	                lcd_cls(bcolor);    
			        show_all_data(f_vfo[cur_vfo], f_vfo[alt_vfo], 1, sideband, 0, cur_vfo, 0, 0, 0, 0, last_memplace, txrx);
			        show_mem_freq(freq_temp1, bcolor);
			        while(get_keys());
			        break;      
		}	            
		
		//METER
		//After 1/10th sec check S-Val resp. PWR value
		if(runseconds10 > runseconds10s)
		{
			if(!txrx)
		 	{   sval = get_s_value();
				smeter((sval >> 1) + (sval >> 2), bcolor); //S-Meter * 1.5
		 	}
		 	else
		 	{
				adcval = get_adc(1);
				smeter(adcval, bcolor); //*0.5
			}    
 		 	runseconds10s = runseconds10;
 		}	
 		
 		//Delete peak value of meter every 2 seconds
		if(runseconds10 > runseconds10speak + 20)
		{
			reset_smax();
			runseconds10speak = runseconds10;				
			usart_transmit('.');
		}
		
		//Show temperature and clear message line after 10 seconds
		if(runseconds10 > runseconds10msg + 100)
		{
			show_temp(bcolor);
		    runseconds10msg = runseconds10;
		    
		    show_msg("", bcolor);
		    show_msg("(K1) Menu (K3) Xtra func", bcolor);
		    
		}
		
		//Measure voltage every 5 secs
		if(runseconds10 > runseconds10volts + 50)
		{
		    show_voltage(bcolor);
		    runseconds10volts = runseconds10;
		}

		//Detect PTT
		if(get_ptt())
		{
			if(!txrx)
		    {
			    txrx = 1;
			    show_txrx(txrx);
                draw_meter_scale(1, bcolor);				
		    
    		    switch(split)
	    	    {
		    		case 1: set_frequency1(f_vfo[vfo_s[0]]);
			    	        show_frequency1(f_vfo[vfo_s[0]], 0, bcolor);
	                        show_frequency2(8, 9, f_vfo[vfo_s[1]], bcolor, 100, 1); 
				            break;
				    case 2: set_frequency1(f_vfo[vfo_s[1]]);
				            show_frequency1(f_vfo[vfo_s[1]], 0, bcolor);
    				        show_frequency2(8, 9, f_vfo[vfo_s[0]], bcolor, 100, 1); 
	    			        break;        
		    		case 0: set_frequency1(f_vfo[cur_vfo]);
			    	        show_frequency1(f_vfo[cur_vfo], 0, bcolor);        
			    }        
			    
			    PORTA |= 8; //TX on
			}   
	    }
	    else
	    {	
		    if(txrx)
		    {
			    txrx = 0;
			    show_txrx(txrx);
			    draw_meter_scale(0, bcolor);
			    
			    switch(split)
		        {
				    case 1: set_frequency1(f_vfo[vfo_s[1]]);
				            show_frequency1(f_vfo[vfo_s[1]], 0, bcolor);
	                        show_frequency2(8, 9, f_vfo[vfo_s[0]], bcolor, 100, 1); 
				            break;
				    case 2: set_frequency1(f_vfo[vfo_s[0]]);
				            show_frequency1(f_vfo[vfo_s[0]], 0, bcolor);
				            show_frequency2(8, 9, f_vfo[vfo_s[1]], bcolor, 100, 1); 
				            break;        
				    case 0: set_frequency1(f_vfo[cur_vfo]);
				            show_frequency1(f_vfo[cur_vfo], 0, bcolor);        
			    }        
			    
			    PORTA &= ~(8); 		//TX off    
		    }
		}   
		
		uchar = usart_receive();
	    if(uchar > -1)
	    {
		    lcd_putchar(calcx(0), calcy(14), uchar, 1, YELLOW, DARK_BLUE1);
	    }
	}	
	
	
	return 0;
}

