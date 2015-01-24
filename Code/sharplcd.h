/*
 * sharplcd.h
 * 
 * Copyright 2015 Oskari Rundgren <orundg@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#ifndef SHARPLCD_H
#define SHARPLCD_H

#include "font.h"

// Sharp Memory LCD Commands. 
#define SHARPLCDWRITELINE 0x01
#define SHARPLCDCLEARMEMORY 0x04

// Ports. 
#define SHARPLCDSCLK PB5
#define SHARPLCDSI PB3
#define SHARPLCDSS PB2
#define SHARPLCDDISP  PB1

uint8_t sharpLCDBuffer[1152];

void sharpLCDInit(void);
void sharpLCDClearScreen(void);
void sharpLCDWriteLine(uint8_t vcom, uint8_t sharpLCDLineNumber, uint8_t *sharpLCDData);
void sharpLCDTransmit(uint8_t cData);
void sharpLCDWriteBuffer(void);
void sharpLCDDrawChar(uint8_t x, uint8_t y, char c);
void sharpLCDToggleVcom(void);
void sharpLCDSetPixel(uint8_t x, uint8_t y);
void sharpLCDUnsetPixel(uint8_t x, uint8_t y);
void sharpLCDClearBuffer(void);
void sharpLCDDrawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);
void sharpLCDPrint(char *string, uint8_t x, uint8_t y);

#endif
