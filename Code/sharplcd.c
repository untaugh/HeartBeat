/*
 * sharplcd.c
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

#include <avr/io.h>
#include "sharplcd.h"
#include "font.h"

// Frame buffer with 12 bytes per line, and 96 lines. 
uint8_t sharpLCDBuffer[1152];
uint8_t sharpLCDVcom;

void sharpLCDInit(void)
{  
  // Set port outputs. 
  DDRB |= (1<<SHARPLCDSCLK) | (1<<SHARPLCDSI) | (1<<SHARPLCDSS) | (1<<SHARPLCDDISP);

  // Enable SPI, set master, clock rate Fosc/16, LSB first. 
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<DORD);

  // Set DISP high to enable LCD. 
  PORTB |= (1<<SHARPLCDDISP);
}

void sharpLCDWriteLine(uint8_t vcom, uint8_t sharpLCDLineNumber, uint8_t *sharpLCDData)
{
  int i;

  PORTB |= (1<< SHARPLCDSS);

  if (vcom)
    sharpLCDTransmit(SHARPLCDWRITELINE & 0b00000010);
  else
    sharpLCDTransmit(SHARPLCDWRITELINE);
    
  sharpLCDTransmit(sharpLCDLineNumber);

  for (i=0; i<12; i++)
  {
    sharpLCDTransmit(sharpLCDData[i]);
  }

  // Trailer
  sharpLCDTransmit(0x00);
  sharpLCDTransmit(0x00);

  PORTB &= ~(1<< SHARPLCDSS);
}

void sharpLCDToggleVcom(void)
{
  if (sharpLCDVcom)
    sharpLCDVcom = 0;
  else
    sharpLCDVcom = 0x02;
}

void sharpLCDWriteBuffer(void)
{
  int line;
  int column; 
  
  // Chip select high. 
  PORTB |= (1<< SHARPLCDSS);

  sharpLCDTransmit(SHARPLCDWRITELINE | sharpLCDVcom);
  sharpLCDToggleVcom();

  for (line = 0; line <96; line++)
  {
    sharpLCDTransmit(line+1); // Begin with first line. Addressing begins with 1. 

    for (column = 0; column < 12; column++)
    {
      sharpLCDTransmit(~sharpLCDBuffer[line*12+column]);
    }
    
    sharpLCDTransmit(0x00); // Trailer for line. 
  }

  sharpLCDTransmit(0x00); // Trailer for all data. 

  // Chip select low. 
  PORTB &= ~(1<< SHARPLCDSS);
}

void sharpLCDTransmit(uint8_t cData)
{
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}

void sharpLCDDrawChar(uint8_t x, uint8_t y, char c)
{
  int column,row;
  
  for (column = 0; column < 5; column++)
  {
    uint8_t fontColumn = pgm_read_byte(font + (c*5)+column);
    
    for (row=0; row<8; row++)
    {
      if (fontColumn & 1)
        sharpLCDSetPixel(x+column,y+row);
      else
        sharpLCDUnsetPixel(x+column,y+row);
      
      fontColumn >>= 1; 
    }
  }
}

void sharpLCDSetPixel(uint8_t x, uint8_t y)
{
  sharpLCDBuffer[y*12+x/8] |= (1 << x%8);
}

void sharpLCDUnsetPixel(uint8_t x, uint8_t y)
{
  sharpLCDBuffer[y*12+x/8] &= ~(1 << x%8);
}

void sharpLCDClearBuffer(void)
{
  uint16_t i;

  for (i=0; i<1152; i++)
  {
    sharpLCDBuffer[i] = 0x00;
  }
}

void sharpLCDClearScreen(void)
{
  // Chip select high. 
  PORTB |= (1<< SHARPLCDSS);

  sharpLCDTransmit(SHARPLCDCLEARMEMORY | sharpLCDVcom);
  sharpLCDToggleVcom();
  sharpLCDTransmit(0x00); // Trailer.
  
  // Chip select low. 
  PORTB &= ~(1<< SHARPLCDSS);
}

void sharpLCDPrint(char *string, uint8_t x, uint8_t y)
{
  uint8_t row, cdata, col, ccount;

  ccount = 0; // String char index. 

  while (string[ccount]) // Traverse all chars. 
  {
    for (col=0; col<5; col++) // Column of a character. 
    {
      cdata = pgm_read_byte(&font[col+5*string[ccount]]);
      for (row = 0; row<8; row++) // Row of a character. 
      {
        if (cdata & (1<<row))
          sharpLCDSetPixel(ccount*6+x+col,y+row); // One empty column between chars. 
      }
    }
    ccount++;
  }
}

void sharpLCDDrawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd)
{
  uint8_t i, temp, x, y;

  int xDelta, yDelta, xDeltaAbs, yDeltaAbs;

  x = xStart;
  y = yStart; 
  xDelta = xEnd - xStart;
  yDelta = yEnd - yStart; 
  xDeltaAbs = xDelta; 
  yDeltaAbs = yDelta; 

  if (xDeltaAbs < 0)
  {
    xDeltaAbs = -xDeltaAbs;
  }

  if (yDeltaAbs < 0)
  {
    yDeltaAbs = -yDeltaAbs;
  }

  if (xDeltaAbs > yDeltaAbs)
  {
    if (xStart > xEnd)
    {
      x = xEnd; 
      y = yEnd;
      yDelta = -yDelta;
    }
    
    for (i=0; i < xDeltaAbs; i++)
    {
      sharpLCDSetPixel(x+i, y + (yDelta*i)/xDeltaAbs);
    }
  }
  else
  {
    if (yStart > yEnd)
    {
      x = xEnd; 
      y = yEnd;
      xDelta = -xDelta;
    }
    
    for (i=0; i < yDeltaAbs; i++)
    {
      sharpLCDSetPixel(x + (xDelta*i)/yDeltaAbs, y+i);
    }
  }

  
}
