/*
 * heartbeat.c
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
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include "sharplcd.h"

#define BUTTON1 PD5

uint8_t data[512]; 
uint8_t dataIndex; 

uint16_t mode; // Keep track of mode. 

char modeString[] = "Mode xx";

uint8_t buttonFlag; // Track if button pressed. 
uint8_t buttonCount; // Count button press time. 

uint16_t trigger;  // Trigger level. 
uint8_t trigTime;  // Length of one trigger to next. 
uint8_t trigCount; // Count from one trigger to next. 
uint8_t trigPos;   // Where in the data is the trigger. 

int main(void)
{
  /* Initialize ADC. */
  // AVCC voltage reference, left adjust result, ADC0 selected.  
  ADMUX |= (1<<REFS0)|(1<<ADLAR); 
  // Enable ADC, start conversion, prescaler 16. 
  ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADSC) | (1<<ADPS2); 
  // Disable digital input ADC0. 
  DIDR0 |= (1<<ADC0D);
  
  /* Initialize LCD ports. */
  sharpLCDInit(); 

  /* Initialize button. */
  // Set internal pull-up high. 
  PORTD |= (1<<BUTTON1);
  buttonFlag = 0;
  buttonCount = 0; 

  /* Initialize timer interrupt. */
  // Clear counter on compare match. 
  TCCR0A |= (1<<WGM01);
  // Prescaler 1024. 
  TCCR0B |= (1<<CS00) | (1<<CS02);
  // Compare Match A interrupt enable. 
  TIMSK0 |= (1<<OCIE0A);
  // Compare register A. 
  OCR0A = 52; // 52 = 300.48Hz
  //OCR0A = 78; // 78 = 200Hz
  //OCR0A = 255; // 255 = 61Hz

  dataIndex = 0;

  sei(); // Enable interrupts. 
  
  int i;


  char timeString[4];
  timeString[3] = 0; 

  uint8_t pulse;
  
  while(1)
  {
    sharpLCDClearBuffer();

    //for(i=0; i<95; i++)
    //{
    //  sharpLCDDrawLine(i,data[i]*96/256,i+1,data[i+1]*96/256);
    //}

    uint16_t startScreen; // Where in data to start screen drawing. 

    if (trigPos >= trigTime/8) // Check trigPos larger or equal to trigTime/8.
    {
      startScreen = trigPos - trigTime/8;  // Start slightly before large peak. 
    }
    else
    {
      startScreen = (512 + trigPos) - trigTime/8; // 511 is highest possible value. 
    }

    for (i=0; i<95; i++)
    {
      if ((startScreen + (i*trigTime)/96) < dataIndex)
      { 
        sharpLCDDrawLine(i, data[(startScreen+(i*trigTime)/96)%512]*96/256, i+1, data[(startScreen+(i*trigTime)/96+1)%512]*96/256);
      }
    }

    trigger = mode*10;

    sharpLCDDrawLine(0,trigger*96/256,10,trigger*96/256);

    modeString[6] = 48 + mode%10;
    modeString[5] = 48 + mode/10;

    pulse = 18000/trigTime;  // Pulse calculation at 300Hz sample rate. 

    timeString[0] = 48 + (pulse/100);
    timeString[1] = 48 + ((pulse%100)/10);
    timeString[2] = 48 + (pulse%10);
    
    sharpLCDPrint(timeString, 10+12, 18);


    sharpLCDPrint(modeString, 10, 10);

    sharpLCDWriteBuffer();

    _delay_loop_2(0xFFFF);
  }

  return 0; 
}

// 122 Hz
ISR(TIMER0_COMPA_vect)
{
  /* Read ADC. */
  data[dataIndex] = ADCH;

  trigCount++;
  if (dataIndex > 0 && data[dataIndex] <= trigger && data[dataIndex - 1] > trigger)
  {
    trigTime = trigCount; 
    trigCount = 0;
    trigPos = dataIndex; 
  } 
    
  dataIndex++;

  if (dataIndex == 512) // 512 = We have come to the end. 
  {
    dataIndex = 0;
  }

  /* Read button. */
  if (PIND & (1<<BUTTON1))
  {
    if (buttonCount > 12)
    {
      if (!buttonFlag)
      {
        buttonFlag = 1; 
        if (++mode > 25)
        {
          mode = 0; 
        }
      }
    }
    else
    {
      buttonCount++;
    }
  }
  else
  {
    buttonFlag = 0; 
    buttonCount = 0;
  }
}
