#include "constants.h"
#include "packet.h"

// Defining Ports used for the Colour Sensor (they are not all the same Port Data Register)
#define COLOUR_S0 (1 << PB2) // Port B
#define COLOUR_S1 (1 << PB1) // Port B
#define COLOUR_S2 (1 << PA4) // Port A
#define COLOUR_S3 (1 << PA6) // Port A
#define COLOUR_READER (1 << PD1) // Port D
#define COLOUR_ON (1 << PA7) // Port A
#define COLOUR_LED (1 << PB3) // Port B
#define COLOUR_GROUND (1 << PA5) // Port A

void initializeColourSensor() {
  // Set the reading pins to be input and the S pins and power source to be output pins
  DDRB |= COLOUR_S0 | COLOUR_S1 | COLOUR_LED;
  DDRD &= ~(COLOUR_READER);
  DDRA |= COLOUR_S2 | COLOUR_S3 | COLOUR_ON | COLOUR_GROUND;

  // Set the frequency of the colour sensor to be at 20%
  PORTB |= COLOUR_S0;
  PORTB &= ~(COLOUR_S1);

  PORTA |= COLOUR_ON;
  PORTA &= ~COLOUR_GROUND;

  // Turn off the colour sensor on start up
  PORTK &= ~(COLOUR_LED);
}

long getAvgReading() {
  long total = 0;
  for (int i = 0; i < 50; i++) {
    total += pulseIn(20, LOW);
  }
  return total / 50;
}

void setColour(Tcolour colour) {
  switch(colour)
  {
    case RED:
      PORTA &= ~COLOUR_S2;
      PORTA &= ~COLOUR_S3;
      break;

    case GREEN:
      PORTA |= COLOUR_S2;
      PORTA |= COLOUR_S3;
      break;

    case BLUE:
      PORTA &= ~COLOUR_S2;
      PORTA |= COLOUR_S3;
      break;

    case WHITE:
      PORTA |= COLOUR_S2;
      PORTA &= ~COLOUR_S3;
      break;
  }
}

void readColour() {
  // Turn on the LED
  PORTB |= COLOUR_LED;
  delay(10);
  setColour(WHITE);
  PORTA |= COLOUR_S2;
  PORTA &= ~COLOUR_S3;
  long control = (double) getAvgReading();
  long rgbArr[3] = {0};
  dbprintf("white: %lu", control);
  
  // Cycle between red green and blue
  setColour(GREEN);
  rgbArr[0] = getAvgReading();
  

  setColour(BLUE);
  rgbArr[1] = getAvgReading();
  
  
  setColour(RED);
  rgbArr[2] = getAvgReading();
  dbprintf("Green raw: %ld", rgbArr[0]);
  dbprintf("Blue raw: %ld", rgbArr[1]);
  dbprintf("Red raw: %ld",rgbArr[2]);
  
  // Turn off colour sensor
  PORTB &= ~(COLOUR_LED);
  
}
