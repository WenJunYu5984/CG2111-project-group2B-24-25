#include "constants.h"
#include "packet.h"

// Defining Ports used for the Colour Sensor (they are not all the same Port Data Register)
#define COLOUR_S0 (1 << 2) // Port L
#define COLOUR_S1 (1 << 1) // Port L
#define COLOUR_S2 (1 << 4) // Port A
#define COLOUR_S3 (1 << 6) // Port A
#define COLOUR_READER (1 << 5) // Port A
#define COLOUR_ON (1 << 7) // Port A
#define COLOUR_LED (1 << 0) // Port L
#define COLOUR_GROUND (1 << 3) // Port A

void initializeColourSensor() {
  // Set the reading pins to be input and the S pins and power source to be output pins
  DDRL |= COLOUR_S0 | COLOUR_S1 | COLOUR_LED;
  DDRA &= ~(COLOUR_READER);
  DDRA |= COLOUR_S2 | COLOUR_S3 | COLOUR_ON | COLOUR_GROUND;

  // Set the frequency of the colour sensor to be at 20%
  PORTL |= COLOUR_S1;
  PORTL &= ~(COLOUR_S0);

  PORTA |= COLOUR_ON;
  PORTA &= ~COLOUR_GROUND;

  // Turn off the colour sensor on start up
  PORTL &= ~(COLOUR_LED);
}

long getAvgReading() {
  long total = 0;
  for (int i = 0; i < 50; i++) {
    total += pulseIn(45, LOW);
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
  PORTL |= COLOUR_LED;
  delay(10);
  setColour(WHITE);
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
  dbprintf("Green: %ld", rgbArr[0]);
  dbprintf("Blue: %ld", rgbArr[1]);
  dbprintf("Red: %ld",rgbArr[2]);
  // Turn off colour sensor
  PORTL &= ~(COLOUR_LED);
  
}
