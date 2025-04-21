#include "constants.h" 
#include "packet.h"

#define LEFT_CLAW (1 << PB4)
#define RIGHT_CLAW (1 << PL4)
#define DISPENSER (1 << PH6)

volatile int servoTimerTick = 0;
int claw_PWM;
int dispenser_PWM;

ISR(TIMER2_COMPA_vect) {
  servoTimerTick++;
  if (servoTimerTick >= claw_PWM) {
      PORTB &= ~(1 << PB4);
  } 
  if (servoTimerTick >= dispenser_PWM) {
    PORTH &= ~DISPENSER;
  }
  if (servoTimerTick >= 200) {
      servoTimerTick = 0;
      PORTH |= DISPENSER;
      PORTB |= LEFT_CLAW;
    }
}

void openservo() {
  OCR5B = 1500;
  claw_PWM = 15;

}

void closeservo() {
  OCR5B = 2100;
  claw_PWM = 9;
}

void dispense() {
  if (dispenser_PWM == 11) dispenser_PWM = 22;
  else dispenser_PWM = 11;
}

void setupservo() {
  // set servo pins to output 
  DDRL |= RIGHT_CLAW;
  DDRB |= LEFT_CLAW;
  DDRH |= DISPENSER;
  
  //setup pwm
  TCCR5A = 0b00100010;
  TCNT5 = 0;
  TCCR5B = 0b00010010;
  ICR5 = 40000;
  OCR5B = 1500;

  TCCR2A = 0b00000010;
  TCCR2B = 0b00000100;
  TIMSK2 = 0b10;
  OCR2A = 24;
  OCR2B = 24;
  TCNT2 = 0;
  claw_PWM = 15;
  dispenser_PWM = 11;
  
}
