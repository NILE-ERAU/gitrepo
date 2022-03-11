#include "temp_i2c.h"
#include "adc_i2c.h"
#include "quad_enc.h"
#include <Wire.h>

TempI2C TEMP(0x48);
ADCI2C ADC1(0x28);
QuadEnc TrolleyQuad(3,23);

#define ROT_COUNT 4096

int t_count = 0;

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {    
  t_count += TrolleyQuad.count();
 }
 
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
 {

 }  
 
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {
    
 }  

 
void setup() {
  // put your setup code here, to run once:
  cli();
  PCICR |= 0b00000111;    // turn on all ports
  PCMSK0 |= 0b00010000; // PCINT0
  sei();

  Serial.begin(9600);
  //Serial.println("Starting...");
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  
  
  TrolleyQuad.init();
  TEMP.init();
  ADC1.init();
}

int desired[4] = {1, 3, -2, 0};
int i = 0;


void loop() {
  // put your main code here, to run repeatedly:
  double val = TEMP.temp();
  int err = t_count - desired[i]*ROT_COUNT;
  int adc0 = ADC1.read(0);
  int adc1 = ADC1.read(1);
  int adc2 = ADC1.read(2);
  //Serial.print("Temp: ");
  //Serial.println(val);
  
  //Serial.print(" ");
  //Serial.print(adc0);
  //Serial.print(" ");
  //Serial.print(adc1);
  //Serial.print(" ");
  //Serial.println(adc2);

  Serial.println(t_count);
  if (abs(err) < 50) {
    i++;
    if (i > 3) {i = 0;}
  }

  uint8_t control = min((abs(err))/15,255);
  if (err >= 0) {
    analogWrite(5,control);
    analogWrite(6,0);
  } else {
    analogWrite(5,0);
    analogWrite(6,control);
  }

  delay(1);
}
