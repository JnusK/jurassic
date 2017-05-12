#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPin 13
const int intF = 36;
const int intB = 37;
const int intL = 38;
const int intR = 39;
const int buttonR = 5;
const int buttonL = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(intF, INPUT);
  pinMode(intB, INPUT);
  pinMode(intL, INPUT);
  pinMode(intR, INPUT);
  pinMode(buttonR, INPUT);
  pinMode(buttonL, INPUT);
  pinMode (LEDPin,OUTPUT);
  
  attachInterrupt(intF, ISRF, RISING);
  attachInterrupt(intB, ISRB, RISING);
  attachInterrupt(intL, ISRL, RISING);
  attachInterrupt(intR, ISRR, RISING);
  attachInterrupt(buttonR, ISRbuttonR, RISING);
  attachInterrupt(buttonL, ISRbuttonL, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(LEDPin, HIGH);
  delay(100);
}

void ISRF(){
  digitalWrite(LEDPin, LOW);
}

void ISRB(){
  digitalWrite(LEDPin, LOW);
}

void ISRL(){
  digitalWrite(LEDPin, LOW);
}

void ISRR(){
  digitalWrite(LEDPin, LOW);
}

void ISRbuttonR(){
  digitalWrite(LEDPin, LOW);
}

void ISRbuttonL(){
  digitalWrite(LEDPin, LOW);
}

