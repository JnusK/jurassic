#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPin 13
const int intF = 39;
const int intB = 37;
const int intL = 38;
const int intR = 36;
const int buttonR = 5;
const int buttonL = 4;

const int pin = 33;

void setup() {
  // put your setup code here, to run once:
  pinMode(intF, INPUT);
  pinMode(intB, INPUT);
  pinMode(intL, INPUT);
  pinMode(intR, INPUT);
  pinMode(buttonR, INPUT);
  pinMode(buttonL, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(pin, OUTPUT);
  
  
  attachInterrupt(intF, ISR, RISING);
  attachInterrupt(intB, ISR, RISING);
  attachInterrupt(intL, ISR, RISING);
  attachInterrupt(intR, ISR, RISING);
  attachInterrupt(buttonR, ISRbuttonR, RISING);
  attachInterrupt(buttonL, ISRbuttonL, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //digitalWrite(LEDPin, HIGH);
  digitalWrite(pin, HIGH);
  delay(5000);
  digitalWrite(pin, LOW);
  digitalWrite(LEDPin, HIGH);
  delay(5000);
}

void ISR(){
  digitalWrite(LEDPin, LOW);
  Serial.println("Triggered");
  //digitalWrite(pin, LOW);
}

void ISRF(){
  digitalWrite(LEDPin, LOW);
  digitalWrite(pin, LOW);
}

void ISRB(){
  digitalWrite(LEDPin, LOW);
  digitalWrite(pin, LOW);
}

void ISRL(){
  digitalWrite(LEDPin, LOW);
  digitalWrite(pin, LOW);
}

void ISRR(){
  digitalWrite(LEDPin, LOW);
  digitalWrite(pin, LOW);
}

void ISRbuttonR(){
  digitalWrite(LEDPin, LOW);
}

void ISRbuttonL(){
  digitalWrite(LEDPin, LOW);
}

