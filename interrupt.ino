#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPin 13
int intF = 36;
int intB = 37;
int intL = 38;
int intR = 39;
/*
int pinF = 26;
int pinB = 25;
int pinL = 24;
int pinR = 28;
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(intF, INPUT);
  pinMode(intB, INPUT);
  pinMode(intL, INPUT);
  pinMode(intR, INPUT);
  /*
  pinMode (pinF,OUTPUT);
  pinMode (pinB,OUTPUT);
  pinMode (pinL,OUTPUT);
  pinMode (pinR,OUTPUT);
  */
  pinMode (LEDPin,OUTPUT);
  
  attachInterrupt(intF, ISRF, RISING);
  attachInterrupt(intB, ISRB, RISING);
  attachInterrupt(intL, ISRL, RISING);
  attachInterrupt(intR, ISRR, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(LEDPin, HIGH);
  delay(100);
  //digitalWrite(pinF, HIGH);
  delay(100);
  digitalWrite(LEDPin, HIGH);
  delay(100);
  //digitalWrite(pinB, HIGH);
  delay(100);
  digitalWrite(LEDPin, HIGH);
  delay(100);
  //digitalWrite(pinL, HIGH);
  delay(100);
  digitalWrite(LEDPin, HIGH);
  delay(100);
  //digitalWrite(pinR, HIGH);
  delay(100);  
}

void ISRF(){
  digitalWrite(LEDPin, LOW);
  //digitalWrite(pinF, LOW);
}

void ISRB(){
  digitalWrite(LEDPin, LOW);
  //digitalWrite(pinB, LOW);
}

void ISRL(){
  digitalWrite(LEDPin, LOW);
  //digitalWrite(pinL, LOW);
}

void ISRR(){
  digitalWrite(LEDPin, LOW);
  //digitalWrite(pinR, LOW);
}

