#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDPin 13

int photoTran = 35; //A16
int reading = 0;
int echoUSF = 15;
int trigUSF = 16;
int echoUSB = 21;
int trigUSB = 22;
int echoUSR = 19;
int trigUSR = 20;
int echoUSL = 17;
int trigUSL = 18;

int lR = 0;

int pwmPinL = 29;
int pwmPinR = 30;
int dirPinL = 27;
int dirPinR = 28;
int coastPinR = 25;
int coastPinL = 24;
int coastTrigger = 23;

bool coastP = true;
bool dirForward = true;
//speed can range from 0 - 255
float wheelSpeed = 255;

int lastDist = -1;
int lastChange = -1;
int prevDistF = -1;

int buttonR = 5;
int buttonL = 4;
volatile bool evasivep;

int currDistF, currDistB, currDistR, currDistL, newDistF, newDistB, newDistR, newDistL = 0;

int clearDist = 20;
int ninetyDegDelay = 800;
int fortyfiveDegDelay = 600;

int ultrasound(int echo, int trig){
  long duration, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  digitalWrite(LEDPin, HIGH);
  duration = pulseIn(echo, HIGH);
  distance = (duration/2)/29.1;
  //distance = (duration/2)/58;
  if (duration != 0 && distance < 20){
    digitalWrite(LEDPin, LOW);
  }
  Serial.println(distance);
  return distance;
}

int phototrans(){
  reading = analogRead(photoTran);
  return reading;
}

//int LED = 25;
void setup(){
  pinMode (photoTran,INPUT);
  pinMode (echoUSF, INPUT);
  pinMode (trigUSF, OUTPUT);
  pinMode (echoUSB, INPUT);
  pinMode (trigUSB, OUTPUT);
  pinMode (echoUSR, INPUT);
  pinMode (trigUSR, OUTPUT);
  pinMode (echoUSL, INPUT);
  pinMode (trigUSL, OUTPUT);
  pinMode (LEDPin, OUTPUT);
  
  pinMode(pwmPinL, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(coastPinR, OUTPUT);
  pinMode(coastPinL, OUTPUT);
  pinMode(coastTrigger, INPUT);

  pinMode(buttonR, INPUT);
  pinMode(buttonL, INPUT);
  attachInterrupt(buttonR, buttonRCallback, RISING);
  attachInterrupt(buttonL, buttonLCallback, RISING);
  randomSeed(analogRead(0));

  Serial.begin(9600);
}

void loop(){
  //brake button
  if (digitalRead(coastTrigger)) {
    if (coastP) {
      setSpd(0);
      digitalWrite(coastPinR, LOW);
      digitalWrite(coastPinL, LOW);
      coastP = false;
    } else {
      stopMove();
    }
    delay(100);
  }

  //normal operation
  newDistF = ultrasound(echoUSF,trigUSF);
  newDistB = ultrasound(echoUSB, trigUSB);
//  newDistR = ultrasound(echoUSR, trigUSR);
//  newDistL = ultrasound(echoUSL, trigUSL);
  if (newDistF != 0 && newDistF < 1000){
    currDistF = newDistF;
  }
    if (newDistB != 0 && newDistB < 1000){
    currDistB = newDistB;
  }
    if (newDistR != 0 && newDistR < 1000){
    currDistR = newDistR;
  }
    if (newDistL != 0 && newDistL < 1000){
    currDistL = newDistL;
  }

//random movement
//changes direction after every random motion, including when the bot hits a wall with both sides clear
/*  if (random(100) == 88 ) {
    if (lR ==  0){
      if (clearLp) {
        aboutTurnL(ninetyDegDelay);
        lR = 1;
      } else if (clearRp) {
        aboutTurnR(ninetyDegDelay);
      }
    } else if (clearRp) {
      aboutTurnR(ninetyDegDelay);
      lR = 0;
    } else {
      aboutTurnL(ninetyDegDelay);
    }
  }*/
  
// detect collision
  if (currDistF < 25 && currDistF != 0){
    if (wheelSpeed == 0) {
      if (!clearRp) {
        aboutTurnL(ninetyDegDelay);
      } else if (!clearLp) {
        aboutTurnR(ninetyDegDelay);
      } else {
        lR = random(300) % 2;
        if (lR == 0) {
          aboutTurnL(ninetyDegDelay);
        } else {
          aboutTurnR(ninetyDegDelay);
        }
      }
    } else {
      wheelSpeed = slowDown(dirForward);
    }
  } else {
    accelerate(dirForward);
  }
  delay(100);
}

int slowDown(bool dirForward) {
  if (wheelSpeed > 0) {
    int newSpeed = wheelSpeed - 50;
    if (newSpeed <= 10) {
      moveWheel(0, dirForward);
      newSpeed = 0;
    } else {
      moveWheel(newSpeed , dirForward);
    }
    return newSpeed;
  } else {
    return 0;
  }
}

bool clearRp() {
  return (currDistR > 15);
}

bool clearLp() {
  return (currDistL > 15);
}

void accelerate(bool forward) {
  if (wheelSpeed < 200 ) {
   wheelSpeed = wheelSpeed + 30;   
  }
  moveWheel(wheelSpeed, forward);
}

void moveWheel(float val, bool forward) {
  if (forward) {
    moveForward(val);
  } else {
    reverse(val);
  }
}

void aboutTurnR (int delaySet){
  moveForwardL(125);
  reverseR(125);
  delay (delaySet);
}

void moveForward (float val) {
  moveForwardR(val);
  moveForwardL(val);
}

void moveForwardR (float val) {
  analogWrite(pwmPinR, val);
  digitalWrite(dirPinR, HIGH);
}

void moveForwardL (float val) {
  analogWrite(pwmPinL, val);
  digitalWrite(dirPinL, LOW);
}

void stopMove() {
  digitalWrite(coastPinR, HIGH);
  digitalWrite(coastPinL, HIGH);
  setSpd(0);
  coastP = true;
}

void reverseL (float val) {
  analogWrite(pwmPinL, val);
  digitalWrite(dirPinL, HIGH);
}

void reverseR (float val) {
  analogWrite(pwmPinR, val);
  digitalWrite(dirPinR, LOW);
}

void reverse(float val) {
  reverseL(val);
  reverseR(val);
}

void setSpd(float val) {
  analogWrite(pwmPinR, val);
  analogWrite(pwmPinL, val);
  wheelSpeed = val;
}

//bool differentiate (char side, bool maximum){
//  switch (side) {
//    case 'r':
//    case 'R':
//      {
//        if (lastDist == -1) {
//          lastDist = ultrasound(echoUSR, trigUSR);
//        } else if (lastChange == -1 ) {
//          lastChange = ultrasound(echoUSR, trigUSR) - lastDist;
//        } else {
//          if (maximum) {
//            return ((ultrasound(echoUSR, trigUSR) - lastDist) <= 0 && lastChange >= 0);
//          } else {
//            return ((ultrasound(echoUSR, trigUSR) - lastDist) >=0 && lastChange <= 0);
//          }
//        }
//      }
//      break;
//    case 'l':
//    case 'L' :
//      {
//        if (lastDist == -1) {
//          lastDist = ultrasound(echoUSL, trigUSL);
//        } else if (lastChange == -1 ) {
//          lastChange = ultrasound(echoUSL, trigUSL) - lastDist;
//        } else {
//          if (maximum) {
//            return ((ultrasound(echoUSL, trigUSL) - lastDist) <= 0 && lastChange >= 0);
//          } else {
//            return ((ultrasound(echoUSL, trigUSL) - lastDist) > 0 && lastChange < 0);
//          }
//        }
//      }
//      break;
//    default: 
//    break;
// } 
//}

void aboutTurnL(int delaySet) {
    moveForwardR(125);
    reverseL(125);
    delay (delaySet);
} 

void buttonRCallback () {
  if (digitalRead(buttonL) == LOW && digitalRead(buttonR) == HIGH) { 
    delay(100);
    if (digitalRead(buttonR) == HIGH) {
      setSpd(0);
      delay(100);
      reverse(100);
      delay(100);
      aboutTurnL(fortyfiveDegDelay);
    }
  }
}

void buttonLCallback () {
  if (digitalRead(buttonR) == LOW && digitalRead(buttonL) == HIGH) {
    delay(100);
    if (digitalRead(buttonL) == HIGH) {
      setSpd(0);
      delay(100);
      reverse(100);
      delay(100);
      aboutTurnR(fortyfiveDegDelay);
    }
  }
}

