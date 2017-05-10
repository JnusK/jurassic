#define LEDPin 13
int photoTran = 35; //A16
int reading = 0;
int trigUSF = 8;
int echoUSF = 7;
int trigUSB = 9;
int echoUSB = 10;

int pwmPinL = 2;
int pwmPinR = 3;
int dirPinL = 11;
int dirPinR = 12;
int coastPinR = 24;
int coastPinL = 25;
int coastTrigger = 26;

bool coastP = true;
bool dirForward = true;
//speed can range from 0 - 255
float wheelSpeed = 255;
int i;

int lastDist;
int lastChange;

int ultrasoundF(){
  long duration, distance;
  digitalWrite(trigUSF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigUSF,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigUSF,LOW);
  digitalWrite(LEDPin, HIGH);
  duration = pulseIn(echoUSF, HIGH);
  distance = (duration/2)/29.1;
  //distance = (duration/2)/58;
  if (duration != 0){
    digitalWrite(LEDPin, LOW);
  }
  Serial.println(distance);
  return distance;
}

int ultrasoundB(){
  long duration, distance;

  digitalWrite(trigUSB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigUSB,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigUSB, LOW);
  digitalWrite(LEDPin, HIGH);
  duration = pulseIn(echoUSB, HIGH);
  distance = (duration/2)/29.1;
  //distance = (duration/2)/58;
  if (duration != 0){
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
  pinMode(photoTran,INPUT);
  pinMode (trigUSF, OUTPUT);
  pinMode (echoUSF, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode (trigUSB, OUTPUT);
  pinMode (echoUSB, INPUT);
  
  pinMode(pwmPinL, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(coastPinR, OUTPUT);
  pinMode(coastPinL, OUTPUT);
  pinMode(coastTrigger, INPUT);
  
  Serial.begin(9600);
}

void loop(){
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
  accelerate(dirForward);
  if (ultrasoundF() < 10 && ultrasoundF() != 0){
    if (wheelSpeed == 0) {
      dirForward = false;
      for (i=0; i<5; i++) {
        aboutTurnR();
      }
      for (i=0; i<10; i++) {
        accelerate(dirForward);
      }
    } else {
      wheelSpeed = slowDown();
    }
  }
  if (ultrasoundB() < 10 && ultrasoundB() != 0){
    if (wheelSpeed == 0) {
      dirForward = true;
      for (i=0; i<10; i++) {
        aboutTurn('l');
      }
    } else {
      wheelSpeed = slowDown();
    }
  }
  delay(100);
}

int slowDown() {
  if (wheelSpeed != 0) {
    return wheelSpeed - 7;
  }
}
void accelerate(bool forward) {
  if (wheelSpeed != 255 ) {
   wheelSpeed = wheelSpeed + 7;   
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

void aboutTurnR (){
  moveForwardL(255);
  reverseR(255);
}

void moveForward (float val) {
  moveForwardR(val);
  moveForwardL(val);
}

void moveForwardR (float val) {
  analogWrite(pwmPinR, val);
  digitalWrite(dirPinR, LOW);
}

void moveForwardL (float val) {
  analogWrite(pwmPinL, val);
  digitalWrite(dirPinL, HIGH);
}

void stopMove() {
  digitalWrite(coastPinR, HIGH);
  digitalWrite(coastPinL, HIGH);
  setSpd(0);
  coastP = true;
}

void reverseL (float val) {
  analogWrite(pwmPinL, val);
  digitalWrite(dirPinL, LOW);
}

void reverseR (float val) {
  analogWrite(pwmPinR, val);
  digitalWrite(dirPinR, HIGH);
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
//        if (lastDist == null) {
//          lastDist = ultrasoundR();
//        } else if (lastChange == null ) {
//          lastChange = ultrasoundR() - lastDist;
//        } else {
//          if (maximum) {
//            return ((ultrasoundR() - lastDist) <= 0 && lastChange >= 0);
//          } else {
//            return ((ultrasoundR() - lastDist) >=0 && lastChange <= 0);
//          }
//        }
//      }
//      break;
//    case 'l':
//    case 'L' :
//      {
//        if (lastDist == null) {
//          lastDist = ultrasoundL();
//        } else if (lastChange == null ) {
//          lastChange = ultrasoundL() - lastDist;
//        } else {
//          if (maximum) {
//            return ((ultrasoundL() - lastDist) <= 0 && lastChange >= 0);
//          } else {
//            return ((ultrasoundL() - lastDist) >=0 && lastChange <= 0);
//          }
//        }
//      }
//      break;
//    default: 
//    break;
//  } 
//}

void aboutTurn(char side) {
  switch (side) {
    case 'l':
    case 'L':
      moveForwardR(130);
      reverseL(130);
    break;
    case 'r':
    case'R':
      moveForwardL(130);
      reverseR(130);
    break;
    default: 
    break;
  }
}

