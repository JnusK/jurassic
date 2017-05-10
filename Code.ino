#define LEDPin 13
int photoTran = A16;
int reading = 0;
int echoUSF = 16;
int trigUSF = 17;
int echoUSB = 18;
int trigUSB = 19;
int echoUSR = 20;
int trigUSR = 21;
int echoUSL = 22;
int trigUSL = 23;


int pwmPinL = 2;
int pwmPinR = 3;
int dirPinL = 27;
int dirPinR = 28;
int coastPinR = 24;
int coastPinL = 25;
int coastTrigger = 26;
bool coastP = false;
bool dirForward = true;
//speed can range from 0 - 255
float wheelSpeed = 255;
int i;


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
  
  Serial.begin(9600);
}
/*
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
        accelerate(dirForward);
      }
    } else {
      wheelSpeed = slowDown();
    }
  }
  delay(100);
}
*/
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

void loop(){
  //Serial.println(ultrasound(17,18));
  Serial.println(ultrasound(echoUSF,trigUSF));
}


