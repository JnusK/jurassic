#define LEDPin 13
int photoTran = 35; //A16
int reading = 0;
int echoUSF = 16;
int trigUSF = 17;
int echoUSB = 18;
int trigUSB = 19;
int echoUSR = 20;
int trigUSR = 21;
int echoUSL = 22;
int trigUSL = 23;

int lR = -1;

int pwmPinL = 2;
int pwmPinR = 3;
int dirPinL = 27;
int dirPinR = 28;
int coastPinR = 24;
int coastPinL = 25;
int coastTrigger = 26;

bool coastP = true;
bool dirForward = true;
//speed can range from 0 - 255
float wheelSpeed = 255;
int i;

int lastDist = -1;
int lastChange = -1;
int prevDistF = -1;
int currDistF, currDistB, currDistR, currDistL = 0;

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
  if (duration != 0 && distance < 10){
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
  randomSeed(analogRead(0));
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
  
  currDistF = ultrasound(echoUSF, trigUSF);
  currDistB = ultrasound(echoUSB, trigUSB);
  currDistR = ultrasound(echoUSR, trigUSR);
  currDistL = ultrasound(echoUSL, trigUSL);
  if (currDistF > 1000 || currDistR > 1000 || currDistL > 1000 || currDistB > 1000) {return;}
  if (currDistF < 15 && currDistF != 0){
    if (wheelSpeed == 0) {
      if (currDistR < 15 && currDistR != 0) {
        while (!differentiate('R', false)){
          aboutTurnL();
        }
      } else if (currDistL < 15 && currDistL !=0) {
        while (!differentiate('L', false)){
          aboutTurnR();
        }
      } else {
        lR = random(300) % 2;
        if (lR == 0) {
          aboutTurnL();
        } else {
          aboutTurnR();
        }
        delay (500/3 );
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
void accelerate(bool forward) {
  if (wheelSpeed < 255 ) {
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

bool differentiate (char side, bool maximum){
  switch (side) {
    case 'r':
    case 'R':
      {
        if (lastDist == -1) {
          lastDist = ultrasound(echoUSR, trigUSR);
        } else if (lastChange == -1 ) {
          lastChange = ultrasound(echoUSR, trigUSR) - lastDist;
        } else {
          if (maximum) {
            return ((ultrasound(echoUSR, trigUSR) - lastDist) <= 0 && lastChange >= 0);
          } else {
            return ((ultrasound(echoUSR, trigUSR) - lastDist) >=0 && lastChange <= 0);
          }
        }
      }
      break;
    case 'l':
    case 'L' :
      {
        if (lastDist == -1) {
          lastDist = ultrasound(echoUSL, trigUSL);
        } else if (lastChange == -1 ) {
          lastChange = ultrasound(echoUSL, trigUSL) - lastDist;
        } else {
          if (maximum) {
            return ((ultrasound(echoUSL, trigUSL) - lastDist) <= 0 && lastChange >= 0);
          } else {
            return ((ultrasound(echoUSL, trigUSL) - lastDist) >=0 && lastChange <= 0);
          }
        }
      }
      break;
    default: 
    break;
 } 
}

void aboutTurnL() {
    moveForwardR(255);
    reverseL(255);
} 
  
//void loop(){
//  Serial.println(ultrasound(echoUSF,trigUSF));
//}

