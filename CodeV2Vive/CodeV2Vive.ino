#include <avr/interrupt.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDPin 13
#define V1PIN 9 // the signal from the front sensor
#define V2PIN 8 // the signat from the back sensor
#define DEG_PER_US 0.0216 // equal to (180 deg) / (8333 us)
#define LIGHTHOUSEHEIGHT 6.0 // in feet
#define viveBuf 0.1

// structure to store the sensor data
typedef struct {
  unsigned long changeTime[11];
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;

// variables for the sensor data and filter
volatile viveSensor V1;
volatile viveSensor V2;
int state = 0;
double xPos1, yPos1;
double xPos2, yPos2;
double xOld1 = 0, yOld1 = 0, xFilt1 = 0, yFilt1 = 0;
double xOld2 = 0, yOld2 = 0, xFilt2 = 0, yFilt2 = 0;
double xNew1 = 0, xNew2 = 0, yNew1 = 0, yNew2 = 0;

// variables for the xbee data
char msg[100];
int msg_index = 0;
float xOpponent, yOpponent;

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
float wheelSpeed = 200;
float RFactor = 0.8;

int buttonR = 5;
int buttonL = 4;
volatile bool evasivep;

int currDistF, currDistB, currDistR, currDistL, newDistF, newDistB, newDistR, newDistL = 0;

int ninetyDegDelay = 800;
int fortyfiveDegDelay = 300;

int selfCurrPos, oppoCurrPos, safePosition;
float xNewOpponent, yNewOpponent;
float xOldOpponent, yOldOpponent;

double alignedX = -1;
double alignedY = -1; 
int nextSafeSpot = -1;
bool safeP = true;
bool alignXP = true;
bool aligningXP = false;
bool aligningYP = false;
int one = 0, two = 0, three = 0;
int sentData = 0;

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
//  Serial.println(distance);
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
  Serial3.begin(9600);
  pinMode(13, OUTPUT); // to blink the led on pin 13
  pinMode(V1PIN, INPUT); // to read the front sensor
  pinMode(V2PIN, INPUT); // to read the back sensor

  // initialize the sensor variables
  V1.horzAng = 0;
  V1.vertAng = 0;
  V1.useMe = 0;
  V1.collected = 0;

  V2.horzAng = 0;
  V2.vertAng = 0;
  V2.useMe = 0;
  V2.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V1PIN), ISRV1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(V2PIN), ISRV2, CHANGE);
  aligningXP = true;
}

void loop(){

  //data collection
  newDistF = ultrasound(echoUSF,trigUSF);
  if (newDistF != 0 && newDistF < 1000){
    currDistF = newDistF;
  }
  newDistB = ultrasound(echoUSB, trigUSB);
  newDistR = ultrasound(echoUSR, trigUSR);
  newDistL = ultrasound(echoUSL, trigUSL);
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
  
//  
  while (!sentData){
  if (Serial3.available() > 0) {
    msg[msg_index] = Serial3.read();
    // if you get a newline, there is data to read
    if (msg[msg_index] == '\n') {
      msg_index = 0;
      // data is in the format of two floats seperated by spaces
      sscanf(msg, "%f %f", &xOpponent, &yOpponent);

//      Serial.print("op: ");
//      Serial.print(xOpponent);
//      Serial.print(" ");
//      Serial.println(yOpponent);
      sentData = 1;
    }
    else {
      // did not get a newline yet, just store for later
      msg_index++;
      if (msg_index == 100) {
        msg_index = 0;
      }
    }
  }
  }
  sentData = 0;
  //mapping
  selfCurrPos = currRegion( xFilt1, yFilt1); 
  if (xOpponent > -10 && xOpponent < 10 && yOpponent > -10 && yOpponent < 10){
    xNewOpponent = 0.5*xOldOpponent + 0.5*xOpponent;
    yNewOpponent = 0.5*yOldOpponent + 0.5*yOpponent;
  }
  xOldOpponent = xNewOpponent;
  yOldOpponent = yNewOpponent;
//  Serial.print("op: ");
//  Serial.print(xNewOpponent);
//  Serial.print(" ");
//  Serial.println(yNewOpponent);
  oppoCurrPos = currRegion ( xNewOpponent, yNewOpponent);
  safePosition = safeSpot (xFilt1, yFilt1);
  if (selfCurrPos == 1) {
    if (safePosition == 11 && nextSafeSpot != 12 && nextSafeSpot != 14) {
      if (oppoCurrPos == 0) {
        xAlign();
        alignedY = 3.7;
        nextSafeSpot = 12;
        safeP = false;
      } else if (oppoCurrPos == 2) {
        yAlign();
        alignedX = -5.6;
        nextSafeSpot = 14;
        safeP = false;
      } else if (oppoCurrPos == 1) {
        if (lR == 0) {
          xAlign();
          alignedY = 3.7;
          nextSafeSpot = 12;
          } else {
            yAlign();
            alignedX = -5.6;
            nextSafeSpot = 14;
          }
          safeP = false;
        } else {
          safeP = true;
          nextSafeSpot = 11;
          setSpd(0);
        }
      }
  } else if (selfCurrPos == 4) {
    if (safePosition == 13 && nextSafeSpot != 12 && nextSafeSpot != 14) {
      if (oppoCurrPos == 3) {
          yAlign();
          alignedX = 3.6;
          nextSafeSpot = 12;
          safeP = false;
        } else if (oppoCurrPos == 5) {
          xAlign();
          alignedY = -5.5;
          nextSafeSpot = 14;
          safeP = false;
        } else if (oppoCurrPos == 4 ) {
          if (lR == 0) {
            xAlign();
            alignedY = -5.5;
            nextSafeSpot = 14;
          } else {
            yAlign();
            alignedX = 3.6;
            nextSafeSpot = 12;
          }
          safeP = false;
        } else {
          safeP = true;
          nextSafeSpot = 13;
          setSpd(0);
        }
    }
  }
  
  //State Machine
  if (safePosition == nextSafeSpot) {
    //find out where is enemy to determine new safespot or stay
    if (safePosition == 12){
      if (oppoCurrPos == 0 || oppoCurrPos == 1 || oppoCurrPos == 2) {
        yAlign();
        alignedX = 3.6;
        nextSafeSpot = 13;
      } else {
        xAlign();
        alignedY = 3.7;
        nextSafeSpot = 11;
      }
    } else if (safePosition == 14) {
      if (oppoCurrPos == 0 || oppoCurrPos == 1 || oppoCurrPos == 2) {
        xAlign();
        alignedY = -5.3;
        nextSafeSpot = 13;
      } else {
        yAlign();
        alignedX = -5.6;
        nextSafeSpot = 11;
      }
    }
  }
  Serial.print("SafePosition: ");
  Serial.println(safePosition);
  Serial.print("Current Region: ");
  Serial.println(selfCurrPos);
  Serial.print("Oppo Region: ");
  Serial.println(oppoCurrPos);
  Serial.print("Next Safe : ");
  Serial.println(nextSafeSpot);
  Serial.print("safe? : ");
  Serial.println(safeP);

   /// Vive and xbee code

  // if the sensor data is new
  if (V1.useMe == 1) {
    V1.useMe = 0;

    // calculate the position and filter it
    xPos1 = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    yPos1 = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    xNew1 = xOld1 * 0.5 + xPos1 * 0.5;
    if (xNew1 < 10 && xNew1 > -10){
      xFilt1 = xNew1;
    }
    yNew1 = yOld1 * 0.5 + yPos1 * 0.5;
    if (yNew1 < 10 && yNew1 > -10) {
      yFilt1 = yNew1;
    }
    xOld1 = xFilt1;
    yOld1 = yFilt1;
    /*Serial.print("Front: ");
    Serial.print(xFilt1);
    Serial.print(" ");
    Serial.println(yFilt1);*/

    // calculate the position and filter it
    xPos2 = tan((V2.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    yPos2 = tan((V2.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
     xNew2 = xOld2 * 0.5 + xPos2 * 0.5;
    if (xNew2 < 10 && xNew2 > -10){
      xFilt2 = xNew2;
    }
    yNew2 = yOld2 * 0.5 + yPos2 * 0.5;
    if (yNew2 < 10 && yNew2 > -10) {
      yFilt2 = yNew2;
    }
    xOld2 = xFilt2;
    yOld2 = yFilt2;
    
    /*Serial.print("Back: ");
    Serial.print(xFilt2);
    Serial.print(" ");
    Serial.println(yFilt2);
    */
  }
//end of Vive and xbee code

  //detect collision
  //this code MUST be the last block, it contaoins end condition
  
  if (currDistF < 15 && currDistF != 0){
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
    }
    else {
    if (aligningXP) {
      xAlign(); 
    } else if (aligningYP) {
      yAlign();
    } else if (safeP) {
      if (!viveEqual(yFilt1, yFilt2)) {
        xAlign();
      } else {
        setSpd(0);
      }
    } else {
      Serial.print("AlignedX: ");
      Serial.print(alignedX);
      Serial.println(xFilt1);
      Serial.print("AlignedY: ");
      Serial.print(alignedY);
      Serial.print(yFilt1);
      accelerate(dirForward);
    }
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
  if (safeP) {
    setSpd(0);
  } else{
    if (wheelSpeed < 200 ) {
      wheelSpeed = wheelSpeed + 30;
    }
    if (alignXP) {
      if (!viveEqual (yFilt1, alignedY)){
        if (xFilt1 > xFilt2) {
          if (yFilt1 > alignedY) {
            RFactor = 0.65;
          } else {
            RFactor = 0.95;
          }
        } else {
          if (yFilt1 > alignedY) {
            RFactor = 0.95;
          } else {
            RFactor = 0.65;
          }
        }
      } else {
        RFactor = 0.8;        
      }
    } else {
      if (!viveEqual (xFilt1, alignedX)){
        if (yFilt1 > yFilt2) {
          if (xFilt1 < alignedX) {
            RFactor = 0.65;
          } else {
            RFactor = 0.95;
          }
        }
        if (yFilt1 < yFilt2) {
          if (xFilt1 < alignedX) {
            RFactor = 0.95;
          } else {
            RFactor = 0.65;
          }
        }
      } else {
        RFactor = 0.8;
      }
    }
    moveWheel(wheelSpeed, forward);
  }
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
  analogWrite(pwmPinR, val*RFactor);
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
  analogWrite(pwmPinR, val*RFactor);
  analogWrite(pwmPinL, val);
  wheelSpeed = val;
}

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
      delay(500);
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
      delay(500);
      aboutTurnR(fortyfiveDegDelay);
    }
  }
}

int currRegion ( double x, double y) {
  if ( y > x ) {
    if ( x < 0 && y < 0) {
      return 0;
    }
    else if ( x > 0 && y > 0) {
      return 2;
    } else {
      return 1;
    }
  }
  else {
    if ( x < 0 && y < 0) {
      return 3;
    }
    else if ( x > 0 && y > 0) {
      return 5;
    } else {
      return 4;
    }
  }
}

//align to x-axis
double xAlign () {
  aligningXP = !viveEqual (yFilt1, yFilt2);
  if (xFilt1 < 0 && xFilt2 < 0) {
   if(yFilt1 > yFilt2) {
     moveForwardL(100);
     reverseR(100);
   } else {
     moveForwardR(100);
     reverseL(100);
   }
  } else {
   if (yFilt1 > yFilt2) {
     moveForwardR(100);
     reverseL(100);
   } else {
     moveForwardL(100);
     reverseR(100);
   }
  }
  alignXP = true;
  return yFilt1;
}

double yAlign () {
 aligningYP = !viveEqual (xFilt1, xFilt2 );
 if (yFilt1 < 0 && yFilt2 < 0) {
  if(xFilt1 < xFilt2) {
    moveForwardL(100);
    reverseR(100);
  } else {
    moveForwardR(100);
    reverseL(100);
  }
 } else {
  if (xFilt1 < xFilt2) {
    moveForwardR(100);
    reverseL(100);
  } else {
    moveForwardL(100);
    reverseR(100);
  }
 }
 
  alignXP = false;
  return xFilt1;
}


bool viveEqual (double x1, double x2) {
  return ( x1 < (x2 + viveBuf) && x1 > (x2 - viveBuf));
}

int safeSpot(double x, double y){
  if ( x < -5.2 && y > 3.3) {
    return 11;
  }
  if ( x > 3.1 && y > 3.5) {
    return 12;
  }
  if ( x > 3.3 && y < -4.5) {
    return 13;
  }
  if ( x < -5.2 && y < -5.5) {
    return 14;
  }
  return 0;
}

// the sensor interrupt
void ISRV1() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V1.changeTime[i] = V1.changeTime[i + 1];
  }
  V1.changeTime[10] = mic;

  // if the buffer is full
  if (V1.collected < 11) {
    V1.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V1.changeTime[1] - V1.changeTime[0] > 7000) && (V1.changeTime[3] - V1.changeTime[2] > 7000) && (V1.changeTime[6] - V1.changeTime[5] < 50) && (V1.changeTime[10] - V1.changeTime[9] < 50)) {
      V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_US;
      V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_US;
      V1.useMe = 1;
    }
  }
}

void ISRV2() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V2.changeTime[i] = V2.changeTime[i + 1];
  }
  V2.changeTime[10] = mic;

  // if the buffer is full
  if (V2.collected < 11) {
    V2.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V2.changeTime[1] - V2.changeTime[0] > 7000) && (V2.changeTime[3] - V2.changeTime[2] > 7000) && (V2.changeTime[6] - V2.changeTime[5] < 50) && (V2.changeTime[10] - V2.changeTime[9] < 50)) {
      V2.horzAng = (V2.changeTime[5] - V2.changeTime[4]) * DEG_PER_US;
      V2.vertAng = (V2.changeTime[9] - V2.changeTime[8]) * DEG_PER_US;
      V2.useMe = 1;
    }
  }
}
