#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <elapsedMillis.h>

elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *xMotor = AFMS.getStepper(400, 1);
Adafruit_StepperMotor *yMotor = AFMS.getStepper(400, 2);

bool xDir;
bool yDir;

bool xJoyCW;
bool xJoyCCW;
bool yJoyCW;
bool yJoyCCW;
bool mSet1;
bool mSet2;
bool mGoTo1;
bool mGoTo2;

long positions[2];
long xSet1 = 0;
long ySet1 = 0;
long xSet2 = 0;
long ySet2 = 0;

volatile long actualYPos = 0;
volatile long actualXPos = 0;

long positionY2reach;
long positionX2reach;
long newYPosition;
long newXPosition;
long constrainYDistance;
long constrainXDistance;
long travelledYDistance;
long travelledXDistance;
long distanceToYPos;
long distanceToXPos;
float totalYDistance;
float totalXDistance;

long recYPos1;
long recXPos1;
long recYPos2;
long recXPos2;
long recYPos3;
long recXPos3;
long recYPos4;
long recXPos4;
long recYPos5;
long recXPos5;

int oldinByte;

unsigned long previousMillis = 0;
unsigned long currentMillis;

const int minY = 60;
const int maxY = 150;

const int minX = 60;
const int maxX = 150;

const int constrainedLimit = 120;         // how many steps does the acceleration last
const int startingYSpeed = 30;            // Y accel start speed
const int startingXSpeed = 30;            // X accel start speed

//const int myArray[30]={0,1,1,2,3,4,5,6,7,8,10,11,12,14,15,16,18,19,20,22,23,24,25,26,27,28,29,29,30};    // half sinewave (30 p-p)
//const int myArray[30]={0,0,0,1,1,1,2,3,3,4,5,6,7,8,9,10,11,12,14,15,16,18,19,21,22,24,25,27,28,30};      // quarter sinewave (60 p-p)
//const int myArray[30]={0,0,0,0,1,1,1,1,2,2,2,3,3,4,4,5,6,6,7,8,8,9,10,10,11,12,13,13,14,15};             // quarter sinewave (30 p-p)
int logAccel;

int Yread;
int Xread;
int err;
int e2;

boolean autoPos;
boolean motorYDir;
boolean motorXDir;
boolean newMove = false;
boolean buttonPressed = false;
boolean joyYCentral = false;
boolean joyXCentral = false;
boolean xLeading;

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  //TWBR = ((F_CPU /400000l) - 16) / 2;               // Change the i2c clock to 400KHz

  pinMode (2, INPUT_PULLUP);
  pinMode (3, INPUT_PULLUP);
  pinMode (4, INPUT_PULLUP);
  pinMode (5, INPUT_PULLUP);
  pinMode (6, INPUT_PULLUP);
  pinMode (7, INPUT_PULLUP);
  pinMode (8, INPUT_PULLUP);
  pinMode (9, INPUT_PULLUP);
}

void loop() {
  xJoyCW = digitalRead(2);
  xJoyCCW = digitalRead(3);
  yJoyCW = digitalRead(4);
  yJoyCCW = digitalRead(5);
  mSet1 = digitalRead(6);
  mSet2 = digitalRead(7);
  mGoTo1 = digitalRead(8);
  mGoTo2 = digitalRead(9);

  if (!xJoyCW) {
    autoPos = false;
    xMotor->onestep(FORWARD, DOUBLE);
    actualXPos++;
  }
  if (!xJoyCCW) {
    autoPos = false;
    xMotor->onestep(BACKWARD, DOUBLE);
    actualXPos--;
  }
  if (!yJoyCW) {
    autoPos = false;
    yMotor->onestep(FORWARD, DOUBLE);
    actualYPos++;
  }
  if (!yJoyCCW) {
    autoPos = false;
    yMotor->onestep(BACKWARD, DOUBLE);
    actualYPos--;
  }
  if (!mSet1) {
    Serial.println("Record Pos1");
    recYPos1 = actualYPos;
    recXPos1 = actualXPos;
    Serial.println(actualYPos);
    Serial.println(actualXPos);
  }
  if (!mSet2) {
    Serial.println("Record Pos2");
    recYPos2 = actualYPos;
    recXPos2 = actualXPos;
    Serial.println(actualYPos);
    Serial.println(actualXPos);
  }
  if (!mGoTo1 && !buttonPressed) {
    travelledXDistance = 0;
    Serial.println("Go To Pos1");
    newYPosition = recYPos1;              
    newXPosition = recXPos1;
    Serial.print("x: ");
    Serial.print(newXPosition);
    Serial.print(", y: ");
    Serial.println(newYPosition);
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (!mGoTo2 && !buttonPressed) {
    travelledXDistance = 0;
    Serial.println("Go To Pos2");
    newYPosition = recYPos2;
    newXPosition = recXPos2;
    Serial.print("x: ");
    Serial.print(newXPosition);
    Serial.print(", y: ");
    Serial.println(newYPosition);
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (autoPos) {
    goToPosition(newYPosition, newXPosition);
  }
}

void goToPosition(long positionY2reach, long positionX2reach) {
  if (newMove) {
    travelledXDistance = 0;
    travelledYDistance = 0;
    totalXDistance = abs(actualXPos - positionX2reach);
    totalYDistance = abs(actualYPos - positionY2reach);

    if (totalXDistance > totalYDistance) {
      xLeading = true;
    }
    if (totalYDistance > totalXDistance) {
      xLeading = false;
    }
    err = (totalXDistance>totalYDistance ? totalXDistance : -totalYDistance)/2;
    newMove = false;
  }

  if (actualYPos > positionY2reach) {
    motorYDir = false;
  }
  else if (actualYPos < positionY2reach) {
    motorYDir = true;
  }
  
  if (actualXPos > positionX2reach) {
    motorXDir = false;
  }
  else if (actualXPos < positionX2reach) {
    motorXDir = true;
  }
  
  if (xLeading) {
    if (actualXPos != positionX2reach) {
      e2 = err;
      distanceToYPos = abs(actualYPos - positionY2reach);
      distanceToXPos = abs(actualXPos - positionX2reach);
      travelledXDistance = totalXDistance-distanceToXPos;
      if (travelledXDistance <= distanceToXPos) {
        constrainXDistance = constrain(travelledXDistance, 0, constrainedLimit);
        int stepperAccel = map(constrainXDistance, 0, constrainedLimit, startingXSpeed, 0);
        //logAccel = myArray[stepperAccel];
        if (timeElapsed >= stepperAccel) {
          if (e2 >-distanceToXPos) {
            err -= distanceToYPos;
            if (motorXDir) {
              xMotor->step(1, FORWARD, DOUBLE);
              actualXPos++;
            }
            else {
              xMotor->step(1, BACKWARD, DOUBLE);
              actualXPos--;
            }
          }
          
          if (e2 < distanceToYPos) {
            err += distanceToXPos;
            if (motorYDir) {
              yMotor->step(1, FORWARD, DOUBLE);
              actualYPos++;
            }
            else {
              yMotor->step(1, BACKWARD, DOUBLE);
              actualYPos--;
            }
          }
          timeElapsed = 0;
          buttonPressed = false;
        }
      }
      if (travelledXDistance > distanceToXPos) {
        constrainXDistance = constrain(distanceToXPos, 0, constrainedLimit);
        int stepperAccel = map(constrainXDistance, 0, constrainedLimit, startingXSpeed, 0);
        //logAccel = myArray[stepperAccel];
        if (timeElapsed >= stepperAccel) {
          if (e2 >-distanceToXPos) {
            err -= distanceToYPos;
            if (motorXDir) {
              xMotor->step(1, FORWARD, DOUBLE);
              actualXPos++;
            }
            else {
              xMotor->step(1, BACKWARD, DOUBLE);
              actualXPos--;
            }
          }
          
          if (e2 < distanceToYPos) {
            err += distanceToXPos;
            if (motorYDir) {
              yMotor->step(1, FORWARD, DOUBLE);
              actualYPos++;
            }
            else {
              yMotor->step(1, BACKWARD, DOUBLE);
              actualYPos--;
            }
          }
          timeElapsed = 0;
          buttonPressed = false;
        }
      }
    }
  }

  if (!xLeading) {
    if (actualYPos != positionY2reach) {
      e2 = err;
      distanceToYPos = abs(actualYPos - positionY2reach);
      distanceToXPos = abs(actualXPos - positionX2reach);
      travelledYDistance = totalYDistance-distanceToYPos;
      if (travelledYDistance <= distanceToYPos) {
        constrainYDistance = constrain(travelledYDistance, 0, constrainedLimit);
        int stepperAccel = map(constrainYDistance, 0, constrainedLimit, startingYSpeed, 0);
        //logAccel = myArray[stepperAccel];
        if (timeElapsed >= stepperAccel) {
          if (e2 >-distanceToXPos) {
            err -= distanceToYPos;
            if (motorXDir) {
              xMotor->step(1, FORWARD, DOUBLE);
              actualXPos++;
            }
            else {
              xMotor->step(1, BACKWARD, DOUBLE);
              actualXPos--;
            }
          }
          
          if (e2 < distanceToYPos) {
            err += distanceToXPos;
            if (motorYDir) {
              yMotor->step(1, FORWARD, DOUBLE);
              actualYPos++;
            }
            else {
              yMotor->step(1, BACKWARD, DOUBLE);
              actualYPos--;
            }
          }
          timeElapsed = 0;
          buttonPressed = false;
        }
      }
      if (travelledYDistance > distanceToYPos) {
        constrainYDistance = constrain(distanceToYPos, 0, constrainedLimit);
        int stepperAccel = map(constrainYDistance, 0, constrainedLimit, startingYSpeed, 0);
        //logAccel = myArray[stepperAccel];
        if (timeElapsed >= stepperAccel) {
          if (e2 >-distanceToXPos) {
            err -= distanceToYPos;
            if (motorXDir) {
              xMotor->step(1, FORWARD, DOUBLE);
              actualXPos++;
            }
            else {
              xMotor->step(1, BACKWARD, DOUBLE);
              actualXPos--;
            }
          }
          
          if (e2 < distanceToYPos) {
            err += distanceToXPos;
            if (motorYDir) {
              yMotor->step(1, FORWARD, DOUBLE);
              actualYPos++;
            }
            else {
              yMotor->step(1, BACKWARD, DOUBLE);
              actualYPos--;
            }
          }
          timeElapsed = 0;
          buttonPressed = false;
        }
      }
    }
  }
  if (actualXPos==positionX2reach && actualYPos==positionY2reach) {
    xMotor->release();
    yMotor->release();
    autoPos = false;
    buttonPressed = false;
  }
}
