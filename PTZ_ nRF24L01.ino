#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <elapsedMillis.h>
#include <RF24.h>
#include <SPI.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *xMotor = AFMS.getStepper(400, 1);
Adafruit_StepperMotor *yMotor = AFMS.getStepper(400, 2);

elapsedMillis timeElapsed;
elapsedMillis timeElapsedX;
elapsedMillis timeElapsedY;

RF24 radio(48,53);

byte addresses[][6] = {"1Node", "2Node"};

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

volatile long actualXPos = 0;
volatile long actualYPos = 0;
volatile long actualZPos = 0;

long positionX2reach;
long positionY2reach;
long positionZ2reach;
long newXPosition;
long newYPosition;
long newZPosition;
long constrainXDistance;
long constrainYDistance;
long constrainZDistance;
long travelledXDistance;
long travelledYDistance;
long travelledZDistance;
long distanceToXPos;
long distanceToYPos;
long distanceToZPos;
float totalXDistance;
float totalYDistance;
float totalZDistance;

long recXPos1;
long recYPos1;
long recZPos1;
long recXPos2;
long recYPos2;
long recZPos2;
long recXPos3;
long recYPos3;
long recZPos3;
long recXPos4;
long recYPos4;
long recZPos4;
long recXPos5;
long recYPos5;
long recZPos5;

int oldinByte;

unsigned long previousMillis = 0;
unsigned long currentMillis;

const int minX = 60;
const int maxX = 150;
const int minY = 60;
const int maxY = 150;

const int constrainedLimit = 120;         // how many steps does the acceleration last
const int startingXSpeed = 30;            // X accel start speed
const int startingYSpeed = 30;            // Y accel start speed

int Xread;
int Yread;
int err;
int e2;

boolean autoPos;
boolean motorXDir;
boolean motorYDir;
boolean newMove = false;
boolean buttonPressed = false;
boolean joyXCentral = false;
boolean joyYCentral = false;
boolean xLeading;

struct dataStruct {
  //unsigned long _micros;                          // to save response times
  int Xposition = 12;                               // The Joystick position values
  int Yposition = 12;
  int buttonNo;                                     // The Joystick push-down switch
} inByte;

void setup() {
  Serial.begin(115200);
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

  radio.begin();          // Initialize the nRF24L01 Radio
  radio.setChannel(108);  // 2.508 Ghz - Above most Wifi Channels
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range - RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  
  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //   radio.setPALevel(RF24_PA_MAX);

  // Open a writing and reading pipe on each radio, with opposite addresses
  //radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();
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

  if (radio.available())
  {

    while (radio.available())   // While there is data ready to be retrieved from the receive pipe
    {
      radio.read( &inByte, sizeof(inByte) );             // Get the data
    }
    Serial.print(inByte.Yposition);
    Serial.print(" - ");
    Serial.print(inByte.Xposition);
    Serial.print(" - ");
    Serial.println(inByte.buttonNo);
  }

  if (inByte.buttonNo == 0)
  {
    buttonPressed = false;
  }

  // ---------------------------------------
  
  if (inByte.Xposition < 11)
  {
    autoPos = false;
    Xread = map(inByte.Xposition, 0, 10, maxX, minX);
    //motorGo(1, CW, Xread);
    if (timeElapsedX >= Xread) {
      xMotor->onestep(FORWARD, DOUBLE);
      actualXPos++;
      timeElapsedX = 0;
    }
    joyXCentral = false;
  }
  if (inByte.Xposition > 13)
  {
    autoPos = false;
    Xread = map(inByte.Xposition, 14, 25, minX, maxX);
    //motorGo(1, CCW, Xread);
    if (timeElapsedX >= Xread) {
      xMotor->onestep(BACKWARD, DOUBLE);
      actualXPos--;
      timeElapsedX = 0;
    }
    joyXCentral = false;
  }
  if (inByte.Xposition > 10 && inByte.Xposition < 14 && !joyXCentral)
  {
    autoPos = false;
    xMotor->release();
    buttonPressed = false;
    joyXCentral = true;
  }
  
  // ---------------------------------------
  
  if (inByte.Yposition < 11)
  {
    autoPos = false;
    Yread = map(inByte.Yposition, 0, 10, maxY, minY);
    //motorGo(0, CCW, Yread);
    if (timeElapsedY >= Yread) {
      yMotor->onestep(FORWARD, DOUBLE);
      actualYPos++;
      timeElapsedY = 0;
    }
    joyYCentral = false;
  }
  if (inByte.Yposition > 13)
  {
    autoPos = false;
    Yread = map(inByte.Yposition, 14, 25, minY, maxY);
    //motorGo(0, CW, Yread);
    if (timeElapsedY >= Yread) {
      yMotor->onestep(BACKWARD, DOUBLE);
      actualYPos--;
      timeElapsedY = 0;
    }
    joyYCentral = false;
  }
  if (inByte.Yposition > 10 && inByte.Yposition < 14 && !joyYCentral)
  {
    autoPos = false;
    yMotor->release();
    buttonPressed = false;
    joyYCentral = true;
  }

  // -----------------------------------------------------------------
  
  if (inByte.buttonNo == 1 && !buttonPressed)
  {
    recXPos1 = actualXPos;
    recYPos1 = actualYPos;
    recZPos1 = actualZPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
    Serial.println(actualZPos);
    buttonPressed = true;
  }
  if (inByte.buttonNo == 3 && !buttonPressed)
  {
    recXPos2 = actualXPos;
    recYPos2 = actualYPos;
    recZPos2 = actualZPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
    Serial.println(actualZPos);
    buttonPressed = true;
  }
  if (inByte.buttonNo == 5 && !buttonPressed)
  {
    recXPos3 = actualXPos;
    recYPos3 = actualYPos;
    recZPos3 = actualZPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
    Serial.println(actualZPos);
    buttonPressed = true;
  }
  if (inByte.buttonNo == 7 && !buttonPressed)
  {
    recXPos4 = actualXPos;
    recYPos4 = actualYPos;
    recZPos4 = actualZPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
    Serial.println(actualZPos);
    buttonPressed = true;
  }
  if (inByte.buttonNo == 9 && !buttonPressed)
  {
    recXPos5 = actualXPos;
    recYPos5 = actualYPos;
    recZPos5 = actualZPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
    Serial.println(actualZPos);
    buttonPressed = true;
  }
  if (inByte.buttonNo == 2 && !buttonPressed)
  {
    newXPosition = recXPos1;
    newYPosition = recYPos1;
    newZPosition = recZPos1;
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (inByte.buttonNo == 4 && !buttonPressed)
  {
    newXPosition = recXPos2;
    newYPosition = recYPos2;
    newZPosition = recZPos2;
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (inByte.buttonNo == 6 && !buttonPressed)
  {             
    newXPosition = recXPos3;
    newYPosition = recYPos3;
    newZPosition = recZPos3;
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (inByte.buttonNo == 8 && !buttonPressed)
  {
    newXPosition = recXPos4;
    newYPosition = recYPos4;
    newZPosition = recZPos4;
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
  if (inByte.buttonNo == 10 && !buttonPressed)
  {
    newXPosition = recXPos5;
    newYPosition = recYPos5;
    newZPosition = recZPos5;
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
/*
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
    recXPos1 = actualXPos;
    recYPos1 = actualYPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
  }
  if (!mSet2) {
    Serial.println("Record Pos2");
    recXPos2 = actualXPos;
    recYPos2 = actualYPos;
    Serial.println(actualXPos);
    Serial.println(actualYPos);
  }
  if (!mGoTo1 && !buttonPressed) {
    travelledXDistance = 0;
    Serial.println("Go To Pos1");        
    newXPosition = recXPos1;
    newYPosition = recYPos1;
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
    newXPosition = recXPos2;
    newYPosition = recYPos2;
    Serial.print("x: ");
    Serial.print(newXPosition);
    Serial.print(", y: ");
    Serial.println(newYPosition);
    newMove = true;    
    autoPos = true;
    buttonPressed = true;
  }
*/
  
  if (autoPos) {
    goToPosition(newXPosition, newYPosition, newYPosition);
  }
}

void goToPosition(long positionX2reach, long positionY2reach, long positionZ2reach) {
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
  
  if (actualXPos > positionX2reach) {
    motorXDir = false;
  }
  else if (actualXPos < positionX2reach) {
    motorXDir = true;
  }
  if (actualYPos > positionY2reach) {
    motorYDir = false;
  }
  else if (actualYPos < positionY2reach) {
    motorYDir = true;
  }
  
  if (xLeading) {
    if (actualXPos != positionX2reach) {
      e2 = err;
      distanceToXPos = abs(actualXPos - positionX2reach);
      distanceToYPos = abs(actualYPos - positionY2reach);
      travelledXDistance = totalXDistance-distanceToXPos;
      if (travelledXDistance <= distanceToXPos) {
        constrainXDistance = constrain(travelledXDistance, 0, constrainedLimit);
        int stepperAccel = map(constrainXDistance, 0, constrainedLimit, startingXSpeed, 0);
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
      distanceToXPos = abs(actualXPos - positionX2reach);
      distanceToYPos = abs(actualYPos - positionY2reach);
      travelledYDistance = totalYDistance-distanceToYPos;
      if (travelledYDistance <= distanceToYPos) {
        constrainYDistance = constrain(travelledYDistance, 0, constrainedLimit);
        int stepperAccel = map(constrainYDistance, 0, constrainedLimit, startingYSpeed, 0);
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
