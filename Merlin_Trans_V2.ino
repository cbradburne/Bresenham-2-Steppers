//#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

RF24 radio(40,53);

byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

unsigned long previousMillis = 0;
const long intervalSend = 500;           // interval at which to blink (milliseconds)

#define joystickPanPin A0
#define joystickPanTilt A1

boolean yawPressed;
boolean pitchPressed;
boolean store1;
boolean moveTo1;
boolean store2;
boolean moveTo2;
boolean store3;
boolean moveTo3;
boolean store4;
boolean moveTo4;
boolean store5;
boolean moveTo5;
boolean moveStoreButt;

int joyPan;
int joyTilt;

struct dataStruct {
  int Xposition;          // The Joystick position values
  int Yposition;
  int buttonNo;          // The Joystick push-down switch
} myData;

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  
  Serial.begin(115200);
  
  radio.begin();          // Initialize the nRF24L01 Radio
  radio.setChannel(108);  // Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range

  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  //radio.setPALevel(RF24_PA_LOW);
  radio.setPALevel(RF24_PA_MAX);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  //radio.openReadingPipe(1, addresses[1]);

  // Start the radio listening for data
  //radio.startListening();
}

void loop() {  
  unsigned long currentMillis = millis();

  joyPan = analogRead(joystickPanPin);
  joyTilt = analogRead(joystickPanTilt);
  myData.Xposition = map(joyPan, 0, 1023, 0, 25);
  myData.Yposition = map(joyTilt, 0, 1023, 0, 25);
  
  store1 = digitalRead(2);
  moveTo1 = digitalRead(3);
  store2 = digitalRead(4);
  moveTo2 = digitalRead(5);
  store3 = digitalRead(6);
  moveTo3 = digitalRead(7);
  store4 = digitalRead(8);
  moveTo4 = digitalRead(9);
  store5 = digitalRead(10);
  moveTo5 = digitalRead(11);
  
  if (!store1 && !moveStoreButt) {
    myData.buttonNo = 1;
    moveStoreButt = 1;
  }
  if (!moveTo1 && !moveStoreButt) {
    myData.buttonNo = 2;
    moveStoreButt = 1;
  }

  if (!store2 && !moveStoreButt) {
    myData.buttonNo = 3;
    moveStoreButt = 1;
  }
  if (!moveTo2 && !moveStoreButt) {
    myData.buttonNo = 4;
    moveStoreButt = 1;
  }

  if (!store3 && !moveStoreButt) {
    myData.buttonNo = 5;
    moveStoreButt = 1;
  }
  if (!moveTo3 && !moveStoreButt) {
    myData.buttonNo = 6;
    moveStoreButt = 1;
  }

  if (!store4 && !moveStoreButt) {
    myData.buttonNo = 7;
    moveStoreButt = 1;
  }
  if (!moveTo4 && !moveStoreButt) {
    myData.buttonNo = 8;
    moveStoreButt = 1;
  }
  
  if (!store5 && !moveStoreButt) {
    myData.buttonNo = 60;
    moveStoreButt = 1;
  }
  if (!moveTo5 && !moveStoreButt) {
    myData.buttonNo = 61;
    moveStoreButt = 1;
  }

  if (store1 && moveTo1 && store2 && moveTo2 && store3 && moveTo3 && store4 && moveTo4 && store5 && moveTo5 && moveStoreButt) {
    moveStoreButt = 0;
    if (myData.buttonNo == 60 || myData.buttonNo == 61)
    {
      myData.buttonNo = 62;
    }
  }
  if (currentMillis - previousMillis >= intervalSend) {
    // save the last time you sent data
    previousMillis = currentMillis;
    radio.write( &myData, sizeof(myData) );
    Serial.print(myData.Yposition);
    Serial.print(" - ");
    Serial.print(myData.Xposition);
    Serial.print(" - ");
    Serial.println(myData.buttonNo);
    
    if (myData.buttonNo > 0 && myData.buttonNo < 11 || myData.buttonNo == 62)
    {
      myData.buttonNo = 0;
    }
  }
}
