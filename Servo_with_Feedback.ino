/*********************************************************************
  This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin Townsend/KTOWN  for Adafruit Industries.
  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses call-backs on the event and RX so there's no data handling in the main loop!

#include <SPI.h>
#include <Servo.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

#define SERVO_PIN 6
#define SWITCH_ON 75    // predefined servo negative angle 
#define SWITCH_OFF 105  // predefined servo positive angle
#define RED_PIN 3
#define GREEN_PIN 4
#define BLUE_PIN 5
#define FEEDBACK_PIN A0

int minDegrees = 30;   
int maxDegrees = 150; 
int minFeedback = 188;
int maxFeedback = 438;
int tolerance = 5; // max feedback measurement error

//   uncomment if using calabrate function
//void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
//{
//  // Move to the minimum position and record the feedback value
//  servo.write(minPos);
//  minDegrees = minPos;
//  delay(2000); // make sure it has time to get there and settle
//  minFeedback = analogRead(analogPin);
//  Serial.println(minFeedback);
//  
//  // Move to the maximum position and record the feedback value
//  servo.write(maxPos);
//  maxDegrees = maxPos;
//  delay(2000); // make sure it has time to get there and settle
//  maxFeedback = analogRead(analogPin);
//  Serial.println(maxFeedback);
//}

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

Servo lighSwitchServo;  // create servo object to control the light switch

enum LED {
  Null=0,
  red,
  green,
  blue
};

enum SWITCH {
  OFF,
  ON
};

void Seek(Servo servo, int analogPin, int pos)
{
  // Start the move...
  servo.attach(SERVO_PIN);
  delay(400);
  servo.write(pos);
  
  // Calculate the target feedback value for the final position
  int target = map(pos, minDegrees, maxDegrees, minFeedback, maxFeedback); 
  
  // Wait until it reaches the target
  while(abs(analogRead(analogPin) - target) > tolerance){} // wait...
  servo.detach();
  delay(400);
}

SWITCH getState(int analogPin)
{
  if (analogRead(analogPin)>270)
    return ON;
  else
    return OFF;  
}


bool flipLightSwitch(bool position){
  
  if(position){
    Seek(lighSwitchServo, FEEDBACK_PIN, SWITCH_ON);
    ledControl(green);
  }
 
  else{
    Seek(lighSwitchServo, FEEDBACK_PIN, SWITCH_OFF);
    ledControl(red);
  }
  
  delay(3000);
  ledControl(blue);
  return true;
}


/*
   This function turns at most one color LED on, turning the others off
*/

LED ledControl(LED color)
{
  switch (color) {

    case (red):
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, HIGH);
      break;

    case (green):
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(RED_PIN, LOW);
      break;

    case (blue):
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      break;

    default:
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      return Null;
      break;
  }
  return color;
}


void parseInput(char input){
  Serial.println(F("Command Received: \n"));
  
    if (input == 'r')
    {
      ledControl(red);
      Serial.println(input);
    }

    else if(input == 'g')
    {
      ledControl(green);
      Serial.println(input);
    }

    else if(input == 'b')
    {
      ledControl(blue);
      Serial.println(input);
    }

    else if(input == 'o')
    {
      ledControl(Null);
      Serial.println(input);
    }

    else if(input == 'u')
    {
      flipLightSwitch(true);
      Serial.println(input);
    }
    
    else if(input == 'd')
    {
      flipLightSwitch(false);
      Serial.println(input);
    }
}
/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch (event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*!
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxCallback(uint8_t *buffer, uint8_t len)
{
  Serial.print(F("Received "));
  Serial.print(len);
  Serial.print(F(" bytes: "));
  char arr[len-1];
  for (int i = 0; i < len; i++){
    Serial.print((char)buffer[i]);
    arr[i]=(char)buffer[i];
  }

  Serial.print(F(" ["));

  for (int i = 0; i < len; i++)
  {
    Serial.print(" 0x"); Serial.print((char)buffer[i], HEX);
  }
  Serial.println(F(" ]"));

  
  parseInput(arr[0]);
  /* Echo the same data back! */
  /*uart.write(buffer, len);*/
}

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  ledControl(Null); //Null case turning off all LEDs
  Serial.begin(9600);
  while (!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 BLE Started"));

  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);

  uart.setDeviceName("Eye_o_T"); /* 7 characters max! */
  uart.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
void loop()
{
  delay(1000);
  flipLightSwitch(true);
  Serial.print(getState(FEEDBACK_PIN));
  delay(1000);
  flipLightSwitch(false);
  Serial.print(getState(FEEDBACK_PIN));
  uart.pollACI();
}


