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

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

Servo lighSwitchServo;  // create servo object to control the light switch

 enum LED {
  Null=0,
  red,
  green,
  blue
};

bool flipLightSwitch(bool position){
  lighSwitchServo.attach(SERVO_PIN);
  
  if(position){
    lighSwitchServo.write(SWITCH_ON);
    ledControl(green);
  }
  
  else{
    lighSwitchServo.write(SWITCH_OFF);
    ledControl(red);
  }
  
  delay(200);
  lighSwitchServo.detach();
  delay(2800);
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
  lighSwitchServo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
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
  uart.pollACI();
}



