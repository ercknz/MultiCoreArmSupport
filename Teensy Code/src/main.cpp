#include <Arduino.h>
#include "ForceSensor.h"

int ledPin = 13;
const uint8_t fxpin = 6;
const uint8_t fypin = 5;
const uint8_t fzpin = 4;
const uint8_t txpin = 2;
const uint8_t typin = 1;
const uint8_t tzpin = 0;
const double inputBitRange = 7443;
const double inputBitZero = 4145;
const double FxMax = 290; // N
const double FyMax = 290; // N
const double FzMax = 580; // N
const double TxyzMax = 10; // Nm

ForceSensor ati = ForceSensor(fxpin, fypin, fzpin, txpin, typin, tzpin, 0.05); 

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(13);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long previousTime, currentTime;
  previousTime = millis();
  unsigned long loopTime, startLoop;
  const float loopDT = 100;

  while(1){
    currentTime = millis();
    if (currentTime - previousTime >= loopDT){
      startLoop = millis();
      previousTime = currentTime;

      ati.ReadForceSensor();
      int * sensorValues = ati.GetRawFT();

      loopTime = millis() - startLoop;

      Serial.print(sensorValues[0]);  Serial.print("\t");
      Serial.print(sensorValues[1]);  Serial.print("\t");
      Serial.print(sensorValues[2]);  Serial.print("\t\t");

      Serial.print(sensorValues[0]*(3.3/8192));  Serial.print("\t");
      Serial.print(sensorValues[1]*(3.3/8192));  Serial.print("\t");
      Serial.print(sensorValues[2]*(3.3/8192));  Serial.print("\t\t");

      Serial.print(-(FxMax/inputBitRange)*((double)sensorValues[0] - inputBitZero));  Serial.print("\t");
      Serial.print(-(FyMax/inputBitRange)*((double)sensorValues[1] - inputBitZero));  Serial.print("\t");
      Serial.print(-(FzMax/inputBitRange)*((double)sensorValues[2] - inputBitZero));  Serial.print("\t\t");

      Serial.println(loopTime);

    }
  }
}
