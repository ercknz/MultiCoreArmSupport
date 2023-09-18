#include <Arduino.h>
#include "ForceSensor.h"

ForceSensor ati = ForceSensor(0.99); 

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(13);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(".....Calibrating Force Sensor.....");
  ati.CalibrateSensor();

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
      int * sensorValues = ati.GetRawCtsFT();
      float * ftValues = ati.GetFilteredFT();

      loopTime = millis() - startLoop;

      Serial.print(sensorValues[0]);  Serial.print("\t");
      Serial.print(sensorValues[1]);  Serial.print("\t");
      Serial.print(sensorValues[2]);  Serial.print("\t\t");

      Serial.print(ftValues[0]);  Serial.print("\t");
      Serial.print(ftValues[1]);  Serial.print("\t");
      Serial.print(ftValues[2]);  Serial.print("\t\t");

      Serial.println(loopTime);

    }
  }
}
