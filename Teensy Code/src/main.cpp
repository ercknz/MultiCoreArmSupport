/* This code...

   Files are pushed to github:
   https://github.com/ercknz/

   Script by erick nunez
   created: 1/24/2023
*/
/* ---------------------------------------------------------------------------------------/
/ Libraries and Headers ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/

#include <Arduino.h>
#include "ForceSensor.h"
#include "SerialPackets.h"
#include "ArmSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Robot Control Objects ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
ForceSensor     ati         = ForceSensor(0.95); 
SerialPackets   openCM      = SerialPackets(&Serial1, ASR::SERIAL_BAUDRATE);

/* ---------------------------------------------------------------------------------------/
/ Setup function -------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void setup() {
  /* Wait for Serial Comm */
  while (!Serial);

  /* Initialize Robot Comm */
  openCM.Connect2Robot(ASR::LED_PIN);
  if (!openCM.IsConnected()) {
    Serial.println("Robot not connected. Check connections and power.");
    while (1);
  }

  delay(100);
  // Set Analog Resolution
  analogReadResolution(13);
}

/* ---------------------------------------------------------------------------------------/
/ Main loop function ---------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  ati.CalibrateSensor();
  delay(100);

  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;

  /* Initialize Robot and Model */
  previousTime = millis();
  openCM.WritePackets(totalTime, ati, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();

    /* Admittance Loop */
    if (currentTime - previousTime >= ASR::LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      ati.ReadForceSensor();

      loopTime = millis() - startLoop;
      openCM.WritePackets(totalTime, ati, loopTime);
    }
  }
  digitalWrite(ASR::LED_PIN, LOW); // Turn off LED
  openCM.SendZeroes();
  while (!Serial);
}
