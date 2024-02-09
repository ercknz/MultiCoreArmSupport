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
#include "RobotComm.h"
#include "AdmittanceModel.h"
#include "PCComm.h"
#include "ArmSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Model Constants and Initial Parameters -------------------------------------------------/
/----------------------------------------------------------------------------------------*/
const float LOOP_DT   = 8;      // Milliseconds
const float MODEL_DT  = 0.008;  // Secs
const float GRAVITY   = 9.8067; // m/sec^2
float initMassXY      = 1.5f;   // kg
float initDampingXY   = 5.0f;   // N*(sec/m)
float initMassZ       = 1.5f;   // kg
float initDampingZ    = 4.5f;   // N*(sec/m)

/* ---------------------------------------------------------------------------------------/
/ Robot Control Objects ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
ForceSensor     ati         = ForceSensor(0.99); 
RobotComm       robot       = RobotComm();
AdmittanceModel admitModel  = AdmittanceModel(initMassXY, initMassZ, initDampingXY, initDampingZ, GRAVITY, MODEL_DT);
PCComm          pc          = PCComm(&Serial, ASR::SERIAL_BAUDRATE);

/* ---------------------------------------------------------------------------------------/
/ Setup function -------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void setup() {
  /* Wait for Serial Comm */
  while (!Serial);

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
  robot.ReadRobot();
  ati.CalculateGlobalForces(robot.GetPresQ());
  admitModel.SetPosition(robot.GetPresPos());
  pcComm.WritePackets(totalTime, ati, admitModel, robot, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();

    /* Incoming Data check */
    if (pcComm.DataAvailable()) {
      pcComm.ReadPackets();
      if (pcComm.ModifyMassXY()) {
        admitModel.SetMassXY(pcComm.GetNewMassXY());
      }
      if (pcComm.ModifyMassZ()) {
        admitModel.SetMassZ(pcComm.GetNewMassZ());
      }
      if (pcComm.ModifyDampingXY()) {
        admitModel.SetDampingXY(pcComm.GetNewDampingXY());
      }
      if (pcComm.ModifyDampingZ()) {
        admitModel.SetDampingZ(pcComm.GetNewDampingZ());
      }
      if (pcComm.ModifyScalingFactor()) {
        robot.SetScalingFactor(pcComm.GetNewScalingFactor());
      }
      if (pcComm.ModifyMode()){
        robot.EnableTorque();
      }
      if (pcComm.ModifyFilter()){
        ati.SetFilter(pcComm.GetNewFilter());
      }
    }

    /* Admittance Loop */
    if (currentTime - previousTime >= LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Control */
      robot.ReadRobot();
      ati.ReadForceSensor();
      ati.CalculateGlobalForces(robot.GetPresQ());
      robot.CalculateSpringForce(ati.GetGlobalF());
      admitModel.UpdateModel(ati.GetGlobalF(), robot.GetSpringForce(), pcComm.GetExternalForces());
      robot.WriteToRobot();

      /* Outgoing Data */
      loopTime = millis() - startLoop;
      pcComm.WritePackets(totalTime, ati, admitModel, robot, loopTime);
    }
  }
  robot.WriteToRobot();
  while (!Serial);
  
}
