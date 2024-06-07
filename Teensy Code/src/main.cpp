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
/ Robot Control Objects ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
ForceSensor     ati         = ForceSensor(0.99); 
RobotComm       robot       = RobotComm(ASR::CONTROLLER_BAUDRATE);
AdmittanceModel admitModel  = AdmittanceModel(ASR::initMassXY, ASR::initMassZ, ASR::initDampingXY, ASR::initDampingZ, ASR::GRAVITY, ASR::MODEL_DT);
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
  pc.WritePackets(totalTime, ati, admitModel, robot, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();

    /* Incoming Data check */
    if (pc.DataAvailable()) {
      pc.ReadPackets();
      if (pc.ModifyMassXY()) {
        admitModel.SetMassXY(pc.GetNewMassXY());
      }
      if (pc.ModifyMassZ()) {
        admitModel.SetMassZ(pc.GetNewMassZ());
      }
      if (pc.ModifyDampingXY()) {
        admitModel.SetDampingXY(pc.GetNewDampingXY());
      }
      if (pc.ModifyDampingZ()) {
        admitModel.SetDampingZ(pc.GetNewDampingZ());
      }
      if (pc.ModifyScalingFactor()) {
        robot.SetScalingFactor(pc.GetNewScalingFactor());
      }
      if (pc.ChangeTorqueMode()){
        robot.EnableTorque();
      }
      if (pc.ModifyFilter()){
        ati.SetFilterWeight(pc.GetNewFilter());
      }
    }

    /* Admittance Loop */
    if (currentTime - previousTime >= ASR::LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Control */
      robot.ReadRobot();
      ati.ReadForceSensor();
      ati.CalculateGlobalForces(robot.GetPresQ());
      //robot.CalculateSpringForce(ati.GetGlobalFT()); // robot MC should be doing this
      admitModel.UpdateModel(ati.GetGlobalFT(), pc.GetExternalForces()); //remove spring force
      robot.WriteToRobot(packetType, goalXYZ, goalXYZdot, torqueMode);

      /* Outgoing Data */
      loopTime = millis() - startLoop;
      pc.WritePackets(totalTime, ati, admitModel, robot, loopTime);
    }
  }
  robot.WriteToRobot();
  while (!Serial);
  
}
