/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Admittance-Control-for-Arm-Support

   Script by erick nunez
   created: 1/24/2019

*/

/* ---------------------------------------------------------------------------------------/
/ Libraries and Headers ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
#include <DynamixelSDK.h>
#include "openCMNamespace.h"
#include "RobotControl.h"
#include "UtilityFunctions.h"
#include "SerialPackets.h"

/* ---------------------------------------------------------------------------------------/
/ DXL port and packets -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

/* ---------------------------------------------------------------------------------------/
/ Robot Control Objects ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
RobotControl    ArmRobot = RobotControl(OCM::A1_LINK, OCM::L1_LINK, OCM::A2_LINK, OCM::L2_LINK, OCM::LINK_OFFSET);
SerialPackets   c2cComm  = SerialPackets(&Serial, OCM::SERIAL_BAUDRATE);

/* ---------------------------------------------------------------------------------------/
/ Setup function -------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void setup() {
  /* Wait for Serial Comm */
  while (!Serial);
  /* Set pin modes */
  pinMode(OCM::TORQUE_SWITCH_PIN, INPUT_PULLUP);
  /* Setup port and packet handlers */
  portHandler   = dynamixel::PortHandler::getPortHandler(OCM::DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(OCM::PROTOCOL_VERSION);
  delay(100);
  /* Dynamixel Setup */
  portHandler -> openPort();
  portHandler -> setBaudRate(OCM::MOTOR_BAUDRATE);
}


/* ---------------------------------------------------------------------------------------/
/ Main loop function ---------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  OptoForceSensor.SensorConfig();
  ArmRobot.MotorConfig(portHandler, packetHandler);
  delay(100);

  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;
  
  /* Sets up dynamixel read/write packet parameters */
  int  goalReturn;
  bool addParamResult = false;
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY + OCM::LEN_PRESENT_POSITION);
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, OCM::ADDRESS_GOAL_POSITION, OCM::LEN_GOAL_POSITION);
  addParamResult = syncReadPacket.addParam(OCM::ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(OCM::ID_ELBOW);
  addParamResult = syncReadPacket.addParam(OCM::ID_ELEVATION);

  /* Torque Enable Switch Check */
  byte switchState = digitalRead(OCM::TORQUE_SWITCH_PIN);
  if (switchState == LOW) {
    ArmRobot.EnableTorque(portHandler, packetHandler, OCM::FULL_ADMITTANCE);
  } else {
    ArmRobot.EnableTorque(portHandler, packetHandler, OCM::FULL_PASSIVE);
  }
  delay(100);

  /* Initialize Robot and Model */
  previousTime = millis();
  ArmRobot.ReadRobot(syncReadPacket);
  pcComm.WritePackets(totalTime, ArmRobot, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();

    /* Admittance Loop */
    if (currentTime - previousTime >= OCM::LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Control */
      ArmRobot.ReadRobot(syncReadPacket);
      ArmRobot.WriteToRobot(AdmitModel.GetGoalPos(), AdmitModel.GetGoalVel(), addParamResult, syncWritePacket);

      /* Outgoing Data */
      loopTime = millis() - startLoop;
      pcComm.WritePackets(totalTime, ArmRobot, loopTime);
    }
  }
  if (!Serial) {
    ArmRobot.EnableTorque(portHandler, packetHandler, OCM::FULL_PASSIVE);
    while (!Serial);
  }
}
