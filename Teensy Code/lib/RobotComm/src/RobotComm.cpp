/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   Class arrays are used as follows:
   xyz[3]     = {x, y, x}; 
   xyzDot[3]  = {xDot, yDot, zDot};
   q[3]       = {q1(shoulder), q2(elevation), q4(elbow)};
   qDot[3]    = {q1Dot(shoulder), q2Dot(elevation), q4Dot(elbow)}

   Created 10/28/2020
   Script by Erick Nunez
*/

#include <Arduino.h>
#include "RobotComm.h"

/* ---------------------------------------------------------------------------------------/
/ Arm Support Constructor ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
RobotComm::RobotComm(){
  scalingFactor_M = SPRING_FORCE_SCALING_FACTOR;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int32_t* RobotComm::GetPresQCts(){ 
  return qPresCts_M;
} 

int32_t* RobotComm::GetPresQDotCts(){
  return qDotPresCts_M;
}

float* RobotComm::GetPresQ(){
  return qPres_M;
}

float* RobotComm::GetPresQDot(){
  return qDotPres_M;
}

float* RobotComm::GetPresPos(){ 
  return xyzPres_M;
}

float* RobotComm::GetPresVel(){
  return xyzDotPres_M;
}

int32_t* RobotComm::GetGoalQCts(){ 
  return qCts_M;
}

int32_t* RobotComm::GetGoalQDotCts(){
  return qDotCts_M;
}

float* RobotComm::GetGoalQ(){
  return q_M;
}

float* RobotComm::GetGoalQDot(){
  return qDot_M;
}

float* RobotComm::GetGoalPos(){ 
  return xyz_M;
}

float* RobotComm::GetGoalVel(){
  return xyzDot_M;
}

float RobotComm::GetSpringForce(){
  return springF_M;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Motors Reading/Writing -----------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::ReadRobot(){
}

void RobotComm::WriteToRobot(){
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Spring Force Member Functions ----------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::SetScalingFactor(float newScalingFactor){
  scalingFactor_M = newScalingFactor;
}
