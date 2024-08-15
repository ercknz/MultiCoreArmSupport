/* Class controls the robot communication

   Created 10/28/2023
   Script by Erick Nunez

*/

#include <Arduino.h>

#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

class RobotComm {
  public:
          RobotComm(const int baudrate);
    void  ReadRobot();
    void  WriteToRobot(uint8_t packetType, float *goalXYZ, float * goalXYZdot, uint8_t torqueMode);
    void  RequestDataOnly();
    void  ChangeTorqueOnly(uint8_t newTorqueValue);
    void  SendNewGoalOnly(float *newXYZGoal, float * newXYZdotGoal);
    void  SendZeroes();
    float *   GetPresQ();
    float *   GetPresQDot();
    int32_t * GetPresQCts();
    int32_t * GetPresQDotCts();
    float *   GetPresPos();
    float *   GetPresVel();
    int32_t * GetGoalQCts();
    int32_t * GetGoalQDotCts();
    float *   GetGoalQ();
    float *   GetGoalQDot();
    float *   GetGoalPos();
    float *   GetGoalVel();
    float     GetSpringForce();
    void      SetScalingFactor(float newScalingFactor);
    
  protected:

    HardwareSerial *robotPort_M;
    const int _BAUDRATE;

    const byte _READ_HEADER[4]  = {170,  6, 9,  69};
    const int16_t _RX_PKT_LEN = 150;
    // const int16_t _RX_presQ_SLOT = 8;
    // const int16_t _RX_presQDOT_SLOT = 20;
    // const int16_t _RX_presCURRENT_SLOT = 32;
    // const int16_t _RX_presXYZ_SLOT = 44;
    // const int16_t _RX_presXYZdot_SLOT = 56;
    // const int16_t _RX_BLANK_SLOT = 68;

    const byte _WRITE_HEADER[4] = {150, 10, 1, 101};
    const int16_t _TX_PKT_LEN = 60;
    const int16_t _TX_PKT_TYPE_SLOT = 5;
    const int16_t _TX_TORQUE_CHANGE_SLOT = 7;
    const int16_t _TX_GOAL_XYZ_SLOT = 8;
    const int16_t _TX_GOAL_XYZDOT_SLOT = 20;
    const int16_t _TX_GOAL_CURRENT_SLOT = 32;
    const int16_t _TX_BLANK_SLOT = 44;    

    // Receiving Data
    float qPres_M[3], qDotPres_M[3];
    int32_t qPresCts_M[3], qDotPresCts_M[3]; 
    float xyzPres_M[3], xyzDotPres_M[3];
    float qGoal_M[3], qDotGoal_M[3];
    int32_t qGoalCts_M[3], qDotGoalCts_M[3];

    // Sending Data
    float xyzGoal_M[3], xyzDotGoal_M[3];

    //float presCurrent_M[3];
    float springF_M;
    float scalingFactor_M;
    int16_t torqueState_M;
    
};

#endif // ROBOT_COMM_H
