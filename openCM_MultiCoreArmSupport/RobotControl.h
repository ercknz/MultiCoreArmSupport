/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <DynamixelSDK.h>

class RobotControl {
  public:
          RobotControl();
    void  EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state);
    void  MotorConfig(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler);
    void  ReadMotors(dynamixel::GroupSyncRead &syncReadPacket);
    int   WriteToMotors(float *goalQ, float *goalQdot, bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);
    float *   GetPresQ();
    float *   GetPresQDot();
    int32_t * GetPresQCts();
    int32_t * GetPresQDotCts();
    float *   GetGoalQ();
    float *   GetGoalQDot();
    int32_t * GetGoalQCts();
    int32_t * GetGoalQDotCts();
    float *   GetPresCurrent();
    int16_t * GetPresCurrentCts();
    
  protected:
    int32_t qPresCts_M[3],  qDotPresCts_M[3];
    float   qPres_M[3],     qDotPres_M[3];
    int32_t qCts_M[3],      qDotCts_M[3];
    float   q_M[3],         qDot_M[3];
    float   iPres_M[3];
    int16_t iPresCts_M[3];
    uint8_t dxl_error = 0;
};

#endif // ROBOT_CONTROL_H
