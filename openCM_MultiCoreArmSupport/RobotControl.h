/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <DynamixelSDK.h>

class RobotControl {
  public:
          RobotControl(const float A1, const float L1, const float A2, const float L2, const float A3, const float A4);
          
    void  EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state);
    void  MotorConfig(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler);
    void  ReadRobot(dynamixel::GroupSyncRead &syncReadPacket);
    void  WriteToRobot(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);
    void  UpdateGoals(float *xyz, float *xyzDot);
    uint8_t   GetCurrentTorqueMode();
    float *   GetPresQ();
    float *   GetPresQDot();
    int32_t * GetPresQCts();
    int32_t * GetPresQDotCts();
    float *   GetPresXYZ();
    float *   GetPresXYZdot();
    float *   GetGoalQ();
    float *   GetGoalQDot();
    int32_t * GetGoalQCts();
    int32_t * GetGoalQDotCts();
    float *   GetGoalXYZ();
    float *   GetGoalXYZdot();
    float *   GetPresCurrent();
    int16_t * GetPresCurrentCts();
    void      CalculateSpringForce(float *forces); 
    float     GetSpringForce();
    void      SetScalingFactor(float newScalingFactor);
    void      InitializeGoals();
    
  protected:
    void  fKine();
    void  ReadMotors(dynamixel::GroupSyncRead &syncReadPacket);
    void  iKineGeometric();
    void  iKineOptimized();
    int   WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);

    const float  _A1A2, _A3, _A4, _L1, _L2;
    const float _MaxVelocityXYZ;
    const float _PHI, _H_OF_L2;
    const float  _Q1_MIN,    _Q1_MAX;
    const float _Q2_LIMIT;
    const float _Q4_MIN,    _Q4_MAX;
    const float _INNER_R,   _Z_LIMIT;
    const float _SPRING_Li, _BETAi, _SPRING_Fi;
    const float _PI;

    const float _TASK_X_LIMIT = 0.0f; // Meters
    const float _TASK_Y_LIMIT = 0.5f; // Meters

    float J_M[3][3] = {{0.0f}};     // Jacobian Matrix

    int32_t qPresCts_M[3],  qDotPresCts_M[3];
    float   qPres_M[3],     qDotPres_M[3];
    float   xyzPres_M[3],   xyzDotPres_M[3];
    int32_t qCts_M[3],      qDotCts_M[3];     // Goal Q and Qdot in counts
    float   q_M[3],         qDot_M[3];        // Goal Q and Qdot in radians
    float   xyz_M[3],       xyzDot_M[3];      // Goal XYZ coordinates in meters
    float   springF_M;
    float   scalingFactor_M;
    float   iPres_M[3];     // Present Current
    int16_t iPresCts_M[3];  // Present Current Counts
    uint8_t dxl_error = 0;
    uint8_t currentTorqueMode_M;
};

#endif // ROBOT_CONTROL_H
