/* Class controls the robot communication

   Created 10/28/2023
   Script by Erick Nunez
*/

#include <Arduino.h>

#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

class RobotComm {
  public:
          RobotComm();
    void  EnableTorque();
    void  ReadRobot();
    void  WriteToRobot();
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

    const int _BAUDRATE;
    const byte _READ_HEADER[4]  = {170,  6, 9,  69};
    const byte _WRITE_HEADER[4] = {150, 10, 1, 101};
    const int16_t _presQ_SLOT = 8;
    const int16_t _presQDOT_SLOT = 20;
    const int16_t _presCURRENT_SLOT = 32;
    const int16_t _presXYZ_SLOT = 44;
    const int16_t _presXYZdot_SLOT = 56;
    const int16_t _BLANK_SLOT = 68;

    float qPres_M[3], qDotPres_M[3];
    float xyzPres_M[3], xyzDotPres_M[3];
    float xyzGoal_M[3], xyzDotGoal_M[3];
    float presCurrent_M[3];
    float   springF_M;
    float   scalingFactor_M;

    int16_t torqueMode;

    const float  _A1A2,      _L1,        _L2;
    const double _OFFSET,    _PHI,       _H_OF_L2;
    const double _Q1_MIN,    _Q1_MAX;
    const double _Q2_LIMIT;
    const double _Q4_MIN,    _Q4_MAX;
    const double _INNER_R,   _Z_LIMIT;
    const double _SPRING_Li, _BETAi, _SPRING_Fi;
    
};

#endif // ROBOT_COMM_H
