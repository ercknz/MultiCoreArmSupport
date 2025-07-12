/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#ifndef ARM_SUPPORT_NS_H
#define ARM_SUPPORT_NS_H

namespace ASR
{
/* ---------------------------------------------------------------------------------------/
/ Constants ------------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
// Dynamixel Communication Parameters
extern const float   PROTOCOL_VERSION;
extern const int     CONTROLLER_BAUDRATE;
extern const int     SERIAL_BAUDRATE;
extern const int     MOTOR_BAUDRATE;
extern const char   *DEVICEPORT;
extern const uint8_t CAL_BUTTON_PIN;
extern const uint8_t TORQUE_SWITCH_PIN;
extern const uint8_t LED_PIN;
extern const uint8_t ROBOT_RX_PIN;
extern const uint8_t ROBOT_TX_PIN;

// Force Sensor Constants
extern const float xyzSens[3];
extern const float SENSOR_FILTER_WEIGHT;

// Admitance Control Constants and initial Values
extern const float LOOP_DT;  // Milliseconds
extern const float MODEL_DT; // Secs
extern const float GRAVITY;  // m/sec^2
extern float initMassXY;     // kg
extern float initDampingXY;  // N*(sec/m)
extern float initMassZ;      // kg
extern float initDampingZ;   // N*(sec/m)

// Kinematic Constants
extern const float A1_LINK;      // Shoulder to 4bar linkage
extern const float L1_LINK;      // length of 4bar linkage
extern const float A2_LINK;      // 4bar linkage to elbow
extern const float L2_LINK;      // elbow to sensor
extern const float LINK_OFFSET;  // elbow to sensor offset
extern const float ELEVATION_MAX_POS;
extern const float ELEVATION_MIN_POS;
extern const float DEGREES_PER_COUNT;
extern const float ELEVATION_RATIO;

// Spring Dynamics
extern const int    SPRING_KS; // N/m
extern const float  SPRING_X0; // unstretched spring length in m = 5"
extern const float  SPRING_XI; // initial stretch lenght in m = 6.3"
extern const float  SPRING_SIDE_A; // in m
extern const float  SPRING_SIDE_B; // in m
extern const float  COS_SIN_45; // Cos Sin of 45 degrees
extern const float  DEG_TO_RAD_45; // 45 Degrees to Radians
extern float SPRING_FORCE_SCALING_FACTOR;
}

#endif // ARM_SUPPORT_NS_H