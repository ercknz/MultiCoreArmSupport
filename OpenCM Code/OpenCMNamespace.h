/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#ifndef OPEN_CM_NS_H
#define OPEN_CM_NS_H

namespace OCM
{
/* ---------------------------------------------------------------------------------------/
/ Constants ------------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
// Dynamixel Communication Parameters
extern const float   PROTOCOL_VERSION;
extern const int     SENSOR_BAUDRATE;
extern const int     SERIAL_BAUDRATE;
extern const int     MOTOR_BAUDRATE;
extern const char   *DEVICEPORT;
extern const uint8_t CAL_BUTTON_PIN;
extern const uint8_t TORQUE_SWITCH_PIN;

// Dynamixel Motor Parameters
extern const uint8_t ID_SHOULDER;
extern const uint8_t ID_SHLDR_SLAVE;
extern const uint8_t ID_ELEVATION;
extern const uint8_t ID_ELVTN_SLAVE;
extern const uint8_t ID_ELBOW;
extern const uint8_t FULL_ADMITTANCE;
extern const uint8_t FULL_PASSIVE;

// Dynamixel Control Table Addresses
extern const uint16_t ADDRESS_OPERATING_MODE;
extern const uint16_t ADDRESS_VELOCITY_LIMIT;
extern const uint16_t ADDRESS_MAX_POSITION;
extern const uint16_t ADDRESS_MIN_POSITION;
extern const uint16_t ADDRESS_TORQUE_ENABLE;
extern const uint16_t ADDRESS_LED;
extern const uint16_t ADDRESS_GOAL_VELOCITY;
extern const uint16_t ADDRESS_PROFILE_VELOCITY;
extern const uint16_t ADDRESS_GOAL_POSITION;
extern const uint16_t ADDRESS_MOVING;
extern const uint16_t ADDRESS_PRESENT_VELOCITY;
extern const uint16_t ADDRESS_PRESENT_POSITION;

// Dynamixel Packet Parameters
extern const uint8_t       VELOCITY_CONTROL;
extern const uint8_t       POSITION_CONTROL;
extern const uint8_t       EXT_POS_CONTROL;
extern const uint16_t      LEN_GOAL_POSITION;
extern const uint16_t      LEN_PROFILE_VELOCITY;
extern const uint16_t      LEN_PRESENT_POSITION;
extern const uint16_t      LEN_PRESENT_VELOCITY;
extern const int           VEL_BASED_PROFILE;
extern const unsigned char ESC_ASCII_VALUE;

// Dynamixel Parameters for calculations
extern const float DEGREES_PER_COUNT;
extern const float RPM_PER_COUNT;

// Dynamixel Motor Limits
extern const int ELBOW_MIN_POS;
extern const int ELBOW_MAX_POS;
extern const int SHOULDER_MIN_POS;
extern const int SHOULDER_MAX_POS;
extern const int ELEVATION_MIN_POS;
extern const int ELEVATION_MAX_POS;
extern const int ELEVATION_CENTER;
extern const float ELEVATION_RATIO;
extern const int VEL_MAX_LIMIT;

// Loop Parameters
extern const float LOOP_DT; // Milliseconds
}

#endif // OPEN_CM_NS_H
