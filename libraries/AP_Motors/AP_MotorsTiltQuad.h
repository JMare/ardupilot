/// @file	AP_MotorsTiltQuad.h
/// @brief	Motor control class for Quadcopters with tilt yaw
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

#define AP_MOTORS_CH_TILTQUAD_YAW_LEFT CH_9
#define AP_MOTORS_CH_TILTQUAD_YAW_RIGHT CH_10

#define RIGHT_PROP 0
#define LEFT_PROP 1
#define FRONT_EDF 2
#define BACK_EDF 3

#define PROP_SCALE_UP 1.1f
#define EDF_SCALE_DOWN 0.9f
#define PROP_TO_EDF_THRUST 2.0f

/// @class      AP_MotorsQuad
class AP_MotorsTiltQuad : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsTiltQuad(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
        {
            AP_Param::setup_object_defaults(this, var_info);
        };

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();
    virtual void        output_to_motors();
    virtual void        enable();
    virtual void        output_armed_stabilizing();
    virtual void        thrust_compensation(void);

    int16_t calc_yaw_radio_output(float yaw_input, float yaw_input_max, uint8_t left);


    static const struct AP_Param::GroupInfo var_info[];
protected:

    AP_Int8 _yaw_left_reverse;
    AP_Int8 _yaw_right_reverse;
    AP_Int16 _yaw_left_servo_trim;
    AP_Int16 _yaw_right_servo_trim;
    AP_Int16        _yaw_left_servo_min;                     // Minimum pwm of yaw servo
    AP_Int16        _yaw_left_servo_max;                     // Maximum pwm of yaw servo
    AP_Int16        _yaw_right_servo_min;                     // Minimum pwm of yaw servo
    AP_Int16        _yaw_right_servo_max;                     // Maximum pwm of yaw servo
    AP_Int16 _yaw_servo_angle_max_deg; //angle range of servo between min and max pwm
    AP_Int16 _yaw_servo_angle_tilt_max_deg; //max degrees used in yaw control

    float _pivot_angle_left;
    float _pivot_angle_right;

};
