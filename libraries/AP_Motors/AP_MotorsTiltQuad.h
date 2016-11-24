/// @file	AP_MotorsTiltQuad.h
/// @brief	Motor control class for Quadcopters with tilt yaw
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

#define AP_MOTORS_CH_TILTQUAD_YAW_LEFT CH_9
#define AP_MOTORS_CH_TILTQUAD_YAW_RIGHT CH_10
/// @class      AP_MotorsQuad
class AP_MotorsTiltQuad : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsTiltQuad(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
        { };

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();
    virtual void        output_to_motors();
    virtual void        enable();
    virtual void        output_armed_stabilizing();
    virtual void        thrust_compensation(void);

    int16_t calc_yaw_radio_output(float yaw_input, float yaw_input_max, uint8_t left);

protected:

    int8_t _yaw_left_reverse = -1;
    int8_t _yaw_right_reverse = -1;
    uint16_t _yaw_left_servo_trim = 1500;
    uint16_t _yaw_right_servo_trim = 1500;
    uint16_t        _yaw_left_servo_min = 1000;                     // Minimum pwm of yaw servo
    uint16_t        _yaw_left_servo_max = 2000;                     // Maximum pwm of yaw servo
    uint16_t        _yaw_right_servo_min = 1000;                     // Minimum pwm of yaw servo
    uint16_t        _yaw_right_servo_max = 2000;                     // Maximum pwm of yaw servo
    uint16_t _yaw_servo_angle_max_deg = 90;

    float _pivot_angle_left;
    float _pivot_angle_right;

};
