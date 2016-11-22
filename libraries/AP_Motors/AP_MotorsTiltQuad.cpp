/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTiltQuad.cpp - ArduCopter motors library
 *       Code adapted by James Mare from code by RandyMackay. DIYDrones.com
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTiltQuad.h"

extern const AP_HAL::HAL& hal;
// setup_motors - configures the motors for a quad
void AP_MotorsTiltQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // plus frame set-up motors on 5-8
    // pretend motors have no coupling to yaw
    add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
    add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
    add_motor(AP_MOTORS_MOT_3,   0, 0,  1);
    add_motor(AP_MOTORS_MOT_4, 180, 0,  3);

    //Add servos
    add_motor_num(AP_MOTORS_CH_TILTQUAD_YAW_LEFT);
    add_motor_num(AP_MOTORS_CH_TILTQUAD_YAW_RIGHT);

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();
}

void AP_MotorsTiltQuad::enable()
{
    //enable servos
    rc_enable_ch(AP_MOTORS_CH_TILTQUAD_YAW_LEFT);
    rc_enable_ch(AP_MOTORS_CH_TILTQUAD_YAW_RIGHT);

    //Call parent
    AP_MotorsMatrix::enable();
}

void AP_MotorsTiltQuad::output_to_motors()
{
    //Output to servos then call parent

    switch (_spool_mode) {
    case SHUT_DOWN: 
        hal.rcout->cork();
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_LEFT, _yaw_left_servo_trim);
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_RIGHT, _yaw_right_servo_trim);
        hal.rcout->push();
        break;
    case SPIN_WHEN_ARMED:
        hal.rcout->cork();
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_LEFT, _yaw_left_servo_trim);
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_RIGHT, _yaw_right_servo_trim);
        hal.rcout->push();
        break;
    case SPOOL_UP:
    case THROTTLE_UNLIMITED:
    case SPOOL_DOWN:
        //need to add yaw output here
        hal.rcout->cork();
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_LEFT, calc_yaw_radio_output(_pivot_angle_left, radians(_yaw_servo_angle_max_deg),1));
        rc_write(AP_MOTORS_CH_TILTQUAD_YAW_RIGHT, calc_yaw_radio_output(_pivot_angle_right, radians(_yaw_servo_angle_max_deg),0));
        hal.rcout->push();
        break;
    }

    //now call parent to do motor outputs
    AP_MotorsMatrix::output_to_motors();
}

void AP_MotorsTiltQuad::output_armed_stabilizing()
{
    //do yaw control then call parent

    float yaw_thrust = _yaw_in * get_compensation_gain()*sinf(radians(_yaw_servo_angle_max_deg));
    _pivot_angle_left = safe_asin(yaw_thrust);
    if (fabsf(_pivot_angle_left) > radians(_yaw_servo_angle_max_deg)) {
        limit.yaw = true;
        _pivot_angle_left = constrain_float(_pivot_angle_left, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
    }
    _pivot_angle_right = - _pivot_angle_left;

    //call parent to do motor outputs
    AP_MotorsMatrix::output_armed_stabilizing();
}


int16_t AP_MotorsTiltQuad::calc_yaw_radio_output(float yaw_input, float yaw_input_max, uint8_t left)
{
    int16_t ret;

    if(left)
    {
    if (_yaw_left_reverse < 0) {
        yaw_input = -yaw_input;
    }

    if (yaw_input >= 0){
        ret = (_yaw_left_servo_trim + (yaw_input/yaw_input_max * (_yaw_left_servo_max - _yaw_left_servo_trim)));
    } else {
        ret = (_yaw_left_servo_trim + (yaw_input/yaw_input_max * (_yaw_left_servo_trim - _yaw_left_servo_min)));
    }
    }
    else 
    {
        if (_yaw_right_reverse < 0) {
            yaw_input = -yaw_input;
        }

        if (yaw_input >= 0){
            ret = (_yaw_right_servo_trim + (yaw_input/yaw_input_max * (_yaw_right_servo_max - _yaw_right_servo_trim)));
        } else {
            ret = (_yaw_right_servo_trim + (yaw_input/yaw_input_max * (_yaw_right_servo_trim - _yaw_right_servo_min)));
        }
    }

    return ret;
}
