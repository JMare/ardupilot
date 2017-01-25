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

const AP_Param::GroupInfo AP_MotorsTiltQuad::var_info[] = {
    // variables from parent
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // use tricopter param group.... not proud
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_GROUPINFO("YAW_SV_L_RV", 31, AP_MotorsTiltQuad, _yaw_left_reverse, -1),
    AP_GROUPINFO("YAW_SV_R_RV", 32, AP_MotorsTiltQuad, _yaw_right_reverse, -1),
    AP_GROUPINFO("YAW_SV_L_TR", 33, AP_MotorsTiltQuad, _yaw_left_servo_trim, 1500),
    AP_GROUPINFO("YAW_SV_R_TR", 34, AP_MotorsTiltQuad, _yaw_right_servo_trim, 1500),
    AP_GROUPINFO("YAW_SV_L_MN", 35, AP_MotorsTiltQuad, _yaw_left_servo_min, 1000),
    AP_GROUPINFO("YAW_SV_R_MN", 36, AP_MotorsTiltQuad, _yaw_right_servo_min, 1000),
    AP_GROUPINFO("YAW_SV_L_MX", 37, AP_MotorsTiltQuad, _yaw_left_servo_max, 2000),
    AP_GROUPINFO("YAW_SV_R_MX", 38, AP_MotorsTiltQuad, _yaw_right_servo_max, 2000),
    AP_GROUPINFO("YAW_SV_A_MX", 39, AP_MotorsTiltQuad, _yaw_servo_angle_max_deg, 90),
    AP_GROUPINFO("YAW_SV_A_TM", 40, AP_MotorsTiltQuad, _yaw_servo_angle_tilt_max_deg, 10),
#else
    AP_GROUPINFO("YAW_SV_L_RV", 31, AP_MotorsTiltQuad, _yaw_left_reverse, -1),
    AP_GROUPINFO("YAW_SV_R_RV", 32, AP_MotorsTiltQuad, _yaw_right_reverse, 1),
    AP_GROUPINFO("YAW_SV_L_TR", 33, AP_MotorsTiltQuad, _yaw_left_servo_trim, 1800),
    AP_GROUPINFO("YAW_SV_R_TR", 34, AP_MotorsTiltQuad, _yaw_right_servo_trim, 1200),
    AP_GROUPINFO("YAW_SV_L_MN", 35, AP_MotorsTiltQuad, _yaw_left_servo_min, 1000),
    AP_GROUPINFO("YAW_SV_R_MN", 36, AP_MotorsTiltQuad, _yaw_right_servo_min, 1000),
    AP_GROUPINFO("YAW_SV_L_MX", 37, AP_MotorsTiltQuad, _yaw_left_servo_max, 2000),
    AP_GROUPINFO("YAW_SV_R_MX", 38, AP_MotorsTiltQuad, _yaw_right_servo_max, 2000),
    AP_GROUPINFO("YAW_SV_A_MX", 39, AP_MotorsTiltQuad, _yaw_servo_angle_max_deg, 80),
    AP_GROUPINFO("YAW_SV_A_TM", 40, AP_MotorsTiltQuad, _yaw_servo_angle_tilt_max_deg, 10),
#endif
    AP_GROUPEND
};

// setup_motors - configures the motors for a quad
void AP_MotorsTiltQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // plus frame set-up motors on 5-8
    // pretend motors have no coupling to yaw
    add_motor(AP_MOTORS_MOT_1,  90, 0, 2); //c5 right wing
    add_motor(AP_MOTORS_MOT_2, -90, 0, 4); //c6 left wing
    add_motor(AP_MOTORS_MOT_3,   0, 0,  1); //c7 front edf
    add_motor(AP_MOTORS_MOT_4, 180, 0,  3); //c8 back edf

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
    if (fabsf(_pivot_angle_left) > radians(_yaw_servo_angle_tilt_max_deg)) {
        limit.yaw = true;
        _pivot_angle_left = constrain_float(_pivot_angle_left, -radians(_yaw_servo_angle_tilt_max_deg), radians(_yaw_servo_angle_tilt_max_deg));
    }
    _pivot_angle_right = - _pivot_angle_left;

    //call parent to do motor outputs
    AP_MotorsMatrix::output_armed_stabilizing();

    //Now we need to do thrust compensation
    //_thrust_rpyt_out[0] = constrain_float( _thrust_rpyt_out[0]/cosf(_pivot_angle_right),0.0f, 1.0f);
    //_thrust_rpyt_out[1] = constrain_float( _thrust_rpyt_out[1]/cosf(_pivot_angle_left), 0.0f, 1.0f);

    //should also constrain the angle if we have maxed out the thrust

    //now we apply scaling to put more weight on the props
    /*
    float highestprop;
    if(_thrust_rpyt_out[0] > _thrust_rpyt_out[1])
    {
        highestprop = _thrust_rpyt_out[0];
    }
    else{
        highestprop = _thrust_rpyt_out[1];
    }
    */
    //_thrust_rpyt_out[0] = constrain_float(_thrust_rpyt_out[0] * PROP_SCALE_UP,0.0f,1.0f); 
    //_thrust_rpyt_out[1] = constrain_float(_thrust_rpyt_out[1] * PROP_SCALE_UP,0.0f,1.0f); 
    /*
    if(_thrust_rpyt_out[2] < 0.5f){
        _thrust_rpyt_out[2] = constrain_float(_thrust_rpyt_out[2] * 0.5f,0.0f,1.0f); 
    } else{
        _thrust_rpyt_out[2] = constrain_float((_thrust_rpyt_out[2] * 1.5f)-0.5f,0.0f,1.0f); 
    }
    if(_thrust_rpyt_out[3] < 0.5f){
        _thrust_rpyt_out[3] = constrain_float(_thrust_rpyt_out[3] * 0.5f,0.0f,1.0f); 
    } else{
        _thrust_rpyt_out[3] = constrain_float((_thrust_rpyt_out[3] * 1.5f)-0.5f,0.0f,1.0f); 
    }
    */
    //_thrust_rpyt_out[2] = constrain_float(_thrust_rpyt_out[2] * 0.7f,0.0f,1.0f); 
    //_thrust_rpyt_out[3] = constrain_float(_thrust_rpyt_out[3] * 0.7f,0.0f,1.0f); 

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

void AP_MotorsTiltQuad::thrust_compensation(void)
{

    if(_thrust_compensation_callback){
        float _packedthrusttilt[6];
        //pack motor thrusts into tilt
        _packedthrusttilt[0] = _thrust_rpyt_out[0];
        _packedthrusttilt[1] = _thrust_rpyt_out[1];
        _packedthrusttilt[2] = _thrust_rpyt_out[2];
        _packedthrusttilt[3] = _thrust_rpyt_out[3];

        //now pack angles
        _packedthrusttilt[4] = _pivot_angle_left;
        _packedthrusttilt[5] = _pivot_angle_right;

        _thrust_compensation_callback(_packedthrusttilt,4);

       _thrust_rpyt_out[0] = _packedthrusttilt[0];
       _thrust_rpyt_out[1] = _packedthrusttilt[1];
       _thrust_rpyt_out[2] = _packedthrusttilt[2];
       _thrust_rpyt_out[3] = _packedthrusttilt[3];

       _pivot_angle_left = _packedthrusttilt[4];
       _pivot_angle_right = _packedthrusttilt[5];
    }
}
