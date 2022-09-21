#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "webots_interface.h"

//-----------------------------------------------------------device
/*电机*/
WbDeviceTag L0_motor;
WbDeviceTag L1_motor;
WbDeviceTag L2_motor;

WbDeviceTag R0_motor;
WbDeviceTag R1_motor;
WbDeviceTag R2_motor;

/*电机编码器*/
WbDeviceTag L0_sensor;
WbDeviceTag L1_sensor;
WbDeviceTag L2_sensor;

WbDeviceTag R0_sensor;
WbDeviceTag R1_sensor;
WbDeviceTag R2_sensor;

/*足底触碰开关*/
WbDeviceTag LF_sensor;
WbDeviceTag RF_sensor;

/*惯导系统*/
WbDeviceTag IMU;

/*
函数功能：初始化devices
*/
void webots_device_init()
{
    // get device
    L0_motor = wb_robot_get_device("L0_motor");
    L1_motor = wb_robot_get_device("L1_motor");
    L2_motor = wb_robot_get_device("L2_motor");

    R0_motor = wb_robot_get_device("R0_motor");
    R1_motor = wb_robot_get_device("R1_motor");
    R2_motor = wb_robot_get_device("R2_motor");

    L0_sensor = wb_robot_get_device("L0_sensor");
    L1_sensor = wb_robot_get_device("L1_sensor");
    L2_sensor = wb_robot_get_device("L2_sensor");

    R0_sensor = wb_robot_get_device("R0_sensor");
    R1_sensor = wb_robot_get_device("R1_sensor");
    R2_sensor = wb_robot_get_device("R2_sensor");

    LF_sensor = wb_robot_get_device("LF_sensor");
    RF_sensor = wb_robot_get_device("RF_sensor");

    IMU = wb_robot_get_device("imu");

    // enable
    wb_position_sensor_enable(L0_sensor, TIME_STEP);
    wb_position_sensor_enable(L1_sensor, TIME_STEP);
    wb_position_sensor_enable(L2_sensor, TIME_STEP);

    wb_position_sensor_enable(R0_sensor, TIME_STEP);
    wb_position_sensor_enable(R1_sensor, TIME_STEP);
    wb_position_sensor_enable(R2_sensor, TIME_STEP);

    wb_touch_sensor_enable(LF_sensor, TIME_STEP);
    wb_touch_sensor_enable(RF_sensor, TIME_STEP);

    wb_inertial_unit_enable(IMU, TIME_STEP);

    wb_keyboard_enable(TIME_STEP);
}
//-----------------------------------------------------------motor

void set_motor_torque(motorNameTypeDef motorName, double torque)
{
    if (torque > 1800)
        torque = 1800;
    if (torque < -1800)
        torque = -1800;

    switch (motorName)
    {
    case L0:
    {
        wb_motor_set_torque(L0_motor, torque);
        break;
    }
    case L1:
    {
        wb_motor_set_torque(L1_motor, torque);
        break;
    }
    case L2:
    {
        wb_motor_set_torque(L2_motor, torque);
        break;
    }

    case R0:
    {
        wb_motor_set_torque(R0_motor, torque);
        break;
    }
    case R1:
    {
        wb_motor_set_torque(R1_motor, torque);
        break;
    }
    case R2:
    {
        wb_motor_set_torque(R2_motor, torque);
        break;
    }

    default:
        break;
    }
}
//-----------------------------------------------------------sensor

double get_motor_angle(motorNameTypeDef motorName)
{
    double angle = 0;
    switch (motorName)
    {
    case L0:
    {
        angle = wb_position_sensor_get_value(L0_sensor);
        break;
    }
    case L1:
    {
        angle = wb_position_sensor_get_value(L1_sensor);
        break;
    }
    case L2:
    {
        angle = wb_position_sensor_get_value(L2_sensor);
        break;
    }

    case R0:
    {
        angle = wb_position_sensor_get_value(R0_sensor);
        break;
    }
    case R1:
    {
        angle = wb_position_sensor_get_value(R1_sensor);
        break;
    }
    case R2:
    {
        angle = wb_position_sensor_get_value(R2_sensor);
        break;
    }
    default:
        break;
    }
    return angle * 180.0f / PI;
}

bool is_foot_touching(legNameTypeDef legName)
{
    if (legName == L)
        return wb_touch_sensor_get_value(LF_sensor);
    if (legName == R)
        return wb_touch_sensor_get_value(RF_sensor);
    return true;
}

eulerAngleTypeDef get_IMU_Angle()
{
    const double *data = wb_inertial_unit_get_roll_pitch_yaw(IMU);

    eulerAngleTypeDef eulerAngle;
    eulerAngle.roll = data[1] * 180.0f / PI;
    eulerAngle.pitch = data[0] * 180.0f / PI;
    eulerAngle.yaw = data[2] * 180.0f / PI;

    return eulerAngle;
}

//-----------------------------------------------------------keyboard

int get_keyboard()
{
    return wb_keyboard_get_key();
}
