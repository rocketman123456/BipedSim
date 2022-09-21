/*
 * File:          biped_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>

#include "webots_interface.h"

/*
 * You may want to add macros here.
 */
extern WbDeviceTag L0_motor;
extern WbDeviceTag L1_motor;
extern WbDeviceTag L2_motor;

extern WbDeviceTag R0_motor;
extern WbDeviceTag R1_motor;
extern WbDeviceTag R2_motor;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
    /* necessary to initialize webots stuff */
    wb_robot_init();

    /*
     * You should declare here WbDeviceTag variables for storing
     * robot devices like this:
     *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
     *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
     */
    webots_device_init();
    //robot_init();

    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1)
    {
        //updateRobotState();

        wb_motor_set_position(L0_motor, 0.0);
        wb_motor_set_position(L1_motor, +60.0 * PI / 180.0);
        wb_motor_set_position(L2_motor, -120.0 * PI / 180.0);

        wb_motor_set_position(R0_motor, 0.0);
        wb_motor_set_position(R1_motor, +60.0 * PI / 180.0);
        wb_motor_set_position(R2_motor, -120.0 * PI / 180.0);

        //robot_control();
    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
