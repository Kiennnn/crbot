#include "ros/ros.h"
#include "crbot_diagnostics/BatteryState.h"
#include "crbot_diagnostics/DriverState.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware_state");
    ros::NodeHandle nh;
    ros::Publisher battery_pub = nh.advertise<crbot_diagnostics::BatteryState>("battery_info", 100);
    ros::Publisher driver_pub = nh.advertise<crbot_diagnostics::DriverState>("driver_info", 100);

    ros::Rate loop_rate(0.5);

    // battery info
    crbot_diagnostics::BatteryState battery_msg;
    battery_msg.voltage = 24;
    battery_msg.current = 2;
    battery_msg.percentage = 75;

    // driver info
    crbot_diagnostics::DriverState driver_msg;
    driver_msg.left_current = 2.5;
    driver_msg.left_voltage = 12;
    driver_msg.right_current = 2.5;
    driver_msg.right_voltage = 12;

    while (ros::ok()) {
        battery_pub.publish(battery_msg);
        driver_pub.publish(driver_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}