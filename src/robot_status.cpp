#include "ros/ros.h"
#include "crbot_diagnostics/BatteryState.h"
#include "crbot_diagnostics/DriverState.h"
#include "crbot_diagnostics/RobotStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_msgs/DiagnosticArray.h"

// void simple_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
// {
//     stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battey is running out!");
//     stat.add("Battery Voltage", 24);
//     stat.add("Battery Current", 2);
//     stat.add("Battery Percentage", 10);
// }

class diagnostic_object
{
  private:
    ros::Subscriber battery_sub;
    ros::Subscriber driver_sub;
    ros::Publisher robot_status_pub;
    ros::Publisher diag_pub;
    crbot_diagnostics::RobotStatus robot_msg;
    diagnostic_msgs::DiagnosticArray diag_msg;

  public:
    diagnostic_object(ros::NodeHandle& nh)
    {
        this->battery_sub = nh.subscribe("battery_info", 1000, &diagnostic_object::readBatteryInfo, this);
        this->driver_sub = nh.subscribe("driver_info", 1000, &diagnostic_object::readDriverInfo, this);
        this->robot_status_pub = nh.advertise<crbot_diagnostics::RobotStatus>("robot_info", 1);
        this->diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    }

    void readBatteryInfo(const crbot_diagnostics::BatteryState::ConstPtr& battery_msg)
    {
        this->robot_msg.battery_voltage = battery_msg->voltage;
        this->robot_msg.battery_current = battery_msg->current;
        this->robot_msg.battery_percentage = battery_msg->percentage;
        this->publish_robot_status();
    }

    void readDriverInfo(const crbot_diagnostics::DriverState::ConstPtr& driver_msg)
    {
        this->robot_msg.left_driver_current = driver_msg->left_current;
        this->robot_msg.left_driver_voltage = driver_msg->left_voltage;
        this->robot_msg.right_driver_current = driver_msg->right_current;
        this->robot_msg.right_driver_voltage = driver_msg->right_voltage;
        this->publish_robot_status();
    }

    void publish_robot_status()
    {
        this->robot_status_pub.publish(this->robot_msg);
    }

    void publish_diag_msg()
    {
        // convert robot status msg to diagnostic msg
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_status");
    ros::NodeHandle nh;

    // ros::Rate loop_rate(10);

    diagnostic_object battery_diagnostic(nh);

    // diagnostic_updater::Updater updater;
    // updater.setHardwareID("Pin AAA");
    // updater.add("Battery Updater", simple_diagnostic);

    // while (ros::ok()) {
    //     updater.update();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();
    return 0;
}
