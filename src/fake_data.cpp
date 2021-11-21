#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "crbot_diagnostics/RawData.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_data");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<crbot_diagnostics::RawData>("raw_data", 1000);

    ros::Rate loop_rate(10);

    crbot_diagnostics::RawData msg;
    msg.name = "battery";

    std::string msg_key_1;
    std::string msg_value_1;
    msg_key_1 = "voltage";
    msg_value_1 = "24V";
    msg.key.push_back(msg_key_1);
    msg.value.push_back(msg_value_1);

    std::string msg_key_2;
    std::string msg_value_2;
    msg_key_2 = "percentage";
    msg_value_2 = "25%";
    msg.key.push_back(msg_key_2);
    msg.value.push_back(msg_value_2);

    while (ros::ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}