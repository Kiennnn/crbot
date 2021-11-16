// #include "ros/ros.h"
// #include "diagnostic_msgs/DiagnosticArray.h"
// #include "diagnostic_msgs/DiagnosticStatus.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "diagnostics_node");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
//     ros::Rate loop_rate(10);
//     while (ros::ok()) {
//         diagnostic_msgs::DiagnosticArray msg;
//         msg.status[0].name = "message";
//         msg.status[0].values[0].key = "value";
//         msg.status[0].values[0].value = "10";

//         pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     return 0;
// }

#include "ros/ros.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_node");
    ros::NodeHandle nh;

    ros::Publisher diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        diagnostic_msgs::DiagnosticArray dia_array;
        diagnostic_msgs::DiagnosticStatus robot_status;
        robot_status.name = "Robot";
        robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        robot_status.message = "Everything seem to be ok.";
        diagnostic_msgs::KeyValue emergency;
        emergency.key = "Emgergencystop hit";
        emergency.value = "false";
        diagnostic_msgs::KeyValue exited_normally;
        emergency.key = "Exited normally";
        emergency.value = "true";

        robot_status.values.push_back(emergency);
        robot_status.values.push_back(exited_normally);

        diagnostic_msgs::DiagnosticStatus eth_status;
        eth_status.name = "EtherCAT Master";
        eth_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        eth_status.message = "Running";

        dia_array.status.push_back(robot_status);
        dia_array.status.push_back(eth_status);

        diagnostic_pub.publish(dia_array);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}