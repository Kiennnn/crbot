#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "crbot_diagnostics/RawData.h"

class diagnostic_object
{
  private:
    ros::Publisher pub;
    ros::Subscriber sub;
    diagnostic_msgs::DiagnosticArray diag_msg;

  public:
    diagnostic_object(ros::NodeHandle& nh)
    {
        this->sub = nh.subscribe("raw_data", 1000, &diagnostic_object::readData, this);
        this->pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    }
    void readData(const crbot_diagnostics::RawData::ConstPtr& msg)
    {
        // header for diag_msg
        this->diag_msg.header.stamp = ros::Time::now();

        // status for diag_msg
        diagnostic_msgs::DiagnosticStatus msg_status;
        msg_status.name = msg->name;

        // read keys and values
        diagnostic_msgs::KeyValue msg_key_value_1;
        msg_key_value_1.key = msg->key[0];
        msg_key_value_1.value = msg->value[0];
        diagnostic_msgs::KeyValue msg_key_value_2;
        msg_key_value_2.key = msg->key[1];
        msg_key_value_2.value = msg->value[1];

        // add components for status.values array
        msg_status.values.push_back(msg_key_value_1);
        msg_status.values.push_back(msg_key_value_2);

        // add components for status array
        diag_msg.status.push_back(msg_status);

        // publish to /diagnostics topic
        this->publish_diagnostics();
    }
    void publish_diagnostics()
    {
        this->pub.publish(this->diag_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_node");
    ros::NodeHandle nh;

    // create an object to diagnose battery info
    diagnostic_object battery_diagnostic(nh);
    ros::spin();

    return 0;
}
