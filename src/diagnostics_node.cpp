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
    diagnostic_object(ros::NodeHandle& nh, std::string topic_name)
    {
        this->sub = nh.subscribe(topic_name, 1000, &diagnostic_object::readData, this);
        this->pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    }
    void readData(const crbot_diagnostics::RawData::ConstPtr& msg)
    {
        // header for diag_msg
        this->diag_msg.header.stamp = ros::Time::now();

        // status for diag_msg
        diagnostic_msgs::DiagnosticStatus msg_status;
        msg_status.name = msg->name;

        // number of msg's keys
        int numberOfKeys = msg->numberOfKeys;

        // read keys and values
        diagnostic_msgs::KeyValue msg_key_value;

        for (int i = 0; i < numberOfKeys; i++) {
            msg_key_value.key = msg->key[i];
            msg_key_value.value = msg->value[i];
            msg_status.values.push_back(msg_key_value);  // add components for status.values array
        }

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
    diagnostic_object battery_diagnostic(nh, "raw_data");

    ros::spin();

    return 0;
}
