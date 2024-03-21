#include "ros/ros.h"
#include "detector/Armor.h"
#include "detector/Armors.h"
#include "detector/TargetCommand.h"

ros::Publisher pub;

void armorCallback(const detector::Armors::ConstPtr& armors_msg)
{
    detector::TargetCommand cmd_msg;
    double min_sq_distance = DBL_MAX;
    if (armors_msg->armors.size() > 0) {
        for (const auto & armor : armors_msg->armors) {
            double sq_distance = armor.pose.position.x * armor.pose.position.x + armor.pose.position.y * armor.pose.position.y;
            if (sq_distance < min_sq_distance) {
                cmd_msg.id = armor.id;
                cmd_msg.permitFiring = false;
            }
        }
    }
    else
        cmd_msg.id = 0;

    pub.publish(cmd_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");

    ros::NodeHandle n;

    pub = n.advertise<detector::TargetCommand>("/decision/target_cmd", 10);

    ros::Subscriber sub = n.subscribe("/detector/armors", 10, armorCallback);

    ros::spin();

    return 0;
}