#include <iostream>
#include <cmath>
#include <complex>
#include <cstdio>
#include <unistd.h>
#include <string>
#include <pthread.h>

#include "ros/publisher.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "tracker_wrapper.hpp"
#include "armor.hpp"
#include "detector/Target.h"
#include "detector/TargetCommand.h"

#define INF 10000000.0

TrackerWrapper* tracker;
void armorsCallback(const detector::Armors &armors_msg) {
    ROS_INFO("armorsCallback");
    tracker->armorsCallback(armors_msg);
}

void idCallback(const detector::TargetCommand &cmd_msg) {
    ROS_INFO("idCallback");
    ROS_INFO("id: %d", cmd_msg.id);
    tracker->setTargetId(cmd_msg.id);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    
    // camera::Camera cam;
    // cv::Mat img;
    // detector::Armor armor;


    tracker = new TrackerWrapper(nh);
    
    ros::Subscriber detector_sub = nh.subscribe("/detector/armors", 10, armorsCallback);
    // ros::Subscriber id_sub = nh.subscribe("/decision/target_cmd", 10, idCallback);

    ros::spin();

    // test_pub = new ros::Publisher(nh.advertise<std_msgs::String>("test", 10));
}