#include <iostream>
#include <cmath>
#include <complex>
#include <cstdio>
#include <ostream>
#include <unistd.h>
#include <string>
#include <pthread.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "hikrobot_camera.hpp"
#include "detector_wrapper.hpp"
#include "armor.hpp"
#include "coord_transformer.hpp"
#include "detector/Target.h"

#define INF 10000000.0


const double tmp_dt = 0.3;

ros::Publisher vis_pub, img_pub, armors_pub;

void publish_armor_marker(detector::Armors armors_msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    int id = 0;
    if (armors_msg.armors.size() > 0) {
        for (detector::Armor& armor : armors_msg.armors) {
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.135;
            marker.scale.y = 0.055;
            marker.scale.z = 0.01;

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker.pose.position.x = armor.pose.position.x;
            marker.pose.position.y = armor.pose.position.y;
            marker.pose.position.z = armor.pose.position.z;
            marker.pose.orientation.x = armor.pose.orientation.x;
            marker.pose.orientation.y = armor.pose.orientation.y;
            marker.pose.orientation.z = armor.pose.orientation.z;
            marker.pose.orientation.w = armor.pose.orientation.w;
        }
    }
    else {
        marker.action = visualization_msgs::Marker::DELETE;
    }
    vis_pub.publish(marker);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh;
    
    ArmorDetectorNode detector(nh);
    CoordinateTransformer transformer(nh);
    camera::Camera cam;
    cv::Mat img;
    detector::Armor armor;

    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    img_pub = nh.advertise<sensor_msgs::Image>("/detector/img", 1);

    armors_pub = nh.advertise<detector::Armors>("/detector/armors", 10);

    bool debug = false;

    while(ros::ok()) {
        ros::Time timestamp = ros::Time::now();
        cam.ReadImg(img);
        // std::cout << "Received an image" << std::endl;
        // check succuss
        if(!img.empty()) {
            // convert color
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // only used on infantry
            // cv::Mat rotated_image;
            // cv::flip(img, rotated_image, -1);
            // img = rotated_image;
            if(debug) {
                cv_bridge::CvImage cv_image;
                cv_image.header.stamp = timestamp;  // Set the timestamp
                cv_image.encoding = sensor_msgs::image_encodings::BGR8;  // Set the encoding (e.g., BGR8 for color image)
                cv_image.image = img;
                sensor_msgs::ImagePtr ros_image = cv_image.toImageMsg();
                img_pub.publish(ros_image);
            }
            
            // detect armors in the image
            std::vector<Armor> camera_armors = detector.detectArmors(img);
            
            // publish the armors
            detector::Armors armors_msg;
            armors_msg.header.frame_id = "world";
            armors_msg.header.stamp = timestamp;

            // std::cout << "Armors detected: " << camera_armors.size() << std::endl;

            armors_msg.armors = std::vector<detector::Armor>();
            for (Armor camera_armor : camera_armors) {
                detector::Armor armor_msg;
                armor_msg.id = camera_armor.id;
                armor_msg.type = (int)camera_armor.type;
                armor_msg.pose = transformer.transform(camera_armor, timestamp);
                armors_msg.armors.push_back(armor_msg);
            }
            
            armors_pub.publish(armors_msg);

            if(debug) {
                publish_armor_marker(armors_msg);
            }
        }
        else {
            // std::cout << "Failed to read image" << std::endl;
        }
        // ros::spinOnce();
    }
    
    pthread_exit(NULL);

    return 0;
}