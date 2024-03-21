// STD
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <sstream>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf2/LinearMath/Quaternion.h"
#include <yaml-cpp/yaml.h>

// Local
#include "gimbal_control.hpp"
#include "detector/GimbalOrientation.h"

#define PI 3.1415926535897932384626433832795

double WJ_dx, WJ_dy, WJ_dz;
double JC_dx, JC_dy, JC_dz;
double C_dx, C_dy, C_dz;

// This function we construct a tf broadcaster to broadcast the relationship between world, joint and camera coordinate
// world coordinate -> joint coordinate -> camera coordinate, we construct a tf tree to represent the relationship
void broadcastTF(double row, double pitch, double yaw, ros::Time time) {
    
    // create tf boadcaster
    static tf2_ros::TransformBroadcaster broadcaster;
    
    // create transform information
    geometry_msgs::TransformStamped tf_WJ;
    geometry_msgs::TransformStamped tf_JC;

    tf2::Quaternion qtn;
    tf2::Quaternion qtn1;
    tf2::Quaternion qtn2;
    tf2::Quaternion qtn3;

    // transform world coordinate to joint coordinate
    // broadcasr tf msg
    tf_WJ.header.frame_id = "world";
    tf_WJ.header.stamp = time;
    tf_WJ.child_frame_id = "joint";

    qtn.setRPY(0, 0, yaw+0.03);
    tf_WJ.transform.rotation.x = qtn.getX();
    tf_WJ.transform.rotation.y = qtn.getY();
    tf_WJ.transform.rotation.z = qtn.getZ();
    tf_WJ.transform.rotation.w = qtn.getW();
    
    tf_WJ.transform.translation.x = WJ_dx;
    tf_WJ.transform.translation.y = WJ_dy;
    tf_WJ.transform.translation.z = WJ_dz;

    broadcaster.sendTransform(tf_WJ);

    // transform joint coordinate to camera coordinate
    // broadcasr tf msg

    tf_JC.header.frame_id = "joint";
    tf_JC.header.stamp = time;
    tf_JC.child_frame_id = "camera";

    qtn1.setRPY(0, pitch, 0);
    qtn2.setRPY(0, 0, -PI/2);
    qtn3.setRPY(-PI/2, 0, 0);
    qtn = qtn1*qtn2*qtn3;
    tf_JC.transform.rotation.x = qtn.getX();
    tf_JC.transform.rotation.y = qtn.getY();
    tf_JC.transform.rotation.z = qtn.getZ();
    tf_JC.transform.rotation.w = qtn.getW();
    
    tf_JC.transform.translation.x = JC_dx + C_dx;
    tf_JC.transform.translation.y = JC_dy + C_dy;
    tf_JC.transform.translation.z = JC_dz + C_dz;

    broadcaster.sendTransform(tf_JC);
    // ROS_INFO("%lf %lf %lf", WJ_dx, WJ_dy, WJ_dz);
}

void read_Params() {
    std::string filename = "/home/hero/Vision_Module/src/detector/params/gimbal.yaml";
    YAML::Node config = YAML::LoadFile(filename);
    if(!config) {
        ROS_INFO("Read file failed!!");
    }
    else {
        if (!config["WJ_translation"]) {
            ROS_INFO("No WJ_translation!!");
        }
        else {
            WJ_dx = config["WJ_translation"]["x"].as<double>();
            WJ_dy = config["WJ_translation"]["y"].as<double>();
            WJ_dz = config["WJ_translation"]["z"].as<double>();
        }
        
        JC_dx = config["JC_translation"]["x"].as<double>();
        JC_dy = config["JC_translation"]["y"].as<double>();
        JC_dz = config["JC_translation"]["z"].as<double>();
    }
        config = YAML::LoadFile("/home/hero/Vision_Module/src/detector/params/camera.yaml");

        C_dx = config["C_translation"]["x"].as<double>();
        C_dy = config["C_translation"]["y"].as<double>();
        C_dz = config["C_translation"]["z"].as<double>();
}

int main(int argc, char *argv[]) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_node");
    ros::NodeHandle nh;

    Gimbal gimbal;
    
    srand(time(NULL));

    read_Params();


    while(ros::ok()) {
        detector::GimbalOrientation gimbal_msg;
        gimbal_msg.pitch = gimbal.cur_pitch() / 180.0 * PI;
        gimbal_msg.yaw = gimbal.cur_yaw() / 180.0 * PI;
        ROS_INFO("Pitch: %f, Yaw: %f", gimbal_msg.pitch, gimbal_msg.yaw);
        broadcastTF(0.0, gimbal_msg.pitch, gimbal_msg.yaw, ros::Time::now());
        ros::spinOnce();
    }
    
    return 0;
}