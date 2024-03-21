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
#include <visualization_msgs/Marker.h>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "tracker_wrapper.hpp"
#include "gimbal_control.hpp"
#include "armor.hpp"
#include "trajectory.hpp"
#include "detector/Target.h"
#include "detector/TargetCommand.h"

#define INF 10000000.0

double delay_t = 0.3; // time delay when shooting
bool permitFiring = true;
Gimbal gimbal;
TrackerWrapper* tracker;
ros::Publisher *test_pub;

const double threshold_low_yaw = 0.2;
const double threshold_upp_yaw = 3.0;

void linear_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {
    
    bool is_current_pair = true;
    geometry_msgs::Point p_a;
    double r = 0;
    double dt = delay_t + std::sqrt(xc * xc + yc * yc) / 29.0;

    double Min_dis = INF, pitch_angle = 0, yaw_angle = 0;
    for (size_t i = 0; i < a_n; i++) {
        double tmp_yaw = yaw + v_yaw * dt + i * (2 * M_PI / a_n);
        // Only 4 armors has 2 radius and height
        if (a_n == 4) {
            r = is_current_pair ? r1 : r2;
            p_a.z = zc + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;
        }
        else {
            r = r1;
            p_a.z = zc;
        }
        p_a.z += vz * dt;
        p_a.x = xc - r * cos(tmp_yaw) + vx * dt;
        p_a.y = yc - r * sin(tmp_yaw) + vy * dt;

        double dis = p_a.x * p_a.x + p_a.y * p_a.y;
        if (dis < Min_dis) {
            Min_dis = dis;
            pitch_angle = -trajectory::SolveTrajectory(p_a.x, p_a.y, p_a.z)*180/ CV_PI;
            yaw_angle = std::atan2(p_a.y, p_a.x) * 180.0 / CV_PI;
        }
    }
    std::cout << "cur pitch_angle: " << gimbal.cur_pitch() << std::endl;
    std::cout << "cur   yaw_angle: " << gimbal.cur_yaw() << std::endl;
    std::cout << "calc pitch_angle: " << pitch_angle << std::endl;
    std::cout << "calc   yaw_angle: " << yaw_angle << std::endl;
    gimbal.set(pitch_angle, yaw_angle, permitFiring);
}

void slow_spin_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {

    bool is_current_pair = true;
    geometry_msgs::Point p_a;
    double r = 0;
    
    double dt = delay_t + std::sqrt(xc * xc + yc * yc) / 29.0;
    
    double Min_dis = INF, pitch_angle = 0, yaw_angle = 0;
    for (size_t i = 0; i < a_n; i++) {
        double tmp_yaw = yaw + v_yaw * dt + i * (2 * M_PI / a_n);
        // Only 4 armors has 2 radius and height
        if (a_n == 4) {
            r = is_current_pair ? r1 : r2;
            p_a.z = zc + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;
        }
        else {
            r = r1;
            p_a.z = zc;
        }
        p_a.z += vz * dt;
        p_a.x = xc - r * cos(tmp_yaw) + vx * dt;
        p_a.y = yc - r * sin(tmp_yaw) + vy * dt;

        double dis = p_a.x * p_a.x + p_a.y * p_a.y;
        if (dis < Min_dis) {
            Min_dis = dis;
            pitch_angle = -trajectory::SolveTrajectory(p_a.x, p_a.y, p_a.z)*180/ CV_PI;
            yaw_angle = std::atan2(p_a.y, p_a.x) * 180.0 / CV_PI;
        }
    }
    std::cout << "cur pitch_angle: " << gimbal.cur_pitch() << std::endl;
    std::cout << "cur   yaw_angle: " << gimbal.cur_yaw() << std::endl;
    std::cout << "calc pitch_angle: " << pitch_angle << std::endl;
    std::cout << "calc   yaw_angle: " << yaw_angle << std::endl;
    gimbal.set(pitch_angle, yaw_angle, permitFiring);
}

void fast_spin_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {
}

void targetCallback(detector::Target tar) {
    double yaw = tar.yaw, v_yaw = tar.v_yaw, r1 = tar.radius_1, r2 = tar.radius_2;
    double xc = tar.position.x, yc = tar.position.y, zc = tar.position.z;
    double vx = tar.velocity.x, vy = tar.velocity.y, vz = tar.velocity.z;
    double dz = tar.dz;
    size_t a_n = tar.armors_num;

    linear_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);

    // if (yaw < threshold_low_yaw) {
        // ROS_INFO("linear_predict");
        // linear_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
    // }
    // else if (yaw > threshold_upp_yaw) {
    //     ROS_INFO("fast_spin_predict");
    //     fast_spin_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
    // }
    // else {
    //     ROS_INFO("slow_spin_predict");
    //     slow_spin_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
    // }

    test_pub->publish(std_msgs::String());
}

void armorsCallback(const detector::Armors &armors_msg) {
    tracker->armorsCallback(armors_msg);
}

void idCallback(const detector::TargetCommand &cmd_msg) {
    tracker->setTargetId(cmd_msg.id);
    // permitFiring = cmd_msg.permitFiring;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh;
    
    // camera::Camera cam;
    cv::Mat img;
    detector::Armor armor;
    tracker = new TrackerWrapper(nh);

    test_pub = new ros::Publisher(nh.advertise<std_msgs::String>("test", 10));

    ros::Subscriber detector_sub = nh.subscribe("/detector/armors", 10, armorsCallback);

    ros::Subscriber id_sub = nh.subscribe("/decision/target_cmd", 10, idCallback);
    
    ros::Subscriber tar_sub = nh.subscribe("/tracker/target", 10, targetCallback);


    ros::spin();
}