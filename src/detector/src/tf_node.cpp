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

#include <opencv2/core.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <yaml-cpp/yaml.h>

// Local
#include "gimbal_control.hpp"
#include "trajectory.hpp"
#include "detector/GimbalOrientation.h"
#include "detector/Target.h"

#define INF 100000000

// camera fixed points to optical center
// const double cx = 0.03657 - 0.0055;
// const double cy = 0.0;
// const double cz = 0.0228;

Gimbal gimbal;
double delay_t = 0.07; // time delay when shooting
double bullet_speed = 25.0;

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

    qtn.setRPY(0, 0, yaw+0.02);
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
    qtn2.setRPY(0, 0, -CV_PI/2);
    qtn3.setRPY(-CV_PI/2, 0, 0);
    qtn = qtn1*qtn2*qtn3;
    tf_JC.transform.rotation.x = qtn.getX();
    tf_JC.transform.rotation.y = qtn.getY();
    tf_JC.transform.rotation.z = qtn.getZ();
    tf_JC.transform.rotation.w = qtn.getW();

    tf_JC.transform.translation.x = JC_dx + C_dx;
    tf_JC.transform.translation.y = JC_dy + C_dy;
    tf_JC.transform.translation.z = JC_dz + C_dz;

    broadcaster.sendTransform(tf_JC);
    // ROS_INFO("%lf %lf %lf", WJ_dx + JC_dx, WJ_dy + JC_dy, WJ_dz + JC_dz);
}

void read_Params() {
    std::string filename = "/home/infantry_1/catkin_ws/src/detector/params/gimbal.yaml";
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
    config = YAML::LoadFile("/home/infantry_1/catkin_ws/src/detector/params/camera.yaml");

    C_dx = config["C_translation"]["x"].as<double>();
    C_dy = config["C_translation"]["y"].as<double>();
    C_dz = config["C_translation"]["z"].as<double>();
}

const double threshold_low_yaw = 0.2;
const double threshold_upp_yaw = 5.0;
const bool firing = true;
const bool not_firing = false;
const bool patrol = true;
const bool not_patrol = false;
int lost_cnt = 0;

void allowFire(double pitch_angle, double yaw_angle, bool firing, bool not_patrol, const double error=3) {
    ROS_INFO("Error: %f", abs(gimbal.cur_yaw() - yaw_angle));
    if(fabs(gimbal.cur_pitch() - pitch_angle) <= fabs(error) && fabs(gimbal.cur_yaw() - yaw_angle) <= fabs(error)) {
        gimbal.set(pitch_angle, yaw_angle, firing, not_patrol);
    }
    else {
        gimbal.set(pitch_angle, yaw_angle, not_firing, not_patrol);
    }
}

void linear_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {
    
    bool is_current_pair = true;
    geometry_msgs::Point p_a;
    double r = 0;
    double dt = delay_t + std::sqrt(xc * xc + yc * yc) / bullet_speed;
    double nx = 0.0, ny = 0.0, nz = 0.0;
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
            nx = p_a.x;
            ny = p_a.y;
            nz = p_a.z;
            
            Min_dis = dis;
            pitch_angle = -trajectory::SolveTrajectory(p_a.x, p_a.y, p_a.z)*180/ CV_PI;
            yaw_angle = std::atan2(p_a.y, p_a.x) * 180.0 / CV_PI;
        }
    }
    ROS_INFO("Cureent position: x: %.4f      y: %.4f     z: %.4f", nx, ny, nz);
    // std::cout << "calc pitch_angle: " << pitch_angle << std::endl;
    // std::cout << "calc   yaw_angle: " << yaw_angle << std::endl;
    // gimbal.set(pitch_angle, yaw_angle, firing, not_patrol);
    allowFire(pitch_angle, yaw_angle, firing, not_patrol);
}

void slow_spin_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {

    bool is_current_pair = true;
    geometry_msgs::Point p_a;
    double r = 0;
    
    double dt = delay_t + std::sqrt(xc * xc + yc * yc) / bullet_speed;
    
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
    // std::cout << "calc pitch_angle: " << pitch_angle << std::endl;
    // std::cout << "calc   yaw_angle: " << yaw_angle << std::endl;
    // gimbal.set(pitch_angle, yaw_angle, firing, not_patrol);
    allowFire(pitch_angle, yaw_angle, firing, not_patrol);
}

void fast_spin_predict(double yaw, double v_yaw, double r1, double r2, double xc, double yc, double zc, double vx, double vy, double vz, double dz, size_t a_n) {
    geometry_msgs::Point p_a;
    double r = (r1+r2)/2; // Aim at lower armors

    double dt = delay_t + std::sqrt(xc * xc + yc * yc) / bullet_speed;

    double tmp_yaw = yaw - (-PI*1.5/v_yaw-dt)*v_yaw;
    p_a.z = zc;
    p_a.x = xc - r * cos(tmp_yaw) + vx * dt;
    p_a.y = yc - r * sin(tmp_yaw) + vy * dt;

    double pitch_angle = -trajectory::SolveTrajectory(p_a.x, p_a.y, p_a. z) * 180.0 / CV_PI;
    double yaw_angle = std::atan2(p_a.y, p_a.x) * 180.0 / CV_PI;

    // gimbal.set(pitch_angle, yaw_angle, firing, not_patrol);
    allowFire(pitch_angle, yaw_angle, firing, not_patrol, 2);
}

void targetCallback(detector::Target tar) {
    if(tar.status == "TRACKING") {
        double yaw = tar.yaw, v_yaw = tar.v_yaw, r1 = tar.radius_1, r2 = tar.radius_2;
        double xc = tar.position.x, yc = tar.position.y, zc = tar.position.z;
        double vx = tar.velocity.x, vy = tar.velocity.y, vz = tar.velocity.z;
        double dz = tar.dz;
        size_t a_n = tar.armors_num;
        lost_cnt = 0;

        ROS_INFO("v_yaw: %lf", v_yaw);

        // linear_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);

        if (std::fabs(v_yaw) < threshold_low_yaw) {
            ROS_INFO("linear_predict");
            linear_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
        }
        else if (std::fabs(v_yaw) > threshold_upp_yaw) {
            ROS_INFO("fast_spin_predict");
            fast_spin_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
        }
        else {
            ROS_INFO("slow_spin_predict");
            slow_spin_predict(yaw, v_yaw, r1, r2, xc, yc, zc, vx, vy, vz, dz, a_n);
        }
    }
    else if(tar.status == "DETECTING") {
        gimbal.set(gimbal.cur_pitch(), gimbal.cur_yaw(), not_firing, not_patrol);
    }
    else if(tar.status == "LOST") {
        if (lost_cnt > 700) { 
            gimbal.set(gimbal.cur_pitch(), gimbal.cur_yaw(), not_firing, patrol);
        }
        else {
            gimbal.set(gimbal.cur_pitch(), gimbal.cur_yaw(), not_firing, not_patrol);
            lost_cnt ++;
        }
    }
}

int main(int argc, char *argv[]) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tf_node");
    ros::NodeHandle nh;
    
    srand(time(NULL));

    read_Params();

    ros::Subscriber tar_sub = nh.subscribe("/tracker/target", 10, targetCallback);

    ros::Rate rate(500); 
    while(ros::ok()) {
        detector::GimbalOrientation gimbal_msg;
        gimbal_msg.pitch = gimbal.cur_pitch() / 180.0 * PI;
        gimbal_msg.yaw = gimbal.cur_yaw() / 180.0 * PI;
        // ROS_INFO("Pitch: %lf, Yaw: %lf", gimbal.cur_pitch(), gimbal.cur_yaw());
        broadcastTF(0.0, gimbal_msg.pitch, gimbal_msg.yaw, ros::Time::now());
        rate.sleep();
        ros::spinOnce();
    }
}
