#include <iostream>
#include <opencv2/core.hpp>
#include <cmath>
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "tracker_wrapper.hpp"
#include "geometry_msgs/Pose.h"
#include "armor.hpp"

class CoordinateTransformer {
public:
    CoordinateTransformer();
    geometry_msgs::Pose transform(Armor &camera_armors, ros::Time timestamp);

private:
    void pnp(const cv::Point2f p[4], ArmorType armor_type, Eigen::Matrix3d &camera_orient, Eigen::Vector3d &camera_posi);
    Eigen::Matrix3d R_CI;   // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    // R_CI << ;
    cv::Mat R_CI_CV_MAT;
    cv::Mat F_CV_MAT;          // 相机内参矩阵CV-Mat
    cv::Mat C_CV_MAT;          // 相机畸变矩阵CV-Mat
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
};