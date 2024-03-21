#include "coord_transformer.hpp"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

CoordinateTransformer::CoordinateTransformer():
        listener(buffer),
        R_CI(Eigen::Matrix3d::Identity()),
        R_CI_CV_MAT(cv::Mat::eye(3, 3, CV_64F)),
        F_CV_MAT(cv::Mat::eye(3, 3, CV_64F)),
        C_CV_MAT(cv::Mat::zeros(5, 1, CV_64F)) {
    cv::FileStorage f_in("/home/hero/Vision_Module/src/detector/params/camera.yaml", cv::FileStorage::READ);
    f_in["K"] >> F_CV_MAT;
    f_in["D"] >> C_CV_MAT;
}

void CoordinateTransformer::pnp(const cv::Point2f p[4], ArmorType armor_type, Eigen::Matrix3d &camera_orient, Eigen::Vector3d &camera_posi) {
    // pre-defined armor cornor points coordinate in camera coordinate
    static const std::vector<cv::Point3d> pw_small = {  // Unit: m
            {-0.0675, 0.0275,  0.},  // bottom_left
            {-0.0675, -0.0275, 0.},  // top_left
            {0.0675,  -0.0275, 0.},  // top_right
            {0.0675,  0.0275,  0.}   // bottom_right
    };
    static const std::vector<cv::Point3d> pw_large = {    // Unit: m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };

    std::vector<cv::Point2d> pu(p, p + 4);
    cv::Mat rvec, tvec;

    assert((armor_type == ArmorType::SMALL || armor_type == ArmorType::LARGE) && "Invalid armor type");

    if (armor_type == ArmorType::SMALL)
        cv::solvePnP(pw_small, pu, F_CV_MAT, C_CV_MAT, rvec, tvec);
    else if (armor_type == ArmorType::LARGE)
        cv::solvePnP(pw_large, pu, F_CV_MAT, C_CV_MAT, rvec, tvec);
    
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::cv2eigen(R, camera_orient);
    cv::cv2eigen(tvec, camera_posi);
}

geometry_msgs::Pose CoordinateTransformer::transform(Armor &camera_armor, ros::Time timestamp) {
    std::vector<geometry_msgs::PoseStamped> world_armors;
    detector::Armors armors_msg;
    armors_msg.header.frame_id = "world";
    armors_msg.header.stamp = timestamp;

    cv::Point2f p[4];
    p[0] = camera_armor.left_light.bottom;
    p[1] = camera_armor.left_light.top;
    p[2] = camera_armor.right_light.top;
    p[3] = camera_armor.right_light.bottom;

    // here we use pnp to get the pose of armors
    Eigen::Matrix3d camera_orient; // armor position      in camera coordinate
    Eigen::Vector3d camera_posi;   // armor orientation   in camera coordinate
    pnp(p, camera_armor.type, camera_orient, camera_posi);

    // here we transform the coordiante from camera coordinante to world coordinante
    /*
        camera
        x: right
        y: down
        z: forward

        world
        x: forward
        y: left
        z: upward
    */
    geometry_msgs::PoseStamped camera_pose;

    camera_pose.header.frame_id = "camera";
    camera_pose.header.stamp = timestamp;

    camera_pose.pose.position.x = camera_posi[0];
    camera_pose.pose.position.y = camera_posi[1];
    camera_pose.pose.position.z = camera_posi[2];

    Eigen::Quaterniond quaternion(camera_orient);
    quaternion.normalize();

    camera_pose.pose.orientation.x = quaternion.x();
    camera_pose.pose.orientation.y = quaternion.y();
    camera_pose.pose.orientation.z = quaternion.z();
    camera_pose.pose.orientation.w = quaternion.w();

    geometry_msgs::PoseStamped world_pose;

    try {
        world_pose = buffer.transform(camera_pose,"world");

        detector::Armor armor_msg;
        // string number
        // string type
        // float32 distance_to_image_center
        // geometry_msgs/Pose pose

        armor_msg.id = camera_armor.id;
        armor_msg.type = (int)camera_armor.type;
        ROS_INFO("Armor type: %d", armor_msg.type);
        // if(camera_armor.type == ArmorType::SMALL)
        //     armor_msg.type = "small";
        // else if(camera_armor.type == ArmorType::LARGE)
        //     armor_msg.type = "large";
        // else
        //     armor_msg.type = "invalid";
        // armor_msg.distance_to_image_center = calculateDistanceToCenter(camera_armor.center);
        armor_msg.pose = world_pose.pose;
        armors_msg.armors.push_back(armor_msg);

        // std_msgs::Float64MultiArray info;
        // info.layout.dim.resize(1);
        // info.layout.dim[0].size = 1;
        // info.layout.dim[0].stride = 1;
        // info.data = {orientationToRoll(camera_pose.pose.orientation), orientationToPitch(camera_pose.pose.orientation), orientationToYaw(camera_pose.pose.orientation)};
        // state_pub.publish(info);
        
        // ROS_INFO("I want yaw: %f",orientationToYaw(world_pose.pose.orientation));
        // std::cout << "Camera coordinates:" << camera_pose.pose.position.x << " " << camera_pose.pose.position.y << " " << camera_pose.pose.position.z << std::endl;
        // std::cout << "Camera orientation:" << camera_pose.pose.orientation.x << " " << camera_pose.pose.orientation.y << " " << camera_pose.pose.orientation.z << " " << camera_pose.pose.orientation.w << std::endl;
        // std::cout << "World  coordinates:" << world_pose.pose.position.x << " " << world_pose.pose.position.y << " " << world_pose.pose.position.z << std::endl;
    }
    catch(const std::exception& e) {
        ROS_INFO("error: %s",e.what());
    }
    return world_pose.pose;
}