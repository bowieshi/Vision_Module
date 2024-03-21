#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>

// ROS
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// local
#include "detector.hpp"
#include "number_classifier.hpp"
#include "armor.hpp"


class ArmorDetectorNode
{
public:
    ArmorDetectorNode(ros::NodeHandle &nh);
    std::vector<Armor> detectArmors(const cv::Mat & img_msg);
  
private:
    std::unique_ptr<Detector> initDetector();

    // Armor Detector
    std::unique_ptr<Detector> detector_;

    // Debug information
    bool debug;
    ros::Publisher binary_img_pub;
    ros::Publisher annotated_img_pub;
    // ros::Publisher result_img_pub;
};

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
