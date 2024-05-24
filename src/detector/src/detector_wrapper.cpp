#include "detector_wrapper.hpp"
#include "armor.hpp"
#include <chrono>

ArmorDetectorNode::ArmorDetectorNode(ros::NodeHandle &nh)
{
    binary_img_pub = nh.advertise<sensor_msgs::Image>("/detector/binary_img", 10);
    annotated_img_pub = nh.advertise<sensor_msgs::Image>("/detector/annotated_img", 10);
    // result_img_pub = nh.advertise<sensor_msgs::Image>("/detector/result_img", 10);

    // Detector
    detector_ = initDetector();
    debug = false;
}   

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
    int binary_thres = 100;

    auto detect_color = RED;

    Detector::LightParams l_params = {
        .min_ratio = 0.001,
        .max_ratio = 0.5,
        .max_angle = 40.0};

    Detector::ArmorParams a_params = {
        .min_light_ratio = 0.6,
        .min_small_center_distance = 0.8,
        .max_small_center_distance = 4.0,
        .min_large_center_distance = 3.2,
        .max_large_center_distance = 5.5,
        .max_angle = 20};

    auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

    // Init classifier
    std::string pkg_path = "/home/infantry_1/catkin_ws/src/detector";
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = 0.7;
    std::vector<std::string> ignore_classes = std::vector<std::string>{"negative"};
    detector->classifier =
        std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(const cv::Mat & img)
{

    // detect all the possible armors
    auto armors = detector_->detect(img);

    std::vector<Armor> filtered_armors;

    // filter out undesirable armors
    for (auto armor : armors) {
        std::cout << "Armor id: " << armor.id << std::endl;
        if (armor.id == 9)
            continue;
        filtered_armors.push_back(armor);
    }

    // publish debug info
    if(debug) {
        // use two crossing lines to annotate each armor
        for (auto armor : filtered_armors)
        {
            cv::Point p1 = armor.left_light.top;
            cv::Point p2 = armor.right_light.bottom;
            cv::Point p3 = armor.left_light.bottom;
            cv::Point p4 = armor.right_light.top;
            cv::Scalar colorLine(0, 255, 0); // Green - (B, G, R)
            int thicknessLine = 2;
            cv::line(img, p1, p2 ,colorLine, thicknessLine);
            cv::line(img, p3, p4 ,colorLine, thicknessLine);
        }

        // Draw cross
        cv::Point center(img.cols / 2, img.rows / 2);

        cv::Point horizontalStart(center.x - 50, center.y);
        cv::Point horizontalEnd(center.x + 50, center.y);
        cv::Point verticalStart(center.x, center.y - 50);
        cv::Point verticalEnd(center.x, center.y + 50);

        cv::line(img, horizontalStart, horizontalEnd, cv::Scalar(0, 255, 0), 2);
        cv::line(img, verticalStart, verticalEnd, cv::Scalar(0, 255, 0), 2);
        // show annotated image
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();  // Set the timestamp
        cv_image.encoding = sensor_msgs::image_encodings::RGB8;  // Set the encoding (e.g., BGR8 for color image)
        cv_image.image = img;
        sensor_msgs::ImagePtr ros_image = cv_image.toImageMsg();
        annotated_img_pub.publish(ros_image);

        // show binarinized image
        cv_image.header.stamp = ros::Time::now();  // Set the timestamp
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;  // Set the encoding (e.g., BGR8 for color image)
        cv_image.image = detector_->binary_img;
        ros_image = cv_image.toImageMsg();
        binary_img_pub.publish(ros_image);
    }

    return filtered_armors;
}