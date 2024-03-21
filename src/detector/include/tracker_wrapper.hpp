#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include <ros/duration.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/convert.h>
#include <geometry_msgs/Quaternion.h>

#include "tracker.hpp"
#include "detector/Target.h"

class TrackerWrapper {
public:
    explicit TrackerWrapper(ros::NodeHandle &nh);
    void armorsCallback(const detector::Armors &armors_msg);
    void setTargetId(int id);

private:
    // Maximum allowable armor distance in the XOY plane
    double max_armor_distance = 10.0;
    double max_match_distance = 0.15, max_match_yaw_diff = 1.0;         // 0.15, 1.0
    // The time when the last message was received
    ros::Time last_time;
    double dt;

    // Armor tracker
    double s2qxyz, s2qyaw, s2qr;
    double r_xyz_factor, r_yaw;
    double lost_time_thres = 0.3;       //0.3
    int tar_id;
    Tracker tracker;

    // Publisher
    ros::Publisher target_pub;

    // Visualization marker publisher
    visualization_msgs::Marker position_marker;
    visualization_msgs::Marker linear_v_marker;
    visualization_msgs::Marker angular_v_marker;
    visualization_msgs::Marker armor_marker;
    ros::Publisher marker_pub;
    void publishMarkers(const detector::Target & target_msg);
};