#include <memory>
#include <string>
#include <cfloat>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Eigen>

#include <angles/angles.h>

#include "armor.hpp"
#include "Extended_KF.hpp"
#include "detector/Armor.h"
#include "detector/Armors.h"

enum class ArmorId { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker {
public:
    Tracker(double max_match_distance, double max_match_yaw_diff);

    void init(const detector::Armors &armors_msg, int id);

    void update(const detector::Armors &armors_msg);

    Extended_KF ekf;

    int tracking_thres = 5;
    int lost_thres;

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state;

    int tracked_id;
    detector::Armor tracked_armor;
    ArmorId tracked_armors_num;

    double info_position_diff;
    double info_yaw_diff;

    Eigen::VectorXd measurement;

    Eigen::VectorXd target_state;

    // To store another pair of armors message
    double dz, another_r;

private:
    void initEKF(const detector::Armor & a);

    void updateArmorsNum(const detector::Armor & a);

    void handleArmorJump(const detector::Armor & a);

    double orientationToYaw(const geometry_msgs::Quaternion & q);

    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

    double max_match_distance_;
    double max_match_yaw_diff_;

    int detect_count_;
    int lost_count_;

    double last_yaw_;
};