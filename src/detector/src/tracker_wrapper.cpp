#include "tracker_wrapper.hpp"

TrackerWrapper::TrackerWrapper(ros::NodeHandle &nh) : tracker(max_match_distance, max_match_yaw_diff) {
    auto f = [this](const Eigen::VectorXd & x) {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt;
        x_new(2) += x(3) * dt;
        x_new(4) += x(5) * dt;
        x_new(6) += x(7) * dt;
        return x_new;
    };
    // J_f - Jacobian of process function
    auto j_f = [this](const Eigen::VectorXd &) {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
        f << 1,   dt,  0,   0,   0,   0,   0,   0,   0,
             0,   1,   0,   0,   0,   0,   0,   0,   0,
             0,   0,   1,   dt,  0,   0,   0,   0,   0, 
             0,   0,   0,   1,   0,   0,   0,   0,   0,
             0,   0,   0,   0,   1,   dt,  0,   0,   0,
             0,   0,   0,   0,   0,   1,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   1,   dt,  0,
             0,   0,   0,   0,   0,   0,   0,   1,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   1;
        return f;
    };
    // h - Observation function
    auto h = [](const Eigen::VectorXd & x) {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = xc - r * cos(yaw);  // xa
        z(1) = yc - r * sin(yaw);  // ya
        z(2) = x(4);               // za
        z(3) = x(6);               // yaw
        return z;
    };
  // J_h - Jacobian of observation function
    auto j_h = [](const Eigen::VectorXd & x) {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
        //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
        h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
            0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
            0,   0,   0,   0,   1,   0,   0,          0,   0,
            0,   0,   0,   0,   0,   0,   1,          0,   0;
        // clang-format on
        return h;
    };

    // update_Q - process noise covariance matrix
    // ekf.sigma2_q_xyz
    s2qxyz = 20.0;    
    // ekf.sigma2_q_yaw
    s2qyaw = 100.0;
    // ekf.sigma2_q_r
    s2qr = 800.0;
    auto u_q = [this]() {
        Eigen::MatrixXd q(9, 9);
        double t = dt, x = s2qxyz, y = s2qyaw, r = s2qr;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
            q_x_vx, q_vx_vx, 0,      0,      0,      0,      0,      0,      0,
            0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
            0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
            0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
            0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
            0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
            0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
            0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        // clang-format on
        return q;
    };
    // update_R - measurement noise covariance matrix
    // ekf.r_xyz_factor
    r_xyz_factor = 0.05;
    // ekf.r_yaw
    r_yaw = 0.02;
    auto u_r = [this](const Eigen::VectorXd & z) {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();
    max_match_distance = 0.15, max_match_yaw_diff = 1.0;

    tracker.ekf = Extended_KF{f, h, j_f, j_h, u_q, u_r, p0};

    target_pub = nh.advertise<detector::Target>("/tracker/target", 10);

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker.ns = "position";
    position_marker.type = visualization_msgs::Marker::SPHERE;
    position_marker.scale.x = position_marker.scale.y = position_marker.scale.z = 0.1;
    position_marker.color.a = 1.0;
    position_marker.color.g = 1.0;
    linear_v_marker.type = visualization_msgs::Marker::ARROW;
    linear_v_marker.ns = "linear_v";
    linear_v_marker.scale.x = 0.03;
    linear_v_marker.scale.y = 0.05;
    linear_v_marker.color.a = 1.0;
    linear_v_marker.color.r = 1.0;
    linear_v_marker.color.g = 1.0;
    angular_v_marker.type = visualization_msgs::Marker::ARROW;
    angular_v_marker.ns = "angular_v";
    angular_v_marker.scale.x = 0.03;
    angular_v_marker.scale.y = 0.05;
    angular_v_marker.color.a = 1.0;
    angular_v_marker.color.b = 1.0;
    angular_v_marker.color.g = 1.0;
    armor_marker.ns = "armors";
    armor_marker.type = visualization_msgs::Marker::CUBE;
    armor_marker.scale.x = 0.03;
    armor_marker.scale.z = 0.125;
    armor_marker.color.a = 1.0;
    armor_marker.color.r = 1.0;
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/tracker/marker", 10);
}

void TrackerWrapper::armorsCallback(const detector::Armors &armors_msg) {
    // Filter abnormal armors
    // armors_msg->armors.erase(
    //     std::remove_if(
    //     armors_msg->armors.begin(), armors_msg->armors.end(),
    //     [this](const auto_aim_interfaces::msg::Armor & armor) {
    //         return abs(armor.pose.position.z) > 1.2 ||
    //             Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
    //                 max_armor_distance_;
    //     }),
    //     armors_msg->armors.end());

    /*
        Init message
        std_msgs/Header header
        bool tracking
        string id
        int32 armors_num
        geometry_msgs/Point position
        geometry_msgs/Vector3 velocity
        float64 yaw
        float64 v_yaw
        float64 radius_1
        float64 radius_2
        float64 dz
    */
    if (tar_id == 0) {
        ROS_INFO("In patrol state");
        // Patrol state to do
        tracker.tracker_state = Tracker::LOST;
        return;
    }

    if(armors_msg.armors.size() == 0) {
        ROS_INFO("Waiting mode");
        // Waiting mode to do
        return;
    }
    
    detector::Target target_msg;
    ros::Time time = armors_msg.header.stamp;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = "world";
    // Update tracker
    if (tracker.tracker_state == Tracker::LOST) {
        ROS_INFO("Tracker LOST!!!!");
        tracker.init(armors_msg, tar_id);
        if (tracker.tracker_state != Tracker::LOST) {
            ROS_INFO("Changed!!");
        } else {
            ROS_INFO("Waiting mode");
            // Waiting mode to do
        }
        target_msg.tracking = false;
    }
    else {
        dt = (time - last_time).toSec();
        tracker.lost_thres = static_cast<int>(lost_time_thres / dt);
        tracker.update(armors_msg);

        // Publish Info
        // info_msg.position_diff = tracker_->info_position_diff;
        // info_msg.yaw_diff = tracker_->info_yaw_diff;
        // info_msg.position.x = tracker_->measurement(0);
        // info_msg.position.y = tracker_->measurement(1);
        // info_msg.position.z = tracker_->measurement(2);
        // info_msg.yaw = tracker_->measurement(3);
        // info_pub_->publish(info_msg);

        if (tracker.tracker_state == Tracker::DETECTING) {
            ROS_INFO("Tracker DETECTING!!!!\n");
            target_msg.tracking = false;
        }
        else if (tracker.tracker_state == Tracker::TRACKING || tracker.tracker_state == Tracker::TEMP_LOST) {
            ROS_INFO("Tracker TRACKING or TEMP_LOST\n");
            target_msg.tracking = true;
            // Fill target message
            const auto & state = tracker.target_state;
            target_msg.id = tracker.tracked_id;
            target_msg.armors_num = static_cast<int>(tracker.tracked_armors_num);
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw = state(6);
            target_msg.v_yaw = state(7);
            target_msg.radius_1 = state(8);
            target_msg.radius_2 = tracker.another_r;
            target_msg.dz = tracker.dz;
            target_pub.publish(target_msg);
            publishMarkers(target_msg);
        }
    }

    last_time = time;
}

void TrackerWrapper::publishMarkers(const detector::Target & target_msg) {
    position_marker.header = target_msg.header;
    linear_v_marker.header = target_msg.header;
    angular_v_marker.header = target_msg.header;
    armor_marker.header = target_msg.header;

    visualization_msgs::MarkerArray marker_array;
    if (target_msg.tracking) {
        double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
        double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
        double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
        double dz = target_msg.dz;

        position_marker.action = visualization_msgs::Marker::ADD;
        position_marker.pose.position.x = xc;
        position_marker.pose.position.y = yc;
        position_marker.pose.position.z = za + dz / 2;

        linear_v_marker.action = visualization_msgs::Marker::ADD;
        linear_v_marker.points.clear();
        linear_v_marker.points.emplace_back(position_marker.pose.position);
        geometry_msgs::Point arrow_end = position_marker.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker.points.emplace_back(arrow_end);

        angular_v_marker.action = visualization_msgs::Marker::ADD;
        angular_v_marker.points.clear();
        angular_v_marker.points.emplace_back(position_marker.pose.position);
        arrow_end = position_marker.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker.points.emplace_back(arrow_end);

        armor_marker.action = visualization_msgs::Marker::ADD;
        armor_marker.scale.y = tracker.tracked_armor.type == (int)ArmorType::SMALL ? 0.135 : 0.23;
        bool is_current_pair = true;
        size_t a_n = target_msg.armors_num;
        geometry_msgs::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            } else {
                r = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            armor_marker.id = i;
            armor_marker.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            armor_marker.pose.orientation.x = q.x();
            armor_marker.pose.orientation.y = q.y();
            armor_marker.pose.orientation.z = q.z();
            armor_marker.pose.orientation.w = q.w();
            // armor_marker.pose.orientation = tf2::toMsg(tf2::QuaternionMsg(q));
            marker_array.markers.emplace_back(armor_marker);
        }
    }
    else {
        position_marker.action = visualization_msgs::Marker::DELETE;
        linear_v_marker.action = visualization_msgs::Marker::DELETE;
        angular_v_marker.action = visualization_msgs::Marker::DELETE;

        armor_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.emplace_back(armor_marker);
    }

    marker_array.markers.emplace_back(position_marker);
    marker_array.markers.emplace_back(linear_v_marker);
    marker_array.markers.emplace_back(angular_v_marker);
    marker_pub.publish(marker_array);
}

void TrackerWrapper::setTargetId(int id) {
    tar_id = id;
    std::cout << "Set target id to " << tar_id << std::endl;
}