#include "controller/lqr_run_node.h"

LQRController::LQRController(ros::NodeHandle& nh) 
    : nh_(nh), x_(0.0), y_(0.0), yaw_(0.0), v_(0.1), e_cg_(0.0), theta_e_(0.0), steer_(0.0),
      gear_drive_(true), ind_old_(0), path_received_(false), is_initialized_(false),
      current_gear_point_idx_(0), waiting_for_stop_(false), stop_start_time_(0.0),
      lqr_core_(wheelbase_, ts_, max_iteration_, eps_) {
    
    path_sub_ = nh_.subscribe("parking_run/searched_path", 1, &LQRController::pathCallback, this);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory", 1);
    // vehicle_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/vehicle_marker", 1);
    vehicle_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_marker", 1);

    traj_path_.header.frame_id = "map";
}

void LQRController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    ref_x_.clear();
    ref_y_.clear();
    ref_yaw_.clear();
    ref_k_.clear();
    gear_change_points_.clear();
    gear_switched_.clear();

    for (const auto& pose : msg->poses) {
        ref_x_.push_back(pose.pose.position.x);
        ref_y_.push_back(pose.pose.position.y);
        double yaw = tf2::getYaw(pose.pose.orientation);
        ref_yaw_.push_back(yaw);
    }
    
    // for (int i = 0; i < 16; ++i) {
    //     ref_x_.push_back(ref_x_.back() + 0.05);
    //     ref_y_.push_back(ref_y_.back());
    //     ref_yaw_.push_back(ref_yaw_.back());
    // }

    detectGearChangePoints();

    std::vector<int> divide_path(ref_x_.size(), 1);
    int current_sign = 1;
    for (size_t i = 0; i < ref_x_.size(); ++i) {
        for (size_t j = 0; j < gear_change_points_.size(); ++j) {
            if (i == gear_change_points_[j]) {
                current_sign = -current_sign;
                break;
            }
        }
        divide_path[i] = current_sign;
    }

    lqr_core_.computePathCurvature(ref_x_, ref_y_, ref_k_, divide_path);
    
    path_received_ = true;
    ind_old_ = 0;
    current_gear_point_idx_ = 0;
    waiting_for_stop_ = false;

    if (!is_initialized_ && !ref_x_.empty()) {
        x_ = ref_x_[0];
        y_ = ref_y_[0];
        yaw_ = ref_yaw_[0];
        v_ = 0.1;
        e_cg_ = 0.0;
        theta_e_ = 0.0;
        steer_ = 0.0;
        is_initialized_ = true;
    }

    ROS_INFO("Detected %lu gear change points:", gear_change_points_.size());
    for (size_t i = 0; i < gear_change_points_.size(); ++i) {
        size_t point_idx = gear_change_points_[i];
        ROS_INFO("  Gear change point %lu at index %lu: (%.2f, %.2f)", 
                 i, point_idx, ref_x_[point_idx], ref_y_[point_idx]);
    }
}

void LQRController::detectGearChangePoints() {
    if (ref_x_.size() < 3) return;

    const double direction_change_threshold = M_PI / 2;
    const double min_segment_length = 1.0;

    for (size_t i = 1; i < ref_x_.size() - 1; ++i) {
        double dx1 = ref_x_[i] - ref_x_[i-1];
        double dy1 = ref_y_[i] - ref_y_[i-1];
        double dx2 = ref_x_[i+1] - ref_x_[i];
        double dy2 = ref_y_[i+1] - ref_y_[i];

        double dot_product = dx1 * dx2 + dy1 * dy2;
        double mag1 = std::hypot(dx1, dy1);
        double mag2 = std::hypot(dx2, dy2);
        
        if (mag1 > 1e-6 && mag2 > 1e-6) {
            double cos_angle = dot_product / (mag1 * mag2);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            double angle = std::acos(cos_angle);
            
            if (angle > direction_change_threshold) {
                bool valid_point = true;
                if (!gear_change_points_.empty()) {
                    size_t last_point = gear_change_points_.back();
                    double dist = std::hypot(ref_x_[i] - ref_x_[last_point], 
                                           ref_y_[i] - ref_y_[last_point]);
                    if (dist < min_segment_length) {
                        valid_point = false;
                    }
                }
                
                if (valid_point) {
                    gear_change_points_.push_back(i);
                    gear_switched_[i] = false;
                }
            }
        }
    }
}

bool LQRController::shouldChangeGear() {
    if (current_gear_point_idx_ >= gear_change_points_.size()) {
        return false;
    }

    size_t next_gear_point = gear_change_points_[current_gear_point_idx_];
    
    if (ind_old_ >= next_gear_point || 
        (ind_old_ + 1 < ref_x_.size() && 
         std::hypot(x_ - ref_x_[next_gear_point], y_ - ref_y_[next_gear_point]) < 0.1)) {
        
        return !gear_switched_[next_gear_point];
    }
    
    return false;
}

void LQRController::handleGearChange() {
    if (current_gear_point_idx_ >= gear_change_points_.size()) {
        return;
    }

    size_t gear_point = gear_change_points_[current_gear_point_idx_];
    
    if (!waiting_for_stop_) {
        waiting_for_stop_ = true;
        stop_start_time_ = ros::Time::now().toSec();
        ROS_INFO("Starting to stop for gear change at point %lu", gear_point);
    } else {
        double current_time = ros::Time::now().toSec();
        bool stopped = (std::abs(v_) < 0.05);
        bool timeout = (current_time - stop_start_time_) > 2.0;
        
        if (stopped || timeout) {
            gear_drive_ = !gear_drive_;
            gear_switched_[gear_point] = true;
            waiting_for_stop_ = false;
            current_gear_point_idx_++;
            
            ROS_INFO("Gear changed to %s at point %lu", 
                     gear_drive_ ? "DRIVE" : "REVERSE", gear_point);
        }
    }
}

double LQRController::getDistanceToNextGearPoint() {
    if (current_gear_point_idx_ >= gear_change_points_.size()) {
        return std::hypot(x_ - ref_x_.back(), y_ - ref_y_.back());
    }
    
    size_t next_gear_point = gear_change_points_[current_gear_point_idx_];
    return std::hypot(x_ - ref_x_[next_gear_point], y_ - ref_y_[next_gear_point]);
}

void LQRController::publishVisualization() {
    // 轨迹发布（保持不变）
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    traj_path_.poses.push_back(pose);
    traj_path_.header.stamp = ros::Time::now();
    traj_pub_.publish(traj_path_);

    // 创建MarkerArray来统一发布所有markers
    visualization_msgs::MarkerArray marker_array;
    ros::Time current_time = ros::Time::now();

    // 车辆主体
    visualization_msgs::Marker vehicle;
    vehicle.header.frame_id = "map";
    vehicle.header.stamp = current_time;  // 使用统一的时间戳
    vehicle.ns = "vehicle";  // 统一使用vehicle命名空间
    vehicle.id = 0;
    vehicle.type = visualization_msgs::Marker::CUBE;
    vehicle.action = visualization_msgs::Marker::ADD;

    double rear_x = x_ + l_r_ * cos(yaw_);
    double rear_y = y_ + l_r_ * sin(yaw_);
    vehicle.pose.position.x = rear_x;
    vehicle.pose.position.y = rear_y;
    vehicle.pose.position.z = 0.0;
    vehicle.pose.orientation = pose.pose.orientation;

    vehicle.scale.x = r_f_ + r_b_;
    vehicle.scale.y = v_w_;
    vehicle.scale.z = 0.5;
    
    if (gear_drive_) {
        vehicle.color.r = 0.0; vehicle.color.g = 0.8; vehicle.color.b = 0.0;
    } else {
        vehicle.color.r = 0.8; vehicle.color.g = 0.0; vehicle.color.b = 0.0;
    }
    vehicle.color.a = 0.3;

    // 方向箭头
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = current_time;
    arrow.ns = "vehicle";  // 改为相同的命名空间
    arrow.id = 1;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 0.8 * wheelbase_;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;
    arrow.pose.position.x = x_;
    arrow.pose.position.y = y_;
    arrow.pose.position.z = 0.1;  // 稍微抬高避免重叠
    arrow.pose.orientation = pose.pose.orientation;

    // 四个轮子
    visualization_msgs::Marker wheel_fl, wheel_fr, wheel_rl, wheel_rr;
    
    // 统一设置轮子的基本属性
    std::vector<visualization_msgs::Marker*> wheels = {&wheel_fl, &wheel_fr, &wheel_rl, &wheel_rr};
    for (int i = 0; i < static_cast<int>(wheels.size()); i++) {
        wheels[i]->header.frame_id = "map";
        wheels[i]->header.stamp = current_time;
        wheels[i]->ns = "vehicle";  // 统一命名空间
        wheels[i]->id = 2 + i;  // ID: 2, 3, 4, 5
        wheels[i]->type = visualization_msgs::Marker::CUBE;
        wheels[i]->action = visualization_msgs::Marker::ADD;
        
        // 轮子尺寸
        wheels[i]->scale.x = 2 * t_r_;  // 轮胎直径
        wheels[i]->scale.y = t_w_;      // 轮胎宽度
        wheels[i]->scale.z = 0.1;       // 轮胎厚度
        
        // 轮子颜色
        wheels[i]->color.r = 0.0;
        wheels[i]->color.g = 0.0;
        wheels[i]->color.b = 1.0;
        wheels[i]->color.a = 1.0;
    }

    // 计算轮子位置
    double half_wheeldist = wheeldist_ / 2.0;
    double front_axle_x = x_ + (l_r_ + l_f_) * cos(yaw_);
    double front_axle_y = y_ + (l_r_ + l_f_) * sin(yaw_);

    tf2::Quaternion front_wheel_q;
    front_wheel_q.setRPY(0, 0, yaw_ + steer_);

    // 前左轮
    wheel_fl.pose.position.x = front_axle_x + half_wheeldist * sin(yaw_);
    wheel_fl.pose.position.y = front_axle_y - half_wheeldist * cos(yaw_);
    wheel_fl.pose.position.z = 0.0;
    // wheel_fl.pose.orientation = pose.pose.orientation;  // 前轮与车身同向（简化处理）
    wheel_fl.pose.orientation.x = front_wheel_q.x();
    wheel_fl.pose.orientation.y = front_wheel_q.y();
    wheel_fl.pose.orientation.z = front_wheel_q.z();
    wheel_fl.pose.orientation.w = front_wheel_q.w();
    
    // 前右轮
    wheel_fr.pose.position.x = front_axle_x - half_wheeldist * sin(yaw_);
    wheel_fr.pose.position.y = front_axle_y + half_wheeldist * cos(yaw_);
    wheel_fr.pose.position.z = 0.0;
    wheel_fr.pose.orientation = wheel_fl.pose.orientation;

    // 后左轮
    wheel_rl.pose.position.x = x_ + half_wheeldist * sin(yaw_);
    wheel_rl.pose.position.y = y_ - half_wheeldist * cos(yaw_);
    wheel_rl.pose.position.z = 0.0;
    wheel_rl.pose.orientation = pose.pose.orientation;

    // 后右轮（修复原代码中的bug）
    wheel_rr.pose.position.x = x_ - half_wheeldist * sin(yaw_);
    wheel_rr.pose.position.y = y_ + half_wheeldist * cos(yaw_);
    wheel_rr.pose.position.z = 0.0;
    wheel_rr.pose.orientation = pose.pose.orientation;

    // 添加所有markers到数组
    marker_array.markers.push_back(vehicle);
    marker_array.markers.push_back(arrow);
    marker_array.markers.push_back(wheel_fl);
    marker_array.markers.push_back(wheel_fr);
    marker_array.markers.push_back(wheel_rl);
    marker_array.markers.push_back(wheel_rr);

    // 统一发布所有markers
    vehicle_marker_pub_.publish(marker_array);
}

void LQRController::run() {
    ros::Rate rate(1.0 / ts_);
    double max_time = 100.0;
    double t = 0.0;

    while (ros::ok() && t < max_time) {
        if (!path_received_ || !is_initialized_) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        if (shouldChangeGear()) {
            handleGearChange();
        }

        double dist = std::hypot(x_ - ref_x_.back(), y_ - ref_y_.back());
        double target_speed = gear_drive_ ? 4.0 / 3.6 : 2.0 / 3.6;

        double delta = lqr_core_.computeLateralControl(x_, y_, yaw_, v_, ref_x_, ref_y_, ref_yaw_, 
                                                     ref_k_, ind_old_, e_cg_, theta_e_);
        double a = lqr_core_.computeLongitudinalControl(target_speed, dist, gear_drive_, 
                                                      waiting_for_stop_, v_, 
                                                      getDistanceToNextGearPoint(), shouldChangeGear());

        lqr_core_.updateState(x_, y_, yaw_, v_, delta, a, gear_drive_, 
                             max_steer_angle_, max_acceleration_, max_speed_);
        steer_ = delta;

        publishVisualization();

        if (dist <= 0.1 && current_gear_point_idx_ >= gear_change_points_.size()) {
            ROS_INFO("Reached final destination!");
            break;
        }

        t += ts_;
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lqr_controller_node");
    ros::NodeHandle nh;
    LQRController controller(nh);
    controller.run();
    return 0;
}