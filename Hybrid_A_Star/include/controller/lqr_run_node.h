#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <map>
#include "lqr_core.h"

class LQRController {
public:
    LQRController(ros::NodeHandle& nh);
    void run();

private:
    // 车辆参数
    const double wheelbase_ = 1.0;        // 轴距 [m]
    const double wheeldist_ = 0.8;        // 轮距 [m]
    const double v_w_ = 1.0;              // 车辆宽度 [m]
    const double r_b_ = 0.5;              // 后轴到车尾 [m]
    const double r_f_ = 1.5;              // 后轴到车头 [m]
    const double t_r_ = 0.15;              // 轮胎半径 [m]
    const double t_w_ = 0.15;              // 轮胎宽度 [m]
    const double l_f_ = 0.5;              // 质心到前轴 [m]
    const double l_r_ = 0.5;              // 质心到后轴 [m]

    // 控制器参数
    const double ts_ = 0.1;               // 时间步长 [s]
    const int max_iteration_ = 150;        // LQR 最大迭代次数
    const double eps_ = 0.03;             // LQR 收敛容差
    const double max_acceleration_ = 5.0; // 最大加速度 [m/s^2]
    const double max_steer_angle_ = 80.0 * M_PI / 180.0; // 最大转向角 [rad]
    const double max_speed_ = 30.0 / 3.6; // 最大速度 [m/s]

    // ROS 相关
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher traj_pub_, vehicle_marker_pub_;
    nav_msgs::Path traj_path_;  // 车辆轨迹

    // 车辆状态
    double x_, y_, yaw_, v_;    // 位置、航向、速度
    double e_cg_, theta_e_;     // 横向误差、航向误差
    double steer_;              // 转向角
    bool gear_drive_;           // 档位（true: 前进, false: 倒车）   

    // 参考路径
    std::vector<double> ref_x_, ref_y_, ref_yaw_, ref_k_;
    size_t ind_old_;  // 最近点索引
    bool path_received_;  // 是否收到路径

    // 换挡相关变量
    std::vector<size_t> gear_change_points_;  // 换挡点索引
    std::map<size_t, bool> gear_switched_;    // 记录每个换挡点是否已切换
    bool is_initialized_;                     // 是否已初始化车辆位置
    size_t current_gear_point_idx_;           // 当前换挡点索引
    bool waiting_for_stop_;                   // 是否正在等待停车换挡
    double stop_start_time_;                  // 开始停车的时间

    // LQR核心算法
    LQRCore lqr_core_;

    // 方法
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void publishVisualization();
    void detectGearChangePoints();
    bool shouldChangeGear();
    void handleGearChange();
    double getDistanceToNextGearPoint();
};

#endif