#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"
#include "util/costmap_subscriber.h"
#include "util/init_pose_subscriber.h"
#include "util/goal_pose_subscriber.h"

#include <sensor_msgs/PointCloud2.h> // 使用 PointCloud2
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "plan_env/sdf_map_2d.h"
#include "util/smooth.h"
#include "util/quintic_spline.h"

#include "util/kino_model.hpp"
#include <tools/config.hpp>
#include <tools/gridmap.hpp>
// #include <tools/CorridorBuilder2d.hpp>
#include "MINCO_SE2_opt/Trajopt_alm.hpp"

#include <string>
#include <memory>

#include <ros/ros.h>

class HybridParkingOpt {
public:
    HybridParkingOpt() = default;

    explicit HybridParkingOpt(ros::NodeHandle &nh);

    void Run();

private:
    void InitPoseData();

    void ReadData();

    bool HasStartPose();

    bool HasGoalPose();

    // 新增点云回调函数
    void ObstaclePointsCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

    void PublishPath(const VectorVec3d &path);

    void PublishSmoothPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    void VehicleMotionPath(const VectorVec3d &path, double sleep_duration);

private:
    std::shared_ptr<Config> config_;
    map_util::OccMapUtil gridMap_;

    std::shared_ptr<HybridAStar> hybrid_astar_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;

    ros::Publisher path_pub_;
    ros::Publisher opt_path_pub_;
    ros::Publisher vehicle_path_pub_;

    Eigen::Matrix<double, 2, 3> iniState2d, finState2d;

    // 新增点云订阅器
    ros::Subscriber obstacle_points_sub_;
    std::deque<sensor_msgs::PointCloud2ConstPtr> obstacle_points_deque_;

    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    bool has_map_;

    double x_offset_, y_offset_;
    int grid_size_x_, grid_size_y_;

    int use_dubins_or_rs_; // 0: dubins, 1: rs
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
