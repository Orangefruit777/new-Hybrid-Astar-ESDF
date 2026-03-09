#include "hybrid_a_star/hybrid_parking_opt.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

HybridParkingOpt::HybridParkingOpt(ros::NodeHandle& nh) {
    std::cout << "Hybrid A* Start" << std::endl;
    // config_ = std::make_shared<Config>(nh);
    config_ = std::make_shared<Config>();
    std::cout << "Config complete" << std::endl;
    gridMap_.setParam(config_, nh);

    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);
    double w_ref = nh.param("planner/w_ref", 0.0);
    use_dubins_or_rs_ = nh.param("planner/use_dubins_or_rs", 0);

    hybrid_astar_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    opt_path_pub_ = nh.advertise<nav_msgs::Path>("opt_path", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);


    obstacle_points_sub_ = nh.subscribe("/obstacle_points", 10, &HybridParkingOpt::ObstaclePointsCallback, this);
    has_map_ = false;
}

void HybridParkingOpt::ObstaclePointsCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    obstacle_points_deque_.push_back(point_cloud_msg);
}

void HybridParkingOpt::Run() {
    ReadData();

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = static_cast<float>(current_costmap_ptr_->info.resolution);

        double x_lower = current_costmap_ptr_->info.origin.position.x;
        double x_upper = x_lower + current_costmap_ptr_->info.width * map_resolution;
        double y_lower = current_costmap_ptr_->info.origin.position.y;
        double y_upper = y_lower + current_costmap_ptr_->info.height * map_resolution;

        x_offset_ = x_lower;
        y_offset_ = y_lower;
        grid_size_x_ = current_costmap_ptr_->info.width;
        grid_size_y_ = current_costmap_ptr_->info.height;
        std::cout << "x_lower: " << x_lower << " x_upper: " << x_upper << " y_lower: " << y_lower << " y_upper: " << y_upper << std::endl;

        // hybrid_astar地图范围为坐标值，且必须从(0, 0)开始
        hybrid_astar_ptr_->Init(0, x_upper - x_lower, 0, x_upper - x_lower, 1.0, map_resolution);

        unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w]) {
                    hybrid_astar_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }
    // 处理障碍物点云
    while (!obstacle_points_deque_.empty()) {
        auto point_cloud_msg = obstacle_points_deque_.front();
        obstacle_points_deque_.pop_front();

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // 将每个点设置为障碍物
        for (const auto& point : cloud->points) {
            hybrid_astar_ptr_->SetObstacle(point.x - x_offset_, point.y - y_offset_);
        }
    }
    costmap_deque_.clear();

    while (HasStartPose() && HasGoalPose()) {
        InitPoseData();

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        // double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x - x_offset_,
                current_init_pose_ptr_->pose.pose.position.y - y_offset_,
                start_yaw
        );
        // Vec3d goal_state = Vec3d(
        //         current_goal_pose_ptr_->pose.position.x - x_offset_,
        //         current_goal_pose_ptr_->pose.position.y - y_offset_,
        //         goal_yaw
        // );

        Vec3d goal_state = Vec3d(11.9, 8.65, 3.1416);
        std::cout << "start_state: " << start_state.transpose() << std::endl;
        std::cout << "goal_state: " << goal_state.transpose() << std::endl;
        ros::Time hybrid_start = ros::Time::now();
        if (hybrid_astar_ptr_->Search(start_state, goal_state, use_dubins_or_rs_)) {
            auto path = hybrid_astar_ptr_->GetPath();
            ros::Time hybrid_end = ros::Time::now();
            std::cout << "\033[1;32mhybrid astar time : " << (hybrid_end - hybrid_start).toSec() * 1000 << " ms\033[0m" << std::endl;
            
            // 将路径点坐标系转换回全局坐标系
            for(auto& state : path){
                state.x() += x_offset_;
                state.y() += y_offset_;
            }
            auto last_state = path.back();
            PublishPath(path);

            // 轨迹优化
            path_searching::KinoTrajData nn_trajs_;
            double totalTrajTime;
                
            hybrid_astar_ptr_->GetKinoTraj(path, 2.5, 2.5, 1.0, 1.0, 1.0, nn_trajs_, totalTrajTime);

            int segnum = nn_trajs_.size();

            std::cout << "nn path size: " << segnum << std::endl;

            std::vector<int> refined_singuals; refined_singuals.resize(segnum);
            Eigen::VectorXd refined_rt; refined_rt.resize(segnum);//uniform piece-wise polynomial
            std::vector<Eigen::MatrixXd> refined_inPs_container; refined_inPs_container.resize(segnum);
            std::vector<Eigen::Vector2d> refined_gearPos;refined_gearPos.resize(segnum - 1);
            std::vector<double> refined_angles; refined_angles.resize(segnum - 1);
            double basetime = 0.0;
            for(int i = 0; i < segnum; i++){
                double timePerPiece = 1.0;
                path_searching::FlatTrajData nn_traj = nn_trajs_.at(i);
                refined_singuals[i] = nn_traj.singul;

                std::cout << "nn_traj.signal: " << nn_traj.singul << std::endl;

                int piece_nums;
                double initTotalduration = nn_traj.duration;//hzchzc

                std::cout << "initTotalduration: " << initTotalduration << std::endl;

                piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),1);
                double dt = initTotalduration / piece_nums; 
                refined_rt[i] = (initTotalduration / piece_nums);
                refined_inPs_container[i].resize(2, piece_nums - 1);
                for(int j = 0; j < piece_nums - 1; j++ ){
                    double t = basetime + (j+1)*dt;
                    Eigen::Vector3d pos = hybrid_astar_ptr_->evaluatePos(t);
                    refined_inPs_container[i].col(j) = pos.head(2);
                }
                if(i >=1){
                    Eigen::Vector3d pos = hybrid_astar_ptr_->evaluatePos(basetime);
                    refined_gearPos[i-1] = pos.head(2);
                    refined_angles[i-1] = pos[2];
                }
                basetime += initTotalduration;
            }
            iniState2d << start_state[0], refined_singuals[0] * cos(start_state[2]), 0.0,
                        start_state[1], refined_singuals[0] * sin(start_state[2]), 0.0;

            finState2d << goal_state[0], refined_singuals[segnum-1] * cos(goal_state[2]), 0.0,
                        goal_state[1], refined_singuals[segnum-1] * sin(goal_state[2]), 0.0;

            PolyTrajOpt::TrajOpt refinedOpt;

            ros::Time t3 = ros::Time::now();
            int flagSs = refinedOpt.OptimizeSe2Trajectory(
            iniState2d, finState2d, refined_rt,
            refined_inPs_container, refined_gearPos,
            refined_angles, &gridMap_,  config_, refined_singuals); //vis_tool
            ros::Time t4 = ros::Time::now();
            std::cout << "OptimizeSe2Trajectory time: " << (t4 - t3).toSec() *1000 << "ms" << std::endl;

            auto optTraj = refinedOpt.getTraj(0.1);

            PublishSmoothPath(optTraj);
            
            PublishVehiclePath(optTraj, 2.0, 1.0, 5u);

            VehicleMotionPath(optTraj, 0.03);

        }
        else{
            // 打印红色提示
            std::cout << "\033[31m No path found! \033[0m" << std::endl;
        }
        hybrid_astar_ptr_->Reset();
    }
}

void HybridParkingOpt::VehicleMotionPath(const VectorVec3d &path, double sleep_duration) {
    nav_msgs::Path path_ros;
    geometry_msgs::PoseStamped pose_stamped;

    // 修改车辆行驶路径， path or quintic_path
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

        path_ros.poses.emplace_back(pose_stamped);
    }

    path_ros.header.frame_id = "world";
    path_ros.header.stamp = ros::Time::now();
    static tf::TransformBroadcaster transform_broadcaster;
    for (const auto &pose: path_ros.poses) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

        tf::Quaternion q;
        q.setX(pose.pose.orientation.x);
        q.setY(pose.pose.orientation.y);
        q.setZ(pose.pose.orientation.z);
        q.setW(pose.pose.orientation.w);
        transform.setRotation(q);

        transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                                ros::Time::now(), "world",
                                                                "ground_link")
        );

        ros::Duration(sleep_duration).sleep();
        // ros::Duration(0.10).sleep();
    }
}
void HybridParkingOpt::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridParkingOpt::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridParkingOpt::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridParkingOpt::HasStartPose() {
    return !init_pose_deque_.empty();
}

void HybridParkingOpt::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void HybridParkingOpt::PublishSmoothPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    opt_path_pub_.publish(nav_path);
}

void HybridParkingOpt::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 3u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}