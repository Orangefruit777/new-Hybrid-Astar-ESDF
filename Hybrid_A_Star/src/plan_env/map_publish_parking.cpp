#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
class MapPublisher{
private:
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Publisher obstacle_points_pub_;
  ros::Publisher lane_points_pub_;
  double grid_resolution_;
  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;

public:
MapPublisher() {
  grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  obstacle_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1); // 初始化 PointCloud2 发布器
  lane_points_pub_ = nh_.advertise<nav_msgs::Path>("/lane_points", 1);
  grid_resolution_ = 0.05; // 10 cm resolution
  map_frame_ = "world";
  // Initialize obstacle point cloud
  initializeObstacleCloud();
}

// 添加矩形障碍物
// void addObstacleRectangle(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double x_min, double x_max, double y_min, double y_max, double step) {
//   for (double x = x_min; x <= x_max; x += step) {
//       for (double y = y_min; y <= y_max; y += step) {
//           cloud->points.emplace_back(x, y, 0.0);
//       }
//   }
// }
void initializeObstacleCloud() {
  // 创建 PCL 点云
  obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // 小车库
  double obstacle_interval = 0.1;
  for(double x = 10.8; x <= 13.3; x += obstacle_interval){
    obstacle_cloud_->points.emplace_back(x, 9.4, 0.0);
    obstacle_cloud_->points.emplace_back(x, 7.9, 0.0);
  }

  for(double y = 8.0; y <= 9.3; y += obstacle_interval){
    obstacle_cloud_->points.emplace_back(13.3, y, 0.0);
  }

  for(double y = 2.9; y <= 7.9; y += obstacle_interval){
    obstacle_cloud_->points.emplace_back(10.8, y, 0.0);
  }

  for(double y = 9.5; y <= 14.5; y += obstacle_interval){
    obstacle_cloud_->points.emplace_back(10.8, y, 0.0);
  }

  for(double y = 2.9; y <= 14.5; y += obstacle_interval){
    obstacle_cloud_->points.emplace_back(7.8, y, 0.0);
  }

  // // 大车库
  // double obstacle_interval = 0.1;
  // for(double x = 10.8; x <= 14.0; x += obstacle_interval){
  //   obstacle_cloud_->points.emplace_back(x, 9.5, 0.0);
  //   obstacle_cloud_->points.emplace_back(x, 7.5, 0.0);
  // }

  // for(double y = 7.6; y <= 9.4; y += obstacle_interval){
  //   obstacle_cloud_->points.emplace_back(14.0, y, 0.0);
  // }

  // 设置点云属性
  obstacle_cloud_->width = obstacle_cloud_->points.size();
  obstacle_cloud_->height = 1;
  obstacle_cloud_->is_dense = true;
}

void publishMap() {
  int width = 300;  // Larger map width
  int height = 300; // Larger map height
  nav_msgs::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = map_frame_;
  grid_msg.header.stamp = ros::Time::now();
  grid_msg.info.resolution = grid_resolution_;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = 0.0;
  grid_msg.info.origin.position.y = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.data.resize(width * height);

  // Initialize map with free space
  std::fill(grid_msg.data.begin(), grid_msg.data.end(), 0);
  grid_map_pub_.publish(grid_msg);
}

void publishObstaclePoints() {
  // 转换为 ROS PointCloud2 消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*obstacle_cloud_, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  // 发布点云
  obstacle_points_pub_.publish(cloud_msg);
}
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher_node");
  MapPublisher map_pub;
  ros::Rate rate(1); // Publish every 1 second
  while (ros::ok()) {
    map_pub.publishMap();
    map_pub.publishObstaclePoints(); // 发布点云
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}