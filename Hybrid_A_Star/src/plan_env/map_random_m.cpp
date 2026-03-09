#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <ros/package.h>
#include "plan_env/sdf_map_2d.h"

class RandomMapPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_map_pub_;
    ros::Publisher obstacle_points_pub_;

    // Parameters
    double grid_resolution_ = 0.2;
    int map_width_ = 250;
    int map_height_ = 250;
    std::string map_frame_ = "world";
    int random_seed_;

    // Random number generators
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_x_;
    std::uniform_real_distribution<> dis_y_;
    std::uniform_real_distribution<> dis_size_;
    std::uniform_int_distribution<> dis_count_;
    std::uniform_int_distribution<> dis_type_;
    std::uniform_int_distribution<> dis_sides_;
    std::uniform_real_distribution<> dis01_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;

    // Generation policy parameters (can be overridden by rosparam)
    double border_m_ = 0.5;
    double clearance_m_ = 2.0;
    int max_place_attempts_ = 100;

    // ESDF map
    SDFMap2D sdf_map_;
    std::string esdf_save_path_ = "/tmp/esdf_map.xml";

public:
    RandomMapPublisher() : dis_x_(2.0, 48.0),
                           dis_y_(2.0, 48.0),
                           dis_size_(1.5, 2.5),
                           dis_count_(38, 40),
                           dis_type_(0, 2),
                           dis_sides_(4, 7),
                           dis01_(0.0, 1.0){

        // Read parameters
        nh_.param("grid_resolution", grid_resolution_, 0.2);
        nh_.param("map_width", map_width_, 250);
        nh_.param("map_height", map_height_, 250);
        nh_.param("map_frame", map_frame_, std::string("world"));
        nh_.param("border_m", border_m_, 0.5);
        nh_.param("clearance_m", clearance_m_, 0.5);
        nh_.param("max_place_attempts", max_place_attempts_, 100);
        nh_.param("random_seed", random_seed_, 12345); // Use a fixed seed

        // Set default ESDF save path to package scripts directory
        std::string pkg_path = ros::package::getPath("hybrid_a_star");
        std::string default_esdf_path = pkg_path + "/scripts/esdf_map.xml";
        nh_.param("esdf_save_path", esdf_save_path_, default_esdf_path);

        gen_.seed(random_seed_); // Seed the generator for deterministic results

        grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        obstacle_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1, true);

        obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        generateRandomMap();
        publishAll();
        computeAndSaveESDF();
    }

    // 添加矩形障碍物
    void addRectangle(double x_min, double x_max, double y_min, double y_max, double step = 0.2) {
        for (double x = x_min; x <= x_max; x += step) {
            for (double y = y_min; y <= y_max; y += step) {
                obstacle_cloud_->points.emplace_back(x, y, 0.0);
            }
        }
    }

    // 添加圆形障碍物
    void addCircle(double cx, double cy, double radius, double step = 0.2) {
        for (double x = cx - radius; x <= cx + radius; x += step) {
            for (double y = cy - radius; y <= cy + radius; y += step) {
                if (std::hypot(x - cx, y - cy) <= radius) {
                    obstacle_cloud_->points.emplace_back(x, y, 0.0);
                }
            }
        }
    }

    // 添加多边形（凸多边形）
    void addPolygon(const std::vector<std::pair<double, double>>& vertices, double step = 0.2) {
        if (vertices.size() < 3) return;

        // 简单填充：扫描线法（简化版）
        double min_x = vertices[0].first, max_x = vertices[0].first;
        double min_y = vertices[0].second, max_y = vertices[0].second;
        for (auto& p : vertices) {
            min_x = std::min(min_x, p.first);
            max_x = std::max(max_x, p.first);
            min_y = std::min(min_y, p.second);
            max_y = std::max(max_y, p.second);
        }

        for (double x = min_x; x <= max_x; x += step) {
            for (double y = min_y; y <= max_y; y += step) {
                if (pointInPolygon(x, y, vertices)) {
                    obstacle_cloud_->points.emplace_back(x, y, 0.0);
                }
            }
        }
    }

    bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& poly) {
        int n = poly.size();
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if ((poly[i].second > y) != (poly[j].second > y) &&
                (x < poly[i].first + (poly[j].first - poly[i].first) * (y - poly[i].second) / (poly[j].second - poly[i].second + 1e-8))) {
                inside = !inside;
            }
        }
        return inside;
    }

    
    void generateRandomMap() {
        obstacle_cloud_->clear();

        // 地图实际尺寸（米）
        const double Wm = map_width_ * grid_resolution_;
        const double Hm = map_height_ * grid_resolution_;

        struct Ob {
            double x, y;     // 中心
            double r;        // 外接圆半径（用于非重叠检测）
            int type;        // 0: rect, 1: circle, 2: poly
            double size;     // 形状尺度参数（与现有 add* 保持一致）
        };
        std::vector<Ob> placed;

        int num_obstacles = dis_count_(gen_);
        ROS_INFO("Generating up to %d random obstacles (no-overlap, in-bounds)...", num_obstacles);

        // 尝试放置障碍物
        for (int i = 0; i < num_obstacles; ++i) {
            // 先随机出类型和尺寸
            int type = dis_type_(gen_);
            double size = dis_size_(gen_);

            // 根据类型估算外接圆半径（与 add* 的参数一致）
            auto bound_radius = [&](int t, double s) {
                if (t == 0) {          // 矩形：半长 s，半宽 0.6s
                    return std::hypot(s, 0.6 * s);  // ≈ 1.166s
                } else if (t == 1) {   // 圆形
                    return s;
                } else {               // 多边形：半径在 [0.8s, 1.2s]，用上界 1.2s 保护
                    return 1.2 * s;
                }
            };
            double r_bound = bound_radius(type, size);

            // 如果太大，按地图可容纳的最大半径压缩一次
            double max_r_allow = std::min(Wm, Hm) * 0.5 - border_m_;
            if (max_r_allow <= 0.0) {
                ROS_WARN("Map is too small to place any obstacle with current border setting.");
                break;
            }
            if (r_bound > max_r_allow) {
                r_bound = max_r_allow;
                // 同步缩小 size，保持和 add* 的几何一致性
                if (type == 0) {
                    size = r_bound / std::hypot(1.0, 0.6);
                } else if (type == 1) {
                    size = r_bound;
                } else {
                    size = r_bound / 1.2;
                }
            }

            // 动态可采样范围，确保形状整体不越界
            const double minx = r_bound + border_m_;
            const double maxx = Wm - (r_bound + border_m_);
            const double miny = r_bound + border_m_;
            const double maxy = Hm - (r_bound + border_m_);

            if (maxx <= minx || maxy <= miny) {
                ROS_WARN("Obstacle %d cannot fit within map bounds after shrink; skipping.", i);
                continue;
            }

            bool placed_ok = false;
            for (int attempt = 0; attempt < max_place_attempts_; ++attempt) {
                double cx = minx + dis01_(gen_) * (maxx - minx);
                double cy = miny + dis01_(gen_) * (maxy - miny);

                // 非重叠检测
                bool ok = true;
                for (const auto& q : placed) {
                    double dx = cx - q.x;
                    double dy = cy - q.y;
                    double min_d = r_bound + q.r + clearance_m_;
                    if (dx * dx + dy * dy < min_d * min_d) {
                        ok = false;
                        break;
                    }
                }
                if (!ok) continue;

                // 通过非重叠与越界检查，正式放置
                placed.push_back({cx, cy, r_bound, type, size});

                switch (type) {
                    case 0: // 矩形
                        addRectangle(cx - size, cx + size, cy - size * 0.6, cy + size * 0.6);
                        break;
                    case 1: // 圆
                        addCircle(cx, cy, size);
                        break;
                    case 2: { // 多边形
                        std::vector<std::pair<double, double>> poly;
                        int sides = dis_sides_(gen_);
                        double angle_step = 2 * M_PI / sides;
                        double r_var = size * 0.8;   // 基半径
                        for (int j = 0; j < sides; ++j) {
                            double r = r_var + dis01_(gen_) * size * 0.4;                // 抖动到 [0.8s,1.2s]
                            double ang = j * angle_step + dis01_(gen_) * angle_step * 0.5;
                            poly.emplace_back(cx + r * std::cos(ang), cy + r * std::sin(ang));
                        }
                        addPolygon(poly);
                        break;
                    }
                    default:
                        break;
                }

                placed_ok = true;
                break;
            }

            if (!placed_ok) {
                ROS_WARN("Failed to place obstacle %d after %d attempts; skipping.", i, max_place_attempts_);
            }
        }

        // 添加边界墙（可选）
        double wall_step = 0.2;
        addRectangle(0.0, 50.0, 0.0, 0.5, wall_step);
        addRectangle(0.0, 50.0, 49.5, 50.0, wall_step);
        addRectangle(0.0, 0.5, 0.5, 49.5, wall_step);
        addRectangle(49.5, 50.0, 0.5, 49.5, wall_step);

        obstacle_cloud_->width = obstacle_cloud_->points.size();
        obstacle_cloud_->height = 1;
        obstacle_cloud_->is_dense = true;
    }

    void publishGridMap() {
        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.frame_id = map_frame_;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.info.resolution = grid_resolution_;
        grid_msg.info.width = map_width_;
        grid_msg.info.height = map_height_;
        grid_msg.info.origin.position.x = 0.0;
        grid_msg.info.origin.position.y = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.assign(map_width_ * map_height_, 0);  // 0 = free

        // 填充障碍物
        for (const auto& pt : obstacle_cloud_->points) {
            int ix = static_cast<int>(pt.x / grid_resolution_);
            int iy = static_cast<int>(pt.y / grid_resolution_);
            if (ix >= 0 && ix < map_width_ && iy >= 0 && iy < map_height_) {
                grid_msg.data[iy * map_width_ + ix] = 100;  // 100 = occupied
            }
        }

        grid_map_pub_.publish(grid_msg);
    }

    void publishObstaclePoints() {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*obstacle_cloud_, cloud_msg);
        cloud_msg.header.frame_id = map_frame_;
        cloud_msg.header.stamp = ros::Time::now();
        obstacle_points_pub_.publish(cloud_msg);
    }

    void publishAll() {
        publishGridMap();
        publishObstaclePoints();
        ROS_INFO("Random map published! (%lu obstacle points)", obstacle_cloud_->size());
    }

    // 重新生成地图（可通过 service 调用）
    void regenerate() {
        generateRandomMap();
        publishAll();
    }

    // 计算并保存ESDF
    void computeAndSaveESDF() {
        ROS_INFO("Computing ESDF...");

        // Convert point cloud to obstacle vector
        std::vector<Eigen::Vector2d> obstacles;
        obstacles.reserve(obstacle_cloud_->points.size());

        for (const auto& pt : obstacle_cloud_->points) {
            obstacles.emplace_back(pt.x, pt.y);
        }

        // Initialize SDF map
        Eigen::Vector2d origin(0.0, 0.0);
        Eigen::Vector2i size(map_width_, map_height_);
        sdf_map_.initMap(origin, size, grid_resolution_, obstacles);

        ROS_INFO("ESDF computed successfully! Total obstacle points: %lu", obstacles.size());

        // Save to XML
        saveESDF();
    }

    // 保存ESDF到XML文件
    void saveESDF() {
        ROS_INFO("Saving ESDF to: %s", esdf_save_path_.c_str());

        std::ofstream file(esdf_save_path_);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", esdf_save_path_.c_str());
            return;
        }

        // Write XML header
        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<ESDF_Map>\n";

        // Map metadata
        file << "  <Metadata>\n";
        file << "    <width>" << map_width_ << "</width>\n";
        file << "    <height>" << map_height_ << "</height>\n";
        file << "    <resolution>" << grid_resolution_ << "</resolution>\n";
        file << "    <origin_x>0.0</origin_x>\n";
        file << "    <origin_y>0.0</origin_y>\n";
        file << "    <frame>" << map_frame_ << "</frame>\n";
        file << "  </Metadata>\n";

        // ESDF data
        file << "  <ESDF_Data>\n";
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                Eigen::Vector2d pos;
                pos(0) = (x + 0.5) * grid_resolution_;
                pos(1) = (y + 0.5) * grid_resolution_;

                double dist = sdf_map_.getDistance(pos);
                file << dist;

                if (x < map_width_ - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }
        file << "  </ESDF_Data>\n";

        file << "</ESDF_Map>\n";
        file.close();

        ROS_INFO("ESDF saved successfully to: %s", esdf_save_path_.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_map_publisher");
    RandomMapPublisher publisher;

    // The map is generated and published once in the constructor.
    // No continuous regeneration.
    ros::spin(); 
    return 0;
}