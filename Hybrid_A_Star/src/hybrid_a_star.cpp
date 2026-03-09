#include "hybrid_a_star/hybrid_a_star.h"

#include <iostream>

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                        int segment_length_discrete_num, double wheel_base, double steering_penalty,
                        double reversing_penalty, double steering_change_penalty, double shot_distance,
                        int grid_size_phi) {
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0;
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    // CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
    //     << "The segment length must be divisible by the step size. segment_length: "
    //     << segment_length_ << " | step_size: " << move_step_size_;

    turning_radius = 2.5;
    dubins_curve_ptr_ = std::make_shared<DubinsCurveNew>(turning_radius, move_step_size_);
    rs_path_ptr_ = std::make_shared<RSPath>(turning_radius);

    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                    double state_grid_resolution, double map_grid_resolution) {
    // 车辆原始尺寸
    // SetVehicleShape(2.0, 1.0, 0.5);
    
    // 稍微膨胀一点
    SetVehicleShape(2.2, 1.2, 0.5);

    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);
    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_.clear();
    map_data_.resize(MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_, 0);

    state_node_map_.clear();
    state_node_map_.resize(STATE_GRID_SIZE_X_, std::vector<std::vector<std::shared_ptr<StateNode>>>(
        STATE_GRID_SIZE_Y_, std::vector<std::shared_ptr<StateNode>>(STATE_GRID_SIZE_PHI_, nullptr)));
}

inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0)) ||
                BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_, (x0 + i) * MAP_GRID_RESOLUTION_))) {
                return false;
            }
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk)) ||
                BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_, yk * MAP_GRID_RESOLUTION_))) {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {
    Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
        std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0) =
            R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
        transformed_vehicle_shape.block<2, 1>(0, 0));
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
        transformed_vehicle_shape.block<2, 1>(2, 0));
    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
        transformed_vehicle_shape.block<2, 1>(4, 0));
    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
        transformed_vehicle_shape.block<2, 1>(6, 0));

    double y1, y0, x1, x0;
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();
    num_check_collision++;
    return true;
}

bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_ &&
            grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_ &&
            map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1);
}

bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_ &&
            grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_ &&
            map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1);
}

void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {
    if (x >= static_cast<unsigned int>(MAP_GRID_SIZE_X_) ||
        y >= static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
        return;
    }

    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetObstacle(const double pt_x, const double pt_y) {
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int>(width / step_size);
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0) -
                                    vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length) =
            vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i) =
            vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0) -
                                    vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i) =
            vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width) =
            vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    pt.x() = ((double)grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double)grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;
    return pt;
}

Vec3i HybridAStar::State2Index(const Vec3d &state) const {
    Vec3i index;
    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
    return index;
}

Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;
    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

void HybridAStar::GetNeighborNodes(const std::shared_ptr<StateNode> &curr_node_ptr,
                                std::vector<std::shared_ptr<StateNode>> &neighbor_nodes) {
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();

        const double phi = i * steering_radian_step_size_;

        // Forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            auto neighbor_forward_node_ptr = std::make_shared<StateNode>(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        // Backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = std::make_shared<StateNode>(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                            double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

bool HybridAStar::BeyondBoundary(const Vec2d &pt) const {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ ||
        pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double HybridAStar::ComputeH(const std::shared_ptr<StateNode> &current_node_ptr,
                            const std::shared_ptr<StateNode> &terminal_node_ptr) {
    double h;
    // h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // DubinsPoint start, end;
    // start.pos = current_node_ptr->state_.head(2);
    // start.yaw = current_node_ptr->state_.z();
    // end.pos = terminal_node_ptr->state_.head(2);
    // end.yaw = terminal_node_ptr->state_.z();

    // 启发函数，选择dubins长度或者rs长度
    // auto dubins_path = dubins_curve_ptr_->computeDubinsPath(start, end, turning_radius);

    if (h < 1.0 * shot_distance_) {
        // h = dubins_path.length;
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }

    return h;
}

double HybridAStar::ComputeG(const std::shared_ptr<StateNode> &current_node_ptr,
                            const std::shared_ptr<StateNode> &neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }
    return g;
}

bool HybridAStar::Search(const Vec3d &start_state, const Vec3d &goal_state, int use_dubins_or_rs) {
    Timer search_used_time;

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = std::make_shared<StateNode>(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = std::make_shared<StateNode>(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<std::shared_ptr<StateNode>> neighbor_nodes_ptr;
    std::shared_ptr<StateNode> current_node_ptr;
    std::shared_ptr<StateNode> neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
            double AEpath_length = 0.0;
            bool AESuccess;
            if(use_dubins_or_rs == 0){
                AESuccess = AnalyticExpansionsDubins(current_node_ptr, goal_node_ptr, AEpath_length);
            }
            else if(use_dubins_or_rs == 1){
                AESuccess = AnalyticExpansionsRS(current_node_ptr, goal_node_ptr, AEpath_length);
            }
            if (AESuccess) {
                terminal_node_ptr_ = goal_node_ptr;

                std::shared_ptr<StateNode> grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + AEpath_length;

                std::cout << "current_node_ptr->state_: " << current_node_ptr->state_.transpose() << std::endl;

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                        << check_collision_use_time / num_check_collision
                        << std::endl;
                std::cout << "\033[1;32mTime in Hybrid A star is: " << search_used_time.End() 
                          << " ms, path length: " << path_length_ << "\033[0m" << std::endl;

                std::cout << "count: " << count << std::endl;

                double dis = (start_state.head(2) - goal_state.head(2)).norm();
                if(path_length_ > 3 * dis){
                    std::cout << "\033[1;31mPath is too long, please try again!\033[0m" << std::endl;
                    return false;
                }
                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true; 
            }
        }

        

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time = neighbor_time + timer_get_neighbor.End();

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr->grid_index_;
            auto &existing_node = state_node_map_[index.x()][index.y()][index.z()];
            if (existing_node == nullptr) {
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                existing_node = neighbor_node_ptr;
            } else if (existing_node->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;
                if (existing_node->g_cost_ > g_cost_temp) {
                    // Find and erase the existing node from openset_
                    for (auto it = openset_.begin(); it != openset_.end(); ++it) {
                        if (it->second == existing_node && it->first == existing_node->f_cost_) {
                            openset_.erase(it);
                            break;
                        }
                    }
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                    openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                    existing_node = neighbor_node_ptr;
                }
            }
        }

        count++;
        
        // count raw 50000
        if (count > 20000) {
            std::cout << "\033[1;31mExceeded the number of iterations, the search failed!\033[0m" << std::endl;
            return false;
        }
    }

    return false;
}

VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                auto node = state_node_map_[i][j][k];
                if (node == nullptr || node->parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = node->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = node->intermediate_states_[l].head(2);
                    point_pair.tail(2) = node->intermediate_states_[l + 1].head(2);
                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = node->intermediate_states_[0].head(2);
                point_pair.tail(2) = node->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {
    map_data_.clear();
    state_node_map_.clear();
    terminal_node_ptr_ = nullptr;
    openset_.clear();
}

VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;

    std::vector<std::shared_ptr<StateNode>> temp_nodes;
    std::shared_ptr<StateNode> state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node : temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }
    return path;
}

void HybridAStar::Reset() {
    for (auto &row : state_node_map_) {
        for (auto &col : row) {
            for (auto &node : col) {
                node = nullptr;
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
    openset_.clear();
}


bool HybridAStar::AnalyticExpansionsDubins(const std::shared_ptr<StateNode> &current_node_ptr,
                                           const std::shared_ptr<StateNode> &goal_node_ptr, double &length) {
    DubinsPoint start, end;
    start.pos = current_node_ptr->state_.head(2);
    start.yaw = current_node_ptr->state_.z();
    end.pos = goal_node_ptr->state_.head(2);
    end.yaw = goal_node_ptr->state_.z();
    double current_h = (start.pos - end.pos).norm();
    auto dubins_path = dubins_curve_ptr_->computeDubinsPath(start, end, turning_radius);
    length = dubins_path.length;

    VectorVec3d dubins_path_poses;
    dubins_path_poses.reserve(dubins_path.points.size());

    dubins_path_poses.emplace_back(Vec3d(
        dubins_path.points[0].x(),
        dubins_path.points[0].y(),
        start.yaw
    ));
    for (size_t i = 1; i < dubins_path.points.size() - 1; ++i) {
        const auto &prev_point = dubins_path.points[i - 1];
        const auto &next_point = dubins_path.points[i + 1];
        double dx = next_point.x() - prev_point.x();
        double dy = next_point.y() - prev_point.y();
        double yaw = std::atan2(dy, dx);
        dubins_path_poses.emplace_back(Vec3d(
            dubins_path.points[i].x(),
            dubins_path.points[i].y(),
            yaw
        ));
    }

    dubins_path_poses.emplace_back(Vec3d(
        dubins_path.points.back().x(),
        dubins_path.points.back().y(),
        end.yaw
    ));

    for (const auto &pose : dubins_path_poses) {
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z()) ||
            length > 1.3 * current_h) {
            return false;
        }
    }

    goal_node_ptr->intermediate_states_ = dubins_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

bool HybridAStar::AnalyticExpansionsRS(const std::shared_ptr<StateNode> &current_node_ptr,
                                       const std::shared_ptr<StateNode> &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                        goal_node_ptr->state_,
                        move_step_size_, length);

    for (const auto &pose: rs_path_poses)
    if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
    return false;
    };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

// Yang 添加，将Hybrid A*路径转化为FlatTrajData类型轨迹
void HybridAStar::GetKinoTraj(const VectorVec3d& path, double max_forward_vel, double max_backward_vel, 
                             double startvel, double endvel, double non_siguav, 
                             std::vector<path_searching::FlatTrajData>& flat_trajs, double& totalTrajTime) {
    flat_trajs.clear();
    SampleTraj = path; // 存储 SampleTraj
    shotindex.clear();
    shot_SList.clear();
    shot_lengthList.clear();
    shot_timeList.clear();
    
    // 存储速度参数
    max_forward_vel_ = max_forward_vel;
    max_backward_vel_ = max_backward_vel;
    non_siguav_ = non_siguav;
    startvel_ = startvel;
    endvel_ = endvel;

    // Step 1: Path Segmentation
    shotindex.push_back(0);
    double tmpl = 0;
    int lastS = (SampleTraj[1] - SampleTraj[0]).head(2).dot(Eigen::Vector2d(cos(SampleTraj[0][2]), sin(SampleTraj[0][2]))) >= 0 ? 1 : -1;
    for (int i = 0; i < SampleTraj.size() - 1; i++) {
        Eigen::Vector3d state1 = SampleTraj[i];
        Eigen::Vector3d state2 = SampleTraj[i + 1];
        int curS = (state2 - state1).head(2).dot(Eigen::Vector2d(cos(state1[2]), sin(state1[2]))) >= 0 ? 1 : -1;
        if (curS * lastS >= 0) {
            tmpl += (state2 - state1).head(2).norm();
        } else {
            shotindex.push_back(i);
            shot_SList.push_back(lastS);
            shot_lengthList.push_back(tmpl);
            shot_timeList.push_back(tmpl / (lastS > 0 ? max_forward_vel_ : max_backward_vel_));
            tmpl = (state2 - state1).head(2).norm();
        }
        lastS = curS;
    }
    shot_SList.push_back(lastS);
    shot_lengthList.push_back(tmpl);
    shot_timeList.push_back(tmpl / (lastS > 0 ? max_forward_vel_ : max_backward_vel_));
    shotindex.push_back(SampleTraj.size() - 1);

    // Adjust time for start and end segments
    if (shot_timeList.size() >= 2) {
        shot_timeList[0] = shot_lengthList[0] / (shot_SList[0] > 0 ? max_forward_vel_ : max_backward_vel_);
        shot_timeList.back() = shot_lengthList.back() / (shot_SList.back() > 0 ? max_forward_vel_ : max_backward_vel_);
    }

    // Step 2: Trajectory Interpolation and KinoTrajData Construction
    for (int i = 0; i < shot_lengthList.size(); i++) {
        double initv = non_siguav_, finv = non_siguav_;
        Eigen::Vector2d Initctrlinput(0, 0), Finctrlinput(0, 0);
        if (i == 0) { initv = startvel_; }
        if (i == shot_lengthList.size() - 1) { finv = endvel_; }

        double locallength = shot_lengthList[i];
        int sig = shot_SList[i];
        VectorVec3d localTraj(SampleTraj.begin() + shotindex[i], SampleTraj.begin() + shotindex[i + 1] + 1);
        VectorVec3d traj_pts;
        std::vector<double> thetas;

        double samplet;
        double tmparc = 0;
        int index = 0;
        double sampletime = 0.1;
        if (shot_timeList[i] <= sampletime) {
            sampletime = shot_timeList[i] / 2.0;
        }

        for (samplet = sampletime; samplet < shot_timeList[i]; samplet += sampletime) {
            double arc = locallength * (samplet / shot_timeList[i]); // Uniform velocity
            for (int k = index; k < localTraj.size() - 1; k++) {
                tmparc += (localTraj[k + 1] - localTraj[k]).head(2).norm();
                if (tmparc >= arc) {
                    index = k;
                    double l1 = tmparc - arc;
                    double l = (localTraj[k + 1] - localTraj[k]).head(2).norm();
                    double l2 = l - l1;
                    double px = (l1 / l * localTraj[k] + l2 / l * localTraj[k + 1])[0];
                    double py = (l1 / l * localTraj[k] + l2 / l * localTraj[k + 1])[1];
                    double yaw = (l1 / l * localTraj[k] + l2 / l * localTraj[k + 1])[2];
                    if (fabs(localTraj[k + 1][2] - localTraj[k][2]) >= M_PI) {
                        double normalize_yaw;
                        if (localTraj[k + 1][2] <= 0) {
                            normalize_yaw = l1 / l * localTraj[k][2] + l2 / l * (localTraj[k + 1][2] + 2 * M_PI);
                        } else if (localTraj[k][2] <= 0) {
                            normalize_yaw = l1 / l * (localTraj[k][2] + 2 * M_PI) + l2 / l * localTraj[k + 1][2];
                        }
                        yaw = normalize_yaw;
                    }
                    traj_pts.push_back(Eigen::Vector3d(px, py, sampletime));
                    thetas.push_back(yaw);
                    tmparc -= (localTraj[k + 1] - localTraj[k]).head(2).norm();
                    break;
                }
            }
        }
        traj_pts.push_back(Eigen::Vector3d(localTraj.back()[0], localTraj.back()[1], shot_timeList[i] - (samplet - sampletime)));
        thetas.push_back(localTraj.back()[2]);

        // Construct FlatTrajData
        path_searching::FlatTrajData flat_traj;
        Eigen::MatrixXd startS(2, 3), endS(2, 3);
        double angle = localTraj.front()[2];
        Eigen::Matrix2d init_R;
        init_R << cos(angle), -sin(angle), sin(angle), cos(angle);
        double vel = sig * (abs(initv) <= non_siguav_ ? non_siguav_ : initv);
        startS << localTraj.front().head(2), init_R * Eigen::Vector2d(vel, 0.0), 
                 init_R * Eigen::Vector2d(Initctrlinput(1), tan(Initctrlinput(0)) / wheel_base_ * vel * vel);
        angle = localTraj.back()[2];
        init_R << cos(angle), -sin(angle), sin(angle), cos(angle);
        vel = sig * (abs(finv) <= non_siguav_ ? non_siguav_ : finv);
        endS << localTraj.back().head(2), init_R * Eigen::Vector2d(vel, 0.0), 
               init_R * Eigen::Vector2d(Finctrlinput(1), tan(Finctrlinput(0)) / wheel_base_ * vel * vel);

        flat_traj.traj_pts = traj_pts;
        flat_traj.thetas = thetas;
        flat_traj.start_state = startS;
        flat_traj.final_state = endS;
        flat_traj.singul = sig;
        flat_traj.duration = shot_timeList[i];
        flat_trajs.push_back(flat_traj);
    }

    // Compute Total Trajectory Time
    totalTrajTime = 0.0;
    for (const auto dt : shot_timeList) {
        totalTrajTime += dt;
    }
    this->totalTrajTime = totalTrajTime; // 存储 totalTrajTime
}

// 添加 evaluatePos 函数
Eigen::Vector3d HybridAStar::evaluatePos(double t) {
    t = std::min<double>(std::max<double>(0, t), totalTrajTime);
    double startvel = startvel_;
    double endvel = endvel_;
    int index = -1;
    double tmpT = 0;
    double CutTime;

    // Locate the local trajectory segment
    for (int i = 0; i < shot_timeList.size(); i++) {
        tmpT += shot_timeList[i];
        if (tmpT >= t) {
            index = i;
            CutTime = t - tmpT + shot_timeList[i];
            break;
        }
    }

    if (index == -1) {
        return SampleTraj.back(); // Return the last point if t exceeds total time
    }

    double initv = non_siguav_, finv = non_siguav_;
    if (index == 0) { initv = startvel; }
    if (index == shot_lengthList.size() - 1) { finv = endvel; }

    double localtime = shot_timeList[index];
    double locallength = shot_lengthList[index];
    int front = shotindex[index];
    int back = shotindex[index + 1];
    VectorVec3d localTraj(SampleTraj.begin() + front, SampleTraj.begin() + back + 1);

    // Calculate arc length using uniform velocity
    double arclength = locallength * (CutTime / localtime);

    // Find the nearest point
    double tmparc = 0;
    for (int i = 0; i < localTraj.size() - 1; i++) {
        tmparc += (localTraj[i + 1] - localTraj[i]).head(2).norm();
        if (tmparc >= arclength) {
            double l1 = tmparc - arclength;
            double l = (localTraj[i + 1] - localTraj[i]).head(2).norm();
            double l2 = l - l1;
            Eigen::Vector3d state = l1 / l * localTraj[i] + l2 / l * localTraj[i + 1];
            if (fabs(localTraj[i + 1][2] - localTraj[i][2]) >= M_PI) {
                double normalize_yaw;
                if (localTraj[i + 1][2] <= 0) {
                    normalize_yaw = l1 / l * localTraj[i][2] + l2 / l * (localTraj[i + 1][2] + 2 * M_PI);
                } else if (localTraj[i][2] <= 0) {
                    normalize_yaw = l1 / l * (localTraj[i][2] + 2 * M_PI) + l2 / l * localTraj[i + 1][2];
                }
                state[2] = normalize_yaw;
            }
            return state;
        }
    }
    return localTraj.back();
}