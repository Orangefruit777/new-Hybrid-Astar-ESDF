#ifndef LQR_CORE_H
#define LQR_CORE_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>

class LQRCore {
public:
    LQRCore(double wheelbase, double ts, int max_iteration, double eps);

    // 计算横向控制
    double computeLateralControl(double x, double y, double yaw, double v, 
                                const std::vector<double>& ref_x,
                                const std::vector<double>& ref_y, 
                                const std::vector<double>& ref_yaw,
                                const std::vector<double>& ref_k, size_t& ind_old, 
                                double& e_cg, double& theta_e);

    // 计算纵向控制
    double computeLongitudinalControl(double target_speed, double dist, bool gear_drive,
                                     bool waiting_for_stop, double v, 
                                     double dist_to_gear_point, bool should_change_gear);

    // 更新车辆状态
    void updateState(double& x, double& y, double& yaw, double& v, double delta, double a, 
                     bool gear_drive, double max_steer_angle, double max_acceleration, 
                     double max_speed);

    // 计算路径曲率
    void computePathCurvature(const std::vector<double>& ref_x, const std::vector<double>& ref_y,
                             std::vector<double>& ref_k, const std::vector<int>& divide);

private:
    // 车辆参数
    const double wheelbase_;  // 轴距 [m]
    const double ts_;         // 时间步长 [s]
    const int max_iteration_; // LQR 最大迭代次数
    const double eps_;        // LQR 收敛容差

    // 控制器参数
    Eigen::MatrixXd matrix_q_ = Eigen::MatrixXd::Identity(4, 4);  // 状态权重
    Eigen::MatrixXd matrix_r_ = Eigen::MatrixXd::Identity(1, 1);  // 输入权重
    double e_cg_prev_ = 0.0;   // 上一次横向误差
    double theta_e_prev_ = 0.0; // 上一次航向误差

    // 方法
    Eigen::MatrixXd solveLQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
    void updateMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double v);
    double computeFeedForward(double k_ref);
    double pi2pi(double angle);
};

#endif