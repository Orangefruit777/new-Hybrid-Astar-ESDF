#include "controller/lqr_core.h"

LQRCore::LQRCore(double wheelbase, double ts, int max_iteration, double eps)
    : wheelbase_(wheelbase), ts_(ts), max_iteration_(max_iteration), eps_(eps),
      e_cg_prev_(0.0), theta_e_prev_(0.0) {
    matrix_q_(0, 0) = 1.0;  // e_cg
    matrix_q_(2, 2) = 1.0;  // theta_e
    matrix_r_(0, 0) = 1.0;
}

double LQRCore::computeLateralControl(double x, double y, double yaw, double v, 
                                     const std::vector<double>& ref_x,
                                     const std::vector<double>& ref_y, 
                                     const std::vector<double>& ref_yaw,
                                     const std::vector<double>& ref_k, size_t& ind_old, 
                                     double& e_cg, double& theta_e) {
    if (ref_x.empty()) return 0.0;

    // 找到最近点
    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = ind_old;
    for (size_t i = ind_old; i < ref_x.size(); ++i) {
        double dist = std::hypot(x - ref_x[i], y - ref_y[i]);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    ind_old = min_idx;

    // 计算横向误差
    Eigen::Vector2d vec_axle_rot_90(std::cos(yaw + M_PI / 2.0), std::sin(yaw + M_PI / 2.0));
    Eigen::Vector2d vec_path_2_cg(x - ref_x[min_idx], y - ref_y[min_idx]);
    e_cg = (vec_axle_rot_90.dot(vec_path_2_cg) > 0.0) ? min_dist : -min_dist;

    // 计算航向误差
    theta_e = pi2pi(yaw - ref_yaw[min_idx]);
    double k_ref = ref_k[min_idx];

    // LQR 控制
    Eigen::MatrixXd A(4, 4), B(4, 1);
    updateMatrix(A, B, v);
    Eigen::MatrixXd K = solveLQR(A, B);

    Eigen::Vector4d state;
    state << e_cg, (e_cg - e_cg_prev_) / ts_, theta_e, (theta_e - theta_e_prev_) / ts_;
    double steer_feedback = -(K * state)(0, 0);
    double steer_feedforward = computeFeedForward(k_ref);

    e_cg_prev_ = e_cg;
    theta_e_prev_ = theta_e;

    return steer_feedback + steer_feedforward;
}

double LQRCore::computeLongitudinalControl(double target_speed, double dist, bool gear_drive,
                                          bool waiting_for_stop, double v, 
                                          double dist_to_gear_point, bool should_change_gear) {
    if (waiting_for_stop) {
        // 这个参数有问题，待修改
        if (std::abs(v) > 0.2) {
            return -3.0;  // 强制刹车
        } else {
            return 0.0;   // 已停车，保持静止
        }
    }

    double direct = gear_drive ? 1.0 : -1.0;
    double a = 0.5 * (target_speed - direct * v);
    
    if (dist_to_gear_point < 2.0 && should_change_gear) {
        a = -2.0;  // 提前减速
    } else if (dist < 3.0) {
        if (v > 2.0) a = -3.0;
        else if (v < -2.0) a = -1.0;
    }
    
    return a;
}

void LQRCore::updateState(double& x, double& y, double& yaw, double& v, double delta, double a,
                         bool gear_drive, double max_steer_angle, double max_acceleration,
                         double max_speed) {
    delta = std::max(-max_steer_angle, std::min(max_steer_angle, delta));
    a = std::max(-max_acceleration, std::min(max_acceleration, a));
    
    x += v * std::cos(yaw) * ts_;
    y += v * std::sin(yaw) * ts_;
    yaw += v / wheelbase_ * std::tan(delta) * ts_;
    yaw = pi2pi(yaw);
    v += (gear_drive ? a : -a) * ts_;
    v = std::max(-max_speed, std::min(max_speed, v));
}

void LQRCore::computePathCurvature(const std::vector<double>& ref_x, const std::vector<double>& ref_y,
                                  std::vector<double>& ref_k, const std::vector<int>& divide) {
    ref_k.resize(ref_x.size(), 0.0);
    std::vector<double> ds(ref_x.size(), 0.0);

    for (size_t i = 1; i < ref_x.size() - 1; ++i) {
        double dxn = ref_x[i] - ref_x[i - 1];
        double dxp = ref_x[i + 1] - ref_x[i];
        double dyn = ref_y[i] - ref_y[i - 1];
        double dyp = ref_y[i + 1] - ref_y[i];

        double dn = std::hypot(dxn, dyn);
        double dp = std::hypot(dxp, dyp);

        double dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp);
        double dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp);

        double ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn);
        double ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn);

        double denominator = dx * dx + dy * dy;
        double curvature = denominator > 1e-6 ? (ddy * dx - ddx * dy) / denominator : 0.0;

        if (divide[i] < 0) {
            curvature = -curvature;
        }

        ref_k[i] = curvature;
        ds[i] = (dn + dp) / 2.0;
    }

    if (ref_x.size() > 1) {
        ref_k[0] = ref_k[1];
        ds[0] = ds[1];
        ref_k.back() = ref_k[ref_k.size() - 2];
        ds.back() = ds[ds.size() - 2];
    }
}

Eigen::MatrixXd LQRCore::solveLQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    Eigen::MatrixXd P = matrix_q_;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4, 1);
    int iter = 0;
    double diff = std::numeric_limits<double>::max();

    while (iter < max_iteration_ && diff > eps_) {
        Eigen::MatrixXd P_next = A.transpose() * P * A -
            (A.transpose() * P * B) * (matrix_r_ + B.transpose() * P * B).inverse() * (B.transpose() * P * A) + matrix_q_;
        diff = (P_next - P).cwiseAbs().maxCoeff();
        P = P_next;
        ++iter;
    }

    return (B.transpose() * P * B + matrix_r_).inverse() * (B.transpose() * P * A);
}

void LQRCore::updateMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double v) {
    A.setZero();
    A(0, 0) = 1.0;
    A(0, 1) = ts_;
    A(1, 2) = v;
    A(2, 2) = 1.0;
    A(2, 3) = ts_;

    B.setZero();
    B(3, 0) = v / wheelbase_;
}

double LQRCore::computeFeedForward(double k_ref) {
    return wheelbase_ * k_ref;
}

double LQRCore::pi2pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}