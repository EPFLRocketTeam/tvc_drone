#pragma once

#include "rocket_model.hpp"

class EKFRocketModel : public RocketModel {
public:
    using state = Eigen::Matrix<double, 14, 1>;

    EKFRocketModel(RocketProps &props) : RocketModel(props) {
    }

    void stateDynamics(const state &x,
                       const Eigen::Vector3d &imu_acc,
                       const Eigen::Vector3d &imu_gyro,
                       const Eigen::Vector3d &gimbal_control,
                       Eigen::Ref<state> xdot) {
        // -------------- Simulation variables -----------------------------
        double g0 = 9.81;  // Earth gravity in [m/s^2]Q

        // Orientation of the rocket with quaternion
        Eigen::Quaterniond attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();

        Eigen::Vector3d omega = rot_matrix * imu_gyro;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaterniond omega_quat(0.0, omega(0), omega(1), omega(2));

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is acceleration
        xdot.segment(3, 3) = rot_matrix * imu_acc - Eigen::Vector3d::UnitZ() * g0;

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) = 0.5 * (omega_quat * attitude).coeffs();

        // Angular speed assumed to be constant between two measure
        xdot.segment(10, 3) << 0, 0, 0;

        // Mass variation is proportional to total thrust
        xdot(13) = -gimbal_control(2) / (props.Isp * g0);
    }
};
