#pragma once

#include "rocket_types/rocket_types.h"
#include "ekf_rocket_model.hpp"
#include "rocket_types/eigen_conversions.h"
#include <memory>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <unsupported/Eigen/EulerAngles>

using namespace rocket;

class PidNavigation {
public:
    using state = EKFRocketModel::state;

    struct Config {
        double initial_roll = 0, rail_zenith = 0, rail_azimuth = 0;
    };

private:
    Config config;

    EKFRocketModel rocket_model;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;
    Eigen::Vector3d gimbal_control;

    // Kalman matrix
    double Q, R, P;

    // Kalman state
    state X;

    // TODO write generic RK4 function that takes the dynamics function as input
    void RK4(double dT) {
        state k1, k2, k3, k4, X_inter;

        auto stateDynamics = [this](const EKFRocketModel::state &x, EKFRocketModel::state &xdot) -> void {
            rocket_model.stateDynamics(x, imu_acc, imu_gyro, gimbal_control, xdot);
        };

        stateDynamics(X, k1);
        X_inter = X + k1 * dT / 2;
        stateDynamics(X_inter, k2);
        X_inter = X + k2 * dT / 2;
        stateDynamics(X_inter, k3);
        X_inter = X + k3 * dT;
        stateDynamics(X_inter, k4);

        X = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    }

public:
    // Constructor using default config
    PidNavigation(RocketProps &rocket_props) : PidNavigation(Config{}, rocket_props) {};

    PidNavigation(Config config_, RocketProps &rocket_props) :
            config(config_),
            rocket_model(rocket_props) {
        imu_acc.setZero();
        gimbal_control.setZero();

        typedef Eigen::EulerSystem<Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_Z> Rail_system;
        typedef Eigen::EulerAngles<double, Rail_system> angle_type;

        angle_type init_angle(config.rail_azimuth, config.rail_zenith, config.initial_roll);

        // Init state X
        X << 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, rocket_model.props.initial_propellant_mass;
        X.segment(6, 4) = Eigen::Quaterniond(init_angle).coeffs();

        // Initialize kalman parameters
        P = 0;
        Q = 1;
        R = 5;
    }

    void barometerUpdate(double &baro_height) {
        double K = P / (P + R);

        X(2) = X(2) + K * (baro_height - X(2));
        P = (1.0 - K) * P;
    }

    void predict(double dT,
                 const Vector3 &imu_acc_,
                 const Vector3 &imu_gyro_,
                 const RocketGimbalControl &gimbal_control_) {
        imu_acc = toEigen(imu_acc_);
        imu_gyro = toEigen(imu_gyro_);
        gimbal_control = toEigen(gimbal_control_);

        RK4(dT);

        P += Q;
    }

    inline RocketState getState() const {
        return toRocketState(X);
    }
};