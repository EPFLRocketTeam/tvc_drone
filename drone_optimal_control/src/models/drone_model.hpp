/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

template<typename scalar>
struct DroneProps {
    double dry_mass;
    double dry_mass_inv;

    double total_CM; // Current Cm of rocket, in real time

    std::vector<double> dry_Inertia{0, 0, 0};

    Eigen::Vector3d inertia;
    Eigen::Vector3d inertia_inv;

    // parameters
    double min_propeller_speed, max_propeller_speed;
    double max_propeller_delta;
    double max_servo1_angle, max_servo2_angle;
    double max_servo_rate;

    // estimated parameters
    double thrust_scaling;
    double torque_scaling;
    Eigen::Vector3d disturbance_force;
    Eigen::Vector3d disturbance_torque;

    double servo1_offset;
    double servo2_offset;

    DroneProps() {
        thrust_scaling = 1;
        torque_scaling = 1;
        disturbance_force.setZero();
        disturbance_torque.setZero();
    };
};


class Drone {
public:
    static const int NX = 13;
    static const int NU = 4;
    static const int NP = 8;

    template<typename T>
    using state_t = Eigen::Matrix<T, NX, 1>;
    using state = state_t<double>;
    template<typename T>
    using control_t = Eigen::Matrix<T, NU, 1>;
    using control = control_t<double>;
    template<typename T>
    using parameters_t = Eigen::Matrix<T, NP, 1>;
    using parameters = parameters_t<double>;

    const double g = 9.81;
    Eigen::Vector3d gravity_vector;

    DroneProps<double> props;

    Drone(DroneProps<double> &props_) : props(props_) {
        gravity_vector << 0, 0, -g;
    }

    template<typename T, typename state>
    inline void generic_rocket_dynamics(const Eigen::Matrix<T, 13, 1> x,
                                        const Eigen::Matrix<T, 3, 1> thrust_vector,
                                        const Eigen::Matrix<T, 3, 1> body_torque,
                                        const Eigen::Matrix<T, 3, 1> disturbance_force,
                                        const Eigen::Matrix<T, 3, 1> disturbance_torque,
                                        state &xdot) const noexcept {
        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));

        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;
        total_force =
                rot_matrix * thrust_vector + gravity_vector.template cast<T>() * (T) props.dry_mass + disturbance_force;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T) 0.0, x(10), x(11), x(12));

        // thrust vector torque in body frame (M = r x F)
        Eigen::Matrix<T, 3, 1> rocket_torque;
        rocket_torque << thrust_vector(1) * props.total_CM,
                -thrust_vector(0) * props.total_CM,
                (T) 0;

        // -------------- Differential equations ---------------------

        // Position derivative is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed derivative is Force/mass
        // using mass inverse to avoid computing slow division (maybe not needed?)
        xdot.segment(3, 3) = total_force * (T) props.dry_mass_inv;

        // Quaternion derivative is 0.5*q◦w (if w in body frame)
        xdot.segment(6, 4) = (T) 0.5 * (attitude * omega_quat).coeffs();

        Eigen::Matrix<T, 3, 1> omega = x.segment(10, 3);

        // Total torque in body frame
        Eigen::Matrix<T, 3, 1> total_torque;
        total_torque = rocket_torque + body_torque + rot_matrix.transpose() * disturbance_torque;

        // Angular speed derivative is given by Euler's rotation equations (if in body frame)
        xdot.segment(10, 3) = (total_torque - omega.cross(props.inertia.template cast<T>().cwiseProduct(omega)))
                .cwiseProduct(props.inertia_inv.template cast<T>());
    }

    template<typename T, typename state>
    inline void state_dynamics(const state_t<T> &x,
                               const control_t<T> &u,
                               const parameters_t<T> &params,
                               state &xdot) const {
        Eigen::Matrix<T, 4, 1> input;
        input << u(0), u(1), u(2), u(3);

        T thrust_scaling = params(0);
        T torque_scaling = params(1);
        Eigen::Matrix<T, 3, 1> dist_force = params.segment(2, 3);
        Eigen::Matrix<T, 3, 1> dist_torque = params.segment(5, 3);

        T servo1 = input(0) + (T) props.servo1_offset;
        T servo2 = input(1) + (T) props.servo2_offset;
        T prop_av = input(2);
        T prop_delta = input(3);

        // thrust is a function of the average propeller speed
        T thrust = getThrust(prop_av) * thrust_scaling;
        // the difference in speed between the two propellers creates a torque
        T torque = getTorque(prop_delta) * torque_scaling;

        // servomotors thrust vector rotation
        Eigen::Matrix<T, 3, 1> thrust_direction;
        thrust_direction << sin(servo2),
                -cos(servo2) * sin(servo1),
                cos(servo1) * cos(servo2);

        Eigen::Matrix<T, 3, 1> thrust_vector = thrust_direction * thrust;

        Eigen::Matrix<T, 3, 1> propeller_torque = thrust_direction * torque;

        Eigen::Matrix<T, 13, 1> x_body = x.segment(0, 13);
        Eigen::Ref<Eigen::Matrix<T, 13, 1>> xdot_body = xdot.segment(0, 13);

        generic_rocket_dynamics(x_body, thrust_vector, propeller_torque, dist_force, dist_torque, xdot_body);
    }

    //thrust 2rd order model
    const double a = -0.000035075437403;
    const double b = 0.029719658777622;
    const double c = -0.341510545964088;

    // roll model: rot_acc = delta * delta_to_acc
    // torque = I * rot_acc
    const double delta_to_acc = -0.1040;

    double getAverageSpeed(const double thrust) const {
        double average = (-b + std::sqrt(b * b - 4 * a * (c - thrust / g))) / (2 * a);
        return average;
    }

    double getHoverSpeedAverage() {
        return getAverageSpeed(g * props.dry_mass);
    }


    template<typename T>
    inline T getThrust(const T prop_average) const {
        T thrust = (a * prop_average * prop_average + b * prop_average + c) * g;
        return thrust;
    }

    template<typename T>
    inline T getTorque(const T prop_delta) const {
        T torque = props.dry_Inertia[2] * prop_delta * delta_to_acc;
        return torque;
    }

    void setParams(double thrust_scaling_val,
                   double torque_scaling_val,
                   double fx, double fy, double fz,
                   double mx, double my, double mz) {
        props.thrust_scaling = thrust_scaling_val;
        props.torque_scaling = torque_scaling_val;
        props.disturbance_force << fx, fy, fz;
        props.disturbance_torque << mx, my, mz;
    }

    template<typename T>
    inline void getParams(parameters_t<T> &params) {
        params << (T) props.thrust_scaling,
                (T) props.torque_scaling,
                (T) props.disturbance_force.x(), (T) props.disturbance_force.y(), (T) props.disturbance_force.z(),
                (T) props.disturbance_torque.x(), (T) props.disturbance_torque.y(), (T) props.disturbance_torque.z();
    }

    void state_dynamics_discrete(const state x0, const control u, double dT, state &x_next) {
        /* RK4 Discretization */
        state k1, k2, k3, k4;

        parameters params;
        getParams(params);

        state_dynamics(x0, u, params, k1);
        state_dynamics((x0 + k1 * dT / 2).eval(), u, params, k2);
        state_dynamics((x0 + k2 * dT / 2).eval(), u, params, k3);
        state_dynamics((x0 + k3 * dT).eval(), u, params, k4);

        x_next = x0 + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    }

};