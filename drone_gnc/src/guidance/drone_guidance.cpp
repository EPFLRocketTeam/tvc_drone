/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "drone_guidance.h"

DroneGuidance::DroneGuidance(Drone *drone, GuidanceSettings<double> &guidance_settings) :
        drone(drone), guidance_settings(guidance_settings) {
    ocp().init(drone, guidance_settings);

    settings().max_iter = guidance_settings.max_sqp_iter;
    settings().line_search_max_iter = guidance_settings.max_line_search_iter;
    qp_settings().max_iter = guidance_settings.max_qp_iter;

    set_time_limits(0, 1);

    // Setup constraints
    const double inf = std::numeric_limits<double>::infinity();
    double eps = 1e-3;
    ocp_state lbx, ubx;
    ocp_control lbu, ubu;
    ocp_control lbu_f, ubu_f;
    lbx << -inf, -inf, guidance_settings.min_z - eps,
            -inf, -inf, guidance_settings.min_dz;
    ubx << inf, inf, inf,
            inf, inf, guidance_settings.max_dz;

    lbu << drone->props.min_propeller_speed, -inf, -guidance_settings.max_attitude_angle; // lower bound on control
    ubu << drone->props.max_propeller_speed, inf, guidance_settings.max_attitude_angle; // upper bound on control


    lbu_f << drone->props.min_propeller_speed, -eps, -eps; // lower bound on control
    ubu_f << drone->props.max_propeller_speed, eps, eps; // upper bound on control=

    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);
    final_control_bounds(lbu_f, ubu_f);

    //minimal time setup
    parameter_t lbp;
    parameter_t ubp;
    lbp << 1e-3;
    ubp << guidance_settings.max_horizon_length;
    parameters_bounds(lbp, ubp);

    Drone::state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    target_apogee
            << guidance_settings.target_apogee_vec[0], guidance_settings.target_apogee_vec[1], guidance_settings.target_apogee_vec[2],
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    target_land
            << guidance_settings.target_land_vec[0], guidance_settings.target_land_vec[1], guidance_settings.target_land_vec[2],
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    precomputeDescent();

    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);
    final_control_bounds(lbu_f, ubu_f);

    setTargetState(target_apogee);
    initGuess(x0, target_apogee);
}

void DroneGuidance::initGuess(Drone::state &x0, Drone::state &target_state) {
    // Initial guess
    double z0 = x0(2);
    double z_target = target_state(2);

    parameter_t p0;
    Eigen::MatrixXd traj_state_guess = x0.head(6).replicate(1, ocp().NUM_NODES);

    ocp_control u0;
    u0 << 0, 0, drone->props.max_propeller_speed;
    Eigen::MatrixXd traj_control_guess = u0.replicate(1, ocp().NUM_NODES);
    double t_end;
    if (true) {
        // initial guess assuming full thrust for first part, then minimum thrust for second part
        double speed1, speed2;
        if (z_target - z0 > 0) {
            speed1 = drone->props.max_propeller_speed;
            speed2 = drone->props.min_propeller_speed;
        } else {
            speed1 = drone->props.min_propeller_speed;
            speed2 = drone->props.max_propeller_speed;
        }

        double a1 = drone->getThrust(speed1) / drone->props.dry_mass - 9.81;
        double a2 = drone->getThrust(speed2) / drone->props.dry_mass - 9.81;

        // obtained by solving delta_z = 1/2 a1 t_mid^2 + v_mid (t_end-t_mid) + 1/2 a2 (t_end-t_mid)^2, v_end = 0
        double t_mid = sqrt(2 * (z_target - z0) / (a1 * (1 - a1 / a2)));
        t_end = t_mid - a1 / a2 * t_mid;
        double z_mid = z0 + 0.5 * a1 * t_mid * t_mid;
        double v_mid = a1 * t_mid;

        std::cout << "z0: " << z0 << " target: " << z_target << std::endl;
        std::cout << "traj length new guess: " << t_end << std::endl;

        RowVectorXd z_guess(DroneGuidanceOCP::NUM_NODES);
        RowVectorXd dz_guess(DroneGuidanceOCP::NUM_NODES);
        RowVectorXd prop_av_guess(DroneGuidanceOCP::NUM_NODES);
        for (int i = 0; i < time_grid.size(); i++) {
            double t = time_grid(i) * t_end;
            if (t < t_mid) {
                z_guess(i) = z0 + 0.5 * a1 * t * t;
                dz_guess(i) = a1 * t;
                prop_av_guess(i) = speed1;
            } else {
                z_guess(i) = z_mid + v_mid * (t - t_mid) + 0.5 * a2 * (t - t_mid) * (t - t_mid);
                dz_guess(i) = v_mid + a2 * (t - t_mid);
                prop_av_guess(i) = speed2;
            }
        }

        traj_state_guess.row(2) = z_guess;
        traj_state_guess.row(5) = dz_guess;
        traj_control_guess.row(0) = prop_av_guess;
//        ROS_INFO_STREAM("traj guess " << traj_state_guess);
//        ROS_INFO_STREAM("control guess " << traj_control_guess);
        //hack to make it fine if dz is small
        t_end = std::max(2.0, t_end);

        double delta_x = target_state(0) - x0(0);
        double delta_y = target_state(1) - x0(1);
        VectorXd time_grid_2 = time_grid.cwiseProduct(time_grid);
        VectorXd time_grid_3 = time_grid.cwiseProduct(time_grid_2);

        ArrayXd dx_guess = 6 * delta_x / t_end * (time_grid - time_grid_2);
        ArrayXd dy_guess = 6 * delta_y / t_end * (time_grid - time_grid_2);

        RowVectorXd x_guess = x0(0) + (delta_x * (3 * time_grid_2 - 2 * time_grid_3)).array();
        RowVectorXd y_guess = x0(1) + (delta_y * (3 * time_grid_2 - 2 * time_grid_3)).array();

        RowVectorXd fx_guess = 6 * delta_x * (1 - 2 * time_grid.array());
        RowVectorXd fy_guess = 6 * delta_y * (1 - 2 * time_grid.array());

        traj_state_guess.row(0) = x_guess;
        traj_state_guess.row(1) = y_guess;
        traj_state_guess.row(3) = dx_guess;
        traj_state_guess.row(4) = dy_guess;
        traj_control_guess.row(1).setConstant(1e-3);
        traj_control_guess.row(2).setConstant(1e-3);

        p0 << t_end;
    } else {
        p0 << 1.0;
    }

    p_guess(p0);

    //TODO scaling
    traj_state_guess.resize(varx_size, 1);
    x_guess(traj_state_guess);

    traj_control_guess.resize(varu_size, 1);
    u_guess(traj_control_guess);

    dual_var_t dual;
    dual.setZero();
    lam_guess(dual);
}

void DroneGuidance::setTargetState(Drone::state &target_state) {
    ocp().xs << target_state.head(6);
}

Drone::state DroneGuidance::solution_x_at(const int t) {
    Drone::state state_sol;
    ocp_state sol = MPC::solution_x_at(t);
    Vector3d tangent = sol.segment(3, 3);
    if (tangent(2) < 0) tangent = -tangent;
    Vector3d unit_z;
    unit_z << 0, 0, 1;
//    Quaterniond tangent_orientation = Quaterniond::FromTwoVectors(unit_z, tangent);
    state_sol << sol,
//            tangent_orientation.coeffs(),
            0, 0, 0, 1,
            0, 0, 0;
    return state_sol;
}

Drone::control DroneGuidance::solution_u_at(const int t) {
    Drone::control sol;
    sol << 0, 0, MPC::solution_u_at(t)(0), 0;
    return sol;
}

Drone::state DroneGuidance::solution_x_at(const double t) {
    double guidance_time_length = solution_p()(0);
    double t_scaled = t / guidance_time_length;
    t_scaled = max(min(t_scaled, 1.0), 0.0); //Clip between 0 and 1;
    Drone::state state_sol;
    ocp_state sol = MPC::solution_x_at(t_scaled);
    Vector3d tangent = sol.segment(3, 3);
    if (tangent(2) < 0) tangent = -tangent;
    Vector3d unit_z;
    unit_z << 0, 0, 1;
//    Quaterniond tangent_orientation = Quaterniond::FromTwoVectors(unit_z, tangent);
    state_sol << sol,
//            tangent_orientation.coeffs(),
            0, 0, 0, 1,
            0, 0, 0;
    return state_sol;
}

Drone::control DroneGuidance::solution_u_at(const double t) {
    double guidance_time_length = solution_p()(0);
    double t_scaled = t / guidance_time_length;
    t_scaled = max(min(t_scaled, 1.0), 0.0); //Clip between 0 and 1;
    Drone::control sol;
    sol << 0, 0, MPC::solution_u_at(t_scaled)(0), 0;
    return sol;
}

double DroneGuidance::node_time(int i) {
    return time_grid(i) * solution_p()(0);
}

void DroneGuidance::solve(Drone::state &x0_full) {
    const double inf = std::numeric_limits<double>::infinity();

    double eps = 1e-3;
    ocp_state x0 = x0_full.head(6);
    initial_conditions(x0.array() - eps, x0.array() + eps);

    //final state bound
    ocp_state lbx_f, ubx_f;
    lbx_f << -inf, -inf, ocp().xs.segment(2, 4);
    ubx_f << inf, inf, ocp().xs.segment(2, 4);
    final_state_bounds(lbx_f.array() - eps, ubx_f.array() + eps);

    MPC::solve();
}

void DroneGuidance::precomputeDescent() {
    std::cout << "Precomputing Descent" << std::endl;

    setTargetState(target_land);
    initGuess(target_apogee, target_land);

    setDescentConstraints();
    for (int i = 0; i < 10; i++) {
        solve(target_apogee);
    }

    descent_control_sol = solution_u();
    descent_state_sol = solution_x();
    descent_dual_sol = solution_dual();
    descent_p_sol = solution_p();
    std::cout << "Done Precomputing Descent" << std::endl;
}

void DroneGuidance::warmStartDescent(Drone::state x0) {
    u_guess(descent_control_sol);
    for (int i = 0; i < num_nodes; i++) {
        descent_state_sol(i * ocp().NX) += x0(0) - target_apogee(0);
        descent_state_sol(i * ocp().NX + 1) += x0(1) - target_apogee(1);
    }
    x_guess(descent_state_sol);
    lam_guess(descent_dual_sol);
    p_guess(descent_p_sol);
}

void DroneGuidance::setDescentConstraints() {
    ocp_control lbu, ubu;
    //TODO
    const double inf = std::numeric_limits<double>::infinity();
    lbu
            << guidance_settings.descent_min_propeller_speed, -inf, -guidance_settings.max_attitude_angle; // lower bound on control
    ubu << drone->props.max_propeller_speed, inf, guidance_settings.max_attitude_angle; // upper bound on control
    control_bounds(lbu, ubu);

    double eps = 1e-5;
    lbu << guidance_settings.descent_min_propeller_speed, -eps, -eps; // lower bound on control
    ubu << drone->props.max_propeller_speed, eps, eps; // upper bound on control=
    final_control_bounds(lbu, ubu);
}