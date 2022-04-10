/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include "control_solver.hpp"
#include "control/mpc_wrapper_rewrite.hpp"
#include "control/collocation_transcription.hpp"

using TranscribedDroneControlOCP = polympc::CollocationTranscription<DroneControlOCP, NUM_SEG, POLY_ORDER, polympc::SPARSE, polympc::CLENSHAW_CURTIS, false>;

class DroneMPC : private polympc::MPCRewrite<TranscribedDroneControlOCP, ControlSolver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;
    using MPC = MPCRewrite;

    using MPC::num_nodes;

    double period;
    double horizon_length;

    DroneMPC(Drone *drone, ControlMPCSettings<scalar_t> &mpc_settings);

    void solve(Drone::state &x0, bool constrain_servo_rate_between_iterations=true);

    Drone::state solution_x_at(double t);

    Drone::control solution_u_at(double t);

    Drone::state solution_x_at(int t);

    Drone::control solution_u_at(int t);

    inline void state_bounds(const Eigen::Ref<const state_t> &xlb, const Eigen::Ref<const state_t> &xub) {
        MPC::state_bounds(xlb.cwiseProduct(ocp().x_scaling_vec), xub.cwiseProduct(ocp().x_scaling_vec));
    }

    inline void control_bounds(const Eigen::Ref<const control_t> &lb, const Eigen::Ref<const control_t> &ub) {
        MPC::control_bounds(lb.cwiseProduct(ocp().u_scaling_vec), ub.cwiseProduct(ocp().u_scaling_vec));
    }

    void setMaximumHorizonLength(double horizon_length);

    double node_time(int i);

    using MPC::info;

    void reset();

    void integrateX0(const Drone::state x0, Drone::state &new_x0);

    void setTargetStateTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> target_state_trajectory);

    void setTargetControlTrajectory(Matrix<double, Drone::NU, DroneMPC::num_nodes> target_control_trajectory);

    void setTargetState(Drone::state target_state);

    void setTargetControl(Drone::control target_control);

private:
    Drone *drone;
};