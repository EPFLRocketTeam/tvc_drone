/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include <drone_model.hpp>

#include "guidance_solver.hpp"
#include "control/mpc_wrapper_rewrite.hpp"
#include "control/collocation_transcription.hpp"

using TranscribedDroneGuidanceOCP = polympc::CollocationTranscription<DroneGuidanceOCP, NUM_SEG_G, POLY_ORDER_G, polympc::SPARSE, polympc::CLENSHAW_CURTIS, false>;

class DroneGuidance : private polympc::MPCRewrite<TranscribedDroneGuidanceOCP, GuidanceSolver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;
    using MPC = MPCRewrite;

    Drone::state target_apogee;
    Drone::state target_land;

    using MPC::num_nodes;

    DroneGuidance(Drone *drone, GuidanceSettings<double> &guidance_settings);

    void solve(Drone::state &x0);

    void initGuess(Drone::state &x0, Drone::state &target);

    void setTargetState(Drone::state &target_state);

    Drone::state solution_x_at(int t);

    Drone::control solution_u_at(int t);

    Drone::state solution_x_at(double t);

    Drone::control solution_u_at(double t);

    using MPC::solution_p;

    void setDescentConstraints();

    double node_time(int i);

    void precomputeDescent();

    void warmStartDescent(Drone::state x0);

    using MPC::info;

private:
    Drone *drone;

    GuidanceSettings<double> guidance_settings;

    traj_control_t descent_control_sol;
    traj_state_t descent_state_sol;
    dual_var_t descent_dual_sol;
    parameter_t descent_p_sol;
};