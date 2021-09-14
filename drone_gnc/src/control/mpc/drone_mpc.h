/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */

#ifndef SRC_DRONE_MPC_H
#define SRC_DRONE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "polympc_redef.hpp"

class DroneMPC : private MPC<DroneControlOCP, Solver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;

    using MPC::num_nodes;

    DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(Drone::state &x0);

    drone_gnc::DroneControl getControlMessage(double t);

    Drone::state solution_x_at(const double t);

    Drone::control solution_u_at(const double t);

    Drone::state solution_x_at(const int t);

    Drone::control solution_u_at(const int t);

    void setHorizonLength(double horizon_length);

    double node_time(int i);

    using MPC::info;

    void reset();

    void integrateX0(const Drone::state x0, Drone::state &new_x0);

    void setTargetStateTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> target_state_trajectory);
    void setTargetControlTrajectory(Matrix<double, Drone::NU, DroneMPC::num_nodes> target_control_trajectory);


    std::shared_ptr<Drone> drone;
    double mpc_period;
    double feedforward_period;
    double fixed_computation_time;
    double last_computation_time = 0;
    double init_time;
    double max_horizon_length;

private:
    void warmStart();

    double solution_time;
    bool is_simu;
};


#endif //SRC_DRONE_MPC_H
