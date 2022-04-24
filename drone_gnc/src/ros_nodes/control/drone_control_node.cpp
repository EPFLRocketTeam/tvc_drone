/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "drone_control_node.h"

DroneControlNode::DroneControlNode(ros::NodeHandle &nh, Drone *drone) :
        drone(drone), drone_mpc(drone, (mpc_settings = loadMPCSettings(nh), mpc_settings)) {
    initTopics(nh);

    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);
    target_apogee.x = initial_target_apogee.at(0);
    target_apogee.y = initial_target_apogee.at(1);
    target_apogee.z = initial_target_apogee.at(2);

    nh.param("track_guidance", track_guidance, false);

    period = mpc_settings.period;

    // Initialize fsm
    current_fsm.state_machine = rocket_utils::FSM::IDLE;
}

void DroneControlNode::initTopics(ros::NodeHandle &nh) {
    // Subscribers
    bool use_simulation_state;
    nh.param<bool>("/control/use_simulation_state", use_simulation_state, true);
    if (use_simulation_state){
        drone_state_sub = nh.subscribe("/rocket_state", 1, &DroneControlNode::simulationStateCallback, this,
                     ros::TransportHints().tcpNoDelay());
    }
    else{
        drone_state_sub = nh.subscribe("/drone_state", 1, &DroneControlNode::stateCallback, this,
                                       ros::TransportHints().tcpNoDelay());
    }
    target_sub = nh.subscribe("/target_apogee", 1, &DroneControlNode::targetCallback, this);
    target_traj_sub = nh.subscribe("/guidance/horizon", 1, &DroneControlNode::targetTrajectoryCallback, this,
                                   ros::TransportHints().tcpNoDelay());
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneControlNode::fsmCallback, this);

    // Publishers
    horizon_viz_pub = nh.advertise<rocket_utils::Trajectory>("/mpc_horizon", 10);
    gimbal_control_pub = nh.advertise<rocket_utils::GimbalControl>("/gimbal_command_0", 10);
    roll_control_pub = nh.advertise<rocket_utils::ControlMomentGyro>("/cmg_command_0", 10);

    // Debug
    sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
    qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
    horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("debug/horizon", 10);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
}

void DroneControlNode::run() {
    bool emergency_stop_enabled = false;
    if ((emergency_stop && emergency_stop_enabled)) {
        Drone::control control;
        control.setZero();
        publishControl(control);
        ROS_ERROR_STREAM("emergency stop");
    } else {
        double loop_start_time = ros::Time::now().toSec();
        double stall_time = 0;
        // State machine ------------------------------------------

        if ((received_trajectory || !track_guidance) && received_state) {
            fetchNewTarget();

            computeControl();

            publishDebugInfo();

            stall_time = loop_start_time + period * 0.93 - ros::Time::now().toSec();
            if (stall_time > 0) ros::Duration(stall_time).sleep();

            if (current_fsm.state_machine == rocket_utils::FSM::IDLE) {
                start_time = ros::Time::now().toSec();
            } else if (current_fsm.state_machine == rocket_utils::FSM::ASCENT ||
                       current_fsm.state_machine == rocket_utils::FSM::DESCENT) {
                Drone::control interpolated_control = drone_mpc.solution_u_at(0.0);
                publishControl(interpolated_control);
            } else if (current_fsm.state_machine == rocket_utils::FSM::STOP) {
                Drone::control control;
                control.setZero();
                publishControl(control);
            }
        }
        if (stall_time > 0) computation_time = (ros::Time::now().toSec() - loop_start_time - stall_time) * 1000;
        else computation_time = (ros::Time::now().toSec() - loop_start_time) * 1000;
    }


}

void DroneControlNode::fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
    current_fsm = *fsm;
}

void DroneControlNode::simulationStateCallback(const rocket_utils::State::ConstPtr &rocket_state) {
    current_state.state = *rocket_state;
    current_state.thrust_scaling = 1;
    current_state.torque_scaling = 1;
    received_state = true;
}

// Callback function to store last received state
void DroneControlNode::stateCallback(const drone_gnc::DroneExtendedState::ConstPtr &rocket_state) {
//    const std::lock_guard<std::mutex> lock(state_mutex);
    current_state = *rocket_state;
    received_state = true;
}

// Callback function to store last received state
void DroneControlNode::targetCallback(const geometry_msgs::Vector3 &target) {
//    const std::lock_guard<std::mutex> lock(target_mutex);
    target_apogee = target;
}


void DroneControlNode::targetTrajectoryCallback(const drone_gnc::DroneTrajectory::ConstPtr &target) {
    if (!received_trajectory) {
        GUIDANCE_NUM_SEG = target->num_segment;
        GUIDANCE_NUM_NODE = target->num_node;

        guidance_state_trajectory = Eigen::MatrixXd(Drone::NX, GUIDANCE_NUM_NODE);
        guidance_control_trajectory = Eigen::MatrixXd(Drone::NU, GUIDANCE_NUM_NODE);

        if (GUIDANCE_NUM_SEG != 0) {
            GUIDANCE_POLY_ORDER = GUIDANCE_NUM_NODE / GUIDANCE_NUM_SEG;
            SEG_LENGTH = 1.0 / GUIDANCE_NUM_SEG;
            // init lagrange basis
            guidance_t0 = target->trajectory.at(0).header.stamp.toSec();
            guidance_tf = target->trajectory.at(GUIDANCE_NUM_NODE - 1).header.stamp.toSec();
            double traj_length = guidance_tf - guidance_t0;
            VectorXd time_grid(GUIDANCE_POLY_ORDER + 1, 1);
            for (int i = 0; i < GUIDANCE_POLY_ORDER + 1; i++) {
                time_grid(i) = (target->trajectory.at(i).header.stamp.toSec() - guidance_t0) / traj_length;
            }
            m_basis = Eigen::MatrixXd(GUIDANCE_POLY_ORDER + 1, GUIDANCE_POLY_ORDER + 1);
            polympc::LagrangeSpline::compute_lagrange_basis(time_grid, m_basis);
        } else {
            //fixed guidance
            GUIDANCE_POLY_ORDER = 0;
            SEG_LENGTH = 0;
        }
        received_trajectory = true;
    }

    guidance_t0 = target->trajectory.at(0).header.stamp.toSec();
    guidance_tf = target->trajectory.at(GUIDANCE_NUM_NODE - 1).header.stamp.toSec();

    int i = 0;
    for (auto waypoint: target->trajectory) {
        guidance_state_trajectory.col(i)
                << waypoint.state.state.pose.position.x, waypoint.state.state.pose.position.y, waypoint.state.state.pose.position.z,
                waypoint.state.state.twist.linear.x, waypoint.state.state.twist.linear.y, waypoint.state.state.twist.linear.z,
                waypoint.state.state.pose.orientation.x, waypoint.state.state.pose.orientation.y, waypoint.state.state.pose.orientation.z, waypoint.state.state.pose.orientation.w,
                waypoint.state.state.twist.angular.x, waypoint.state.state.twist.angular.y, waypoint.state.state.twist.angular.z;

        guidance_control_trajectory.col(i)
                << 0, 0, drone->getAverageSpeed(waypoint.gimbal_control.thrust), 0;
        i++;
    }
}

void DroneControlNode::sampleTargetTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_state_traj,
                                              Matrix<double, Drone::NU, DroneMPC::num_nodes> &mpc_target_control_traj) {
    double time_now = ros::Time::now().toSec();
    //first value is to make startup work, second value is used in general
    double time_delta = std::min(time_now - start_time, time_now - guidance_t0);

    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        //scaled time;
        double t = (time_delta + drone_mpc.node_time(i) + drone_mpc.period) / (guidance_tf - guidance_t0);

        //Clip between 0 and 1;
        t = max(min(t, 1.0), 0.0);

        Eigen::Index idx = std::floor(t / SEG_LENGTH);
        idx = std::max(Eigen::Index(0), std::min(idx, (Eigen::Index) GUIDANCE_NUM_SEG - 1));

        MatrixXd state_guidance_segment = guidance_state_trajectory.block(0, idx * GUIDANCE_POLY_ORDER, Drone::NX,
                                                                          GUIDANCE_POLY_ORDER + 1);
        mpc_target_state_traj.col(i) = polympc::LagrangeSpline::eval(t - idx * SEG_LENGTH, state_guidance_segment,
                                                                     m_basis);

        MatrixXd control_guidance_segment = guidance_control_trajectory.block(0, idx * GUIDANCE_POLY_ORDER, Drone::NX,
                                                                              GUIDANCE_POLY_ORDER + 1);
        mpc_target_control_traj.col(i) = polympc::LagrangeSpline::eval(t - idx * SEG_LENGTH, control_guidance_segment,
                                                                       m_basis);
    }
}


void
DroneControlNode::sampleTargetTrajectoryLinear(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_state_traj,
                                               Matrix<double, Drone::NU, DroneMPC::num_nodes> &mpc_target_control_traj) {
    double time_now = ros::Time::now().toSec();
    //first value is to make startup work, second value is used in general
    double time_delta = std::min(time_now - start_time, time_now - guidance_t0);

    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        double t = time_delta + drone_mpc.node_time(i);
        //TODO remove constants
        double idx = t * GUIDANCE_NUM_NODE / (guidance_tf - guidance_t0);

        //linear interpolation
        int l_idx = floor(idx);
        l_idx = min(l_idx, GUIDANCE_NUM_NODE - 1);
        Drone::state l_state = guidance_state_trajectory.col(l_idx);
        Drone::control l_control = guidance_control_trajectory.col(l_idx);

        int u_idx = ceil(idx);
        u_idx = min(u_idx, GUIDANCE_NUM_NODE - 1);
        Drone::state u_state = guidance_state_trajectory.col(u_idx);
        Drone::control u_control = guidance_control_trajectory.col(u_idx);

        Drone::state state = l_state + (u_state - l_state) * (idx - l_idx);
        Drone::control control = l_control + (u_control - l_control) * (idx - l_idx);
        mpc_target_state_traj.col(i) = state;
        mpc_target_control_traj.col(i) = control;
    }
}


void DroneControlNode::computeControl() {
    time_compute_start = ros::Time::now().toSec();

    Drone::state x0;
//    state_mutex.lock();
    x0 << current_state.state.pose.position.x, current_state.state.pose.position.y, current_state.state.pose.position.z,
            current_state.state.twist.linear.x, current_state.state.twist.linear.y, current_state.state.twist.linear.z,
            current_state.state.pose.orientation.x, current_state.state.pose.orientation.y, current_state.state.pose.orientation.z, current_state.state.pose.orientation.w,
            current_state.state.twist.angular.x, current_state.state.twist.angular.y, current_state.state.twist.angular.z;
//    state_mutex.unlock();
    double attitude_angle_cos = x0(9) * x0(9) - x0(6) * x0(6) - x0(7) * x0(7) + x0(8) * x0(8);
    if (attitude_angle_cos < cos(0.5) || abs(x0(10)) > 4 || abs(x0(11)) > 4 || abs(x0(12)) > 4) {
        emergency_stop = true;
    }

    drone->setParams(current_state.thrust_scaling,
                     current_state.torque_scaling,
                     current_state.disturbance_force.x, current_state.disturbance_force.y,
                     current_state.disturbance_force.z,
                     current_state.disturbance_torque.x, current_state.disturbance_torque.y,
                     current_state.disturbance_torque.z);

    drone_mpc.solve(x0);

    //TODO
    if (isnan(drone_mpc.solution_x_at(0)(0))) {
        drone_mpc.reset();
        ROS_ERROR("MPC ISSUE\n");
    }

}

void DroneControlNode::toROS(const Drone::control &control, rocket_utils::GimbalControl &gimbal_control,
                             rocket_utils::ControlMomentGyro &roll_control) {
    double servo1 = control(0);
    double servo2 = control(1);
    double prop_av = control(2);
    double prop_delta = control(3);

    // saturation
    servo1 = std::min(std::max(servo1, -drone->props.max_servo1_angle), drone->props.max_servo1_angle);
    servo2 = std::min(std::max(servo2, -drone->props.max_servo2_angle), drone->props.max_servo2_angle);
    double bottom = prop_av - prop_delta / 2;
    double top = prop_av + prop_delta / 2;
    bottom = std::min(std::max(bottom, drone->props.min_propeller_speed), drone->props.max_propeller_speed);
    top = std::min(std::max(top, drone->props.min_propeller_speed), drone->props.max_propeller_speed);
    prop_av = 0.5 * (bottom + top);
    prop_delta = top - bottom;

    gimbal_control.outer_angle = control(0);
    gimbal_control.inner_angle = control(1);
    gimbal_control.thrust = drone->getThrust(prop_av);

    roll_control.outer_angle = control(0);
    roll_control.inner_angle = control(1);
    roll_control.torque = drone->getTorque(prop_delta);
}

void DroneControlNode::publishControl(Drone::control &control) {
    rocket_utils::GimbalControl gimbal_control;
    rocket_utils::ControlMomentGyro roll_control;
    toROS(control, gimbal_control, roll_control);

    ros::Time time_now = ros::Time::now();
    gimbal_control.header.stamp = time_now;
    roll_control.header.stamp = time_now;

    gimbal_control_pub.publish(gimbal_control);
    roll_control_pub.publish(roll_control);
}

void DroneControlNode::publishTrajectory() {
    // Send optimal trajectory computed by control for debugging purpose
    rocket_utils::Trajectory trajectory_msg;
    drone_gnc::DroneTrajectory horizon_msg;
    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        Drone::state state_val = drone_mpc.solution_x_at(i);

        rocket_utils::Waypoint point;
        point.time = drone_mpc.node_time(i);
        point.position.x = state_val(0);
        point.position.y = state_val(1);
        point.position.z = state_val(2);
        trajectory_msg.trajectory.push_back(point);

        drone_gnc::DroneExtendedState state_msg;
        state_msg.state.pose.position.x = state_val(0);
        state_msg.state.pose.position.y = state_val(1);
        state_msg.state.pose.position.z = state_val(2);

        state_msg.state.twist.linear.x = state_val(3);
        state_msg.state.twist.linear.y = state_val(4);
        state_msg.state.twist.linear.z = state_val(5);

        state_msg.state.pose.orientation.x = state_val(6);
        state_msg.state.pose.orientation.y = state_val(7);
        state_msg.state.pose.orientation.z = state_val(8);
        state_msg.state.pose.orientation.w = state_val(9);

        state_msg.state.twist.angular.x = state_val(10);
        state_msg.state.twist.angular.y = state_val(11);
        state_msg.state.twist.angular.z = state_val(12);

        Drone::control control_val = drone_mpc.solution_u_at(i);
        rocket_utils::GimbalControl gimbal_control_msg;
        rocket_utils::ControlMomentGyro roll_control_msg;
        toROS(control_val, gimbal_control_msg, roll_control_msg);

        drone_gnc::DroneWaypointStamped state_msg_stamped;
        state_msg_stamped.state = state_msg;
        state_msg_stamped.gimbal_control = gimbal_control_msg;
        state_msg_stamped.roll_control = roll_control_msg;
        state_msg_stamped.header.stamp = ros::Time::now() + ros::Duration(drone_mpc.node_time(i));
        state_msg_stamped.header.frame_id = ' ';

        horizon_msg.trajectory.push_back(state_msg_stamped);
    }
    horizon_msg.header.stamp = ros::Time::now();
    horizon_viz_pub.publish(trajectory_msg);
    horizon_pub.publish(horizon_msg);
}


void DroneControlNode::fetchNewTarget() {
    Drone::state target_state;
    Drone::control target_control;

//    target_mutex.lock();
    target_state << target_apogee.x, target_apogee.y, target_apogee.z,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
//    target_mutex.unlock();

    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;

    Matrix<double, Drone::NX, DroneMPC::num_nodes> mpc_target_state_traj;
    Matrix<double, Drone::NU, DroneMPC::num_nodes> mpc_target_control_traj;
    if (!track_guidance) {
        for (int i = 0; i < DroneMPC::num_nodes; i++) {
            mpc_target_state_traj.col(i) = target_state;
            mpc_target_control_traj.col(i) = target_control;
        }
    } else if (GUIDANCE_NUM_SEG == 0) {
        sampleTargetTrajectoryLinear(mpc_target_state_traj, mpc_target_control_traj);
    } else {
        drone_mpc.setMaximumHorizonLength(guidance_tf - ros::Time::now().toSec() - drone_mpc.period);
        sampleTargetTrajectory(mpc_target_state_traj, mpc_target_control_traj);
        //TODO remove
        for (int i = 0; i < DroneMPC::num_nodes; i++) {
            mpc_target_control_traj.col(i) = target_control;
        }
    }

    drone_mpc.setTargetStateTrajectory(mpc_target_state_traj);
    drone_mpc.setTargetControlTrajectory(mpc_target_control_traj);
}

void DroneControlNode::publishDebugInfo() {
    publishTrajectory();

    std_msgs::Int32 msg1;
    msg1.data = drone_mpc.info().iter;
    sqp_iter_pub.publish(msg1);
    std_msgs::Int32 msg2;
    msg2.data = drone_mpc.info().qp_solver_iter;
    qp_iter_pub.publish(msg2);
    std_msgs::Float64 msg3;
    msg3.data = computation_time;
    computation_time_pub.publish(msg3);
}


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("control");

    DroneProps<double> drone_props = loadDroneProps(nh);
    Drone drone(drone_props);

    DroneControlNode drone_control_node(nh, &drone);

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(drone_control_node.period), [&](const ros::TimerEvent &) {
        drone_control_node.run();
    });
    ros::spin();
}
