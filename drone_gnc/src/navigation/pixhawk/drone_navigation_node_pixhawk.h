/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */

#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "nav_msgs/Odometry.h"

#include <std_msgs/Float64.h>

#include "drone_EKF_pixhawk.hpp"

class DroneNavigationNodePixhawk {
private:
    DroneEKFPixhawk kalman;

    drone_gnc::FSM current_fsm;
    drone_gnc::DroneControl current_control;
    drone_gnc::DroneControl previous_control;
    bool received_pixhawk = false;
    bool received_optitrack = false;
    bool initialized_orientation = false;
    double last_predict_time;
    double last_computation_time = 0;
    Drone::state measured_drone_state;
    Eigen::Vector3d origin;
    Eigen::Quaterniond initial_orientation;
    bool use_gps;
    bool update_trigger = false;
    double init_time;

    ros::Publisher kalman_pub;
    ros::Publisher computation_time_pub;
    ros::Subscriber fsm_sub;
    ros::Subscriber control_sub;
    ros::Subscriber optitrack_sub;
    ros::Subscriber pixhawk_pose_sub;
    ros::Subscriber pixhawk_twist_local_sub;
    ros::Subscriber pixhawk_twist_body_sub;
    ros::Subscriber pixhawk_ekf_sub;

public:
    double period;

    DroneNavigationNodePixhawk(ros::NodeHandle &nh);

    void initTopics(ros::NodeHandle &nh);

    void kalmanStep();

    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm);

    void pixhawkEKFCallback(const nav_msgs::Odometry::ConstPtr &state);

    void pixhawkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

    void pixhawkTwistBodyCallback(const geometry_msgs::TwistStamped::ConstPtr &twist);

    void pixhawkTwistLocalCallback(const geometry_msgs::TwistStamped::ConstPtr &twist);

    void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

    void controlCallback(const drone_gnc::DroneControl::ConstPtr &control);

    void publishDroneState();
};