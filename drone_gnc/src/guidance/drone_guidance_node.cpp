#include "drone_guidance_node.h"

DroneGuidanceNode::DroneGuidanceNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone_mpc(nh, drone_ptr),
                                                                                              drone(drone_ptr) {
    initTopics(nh);

    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);
    target_apogee.x = initial_target_apogee.at(0);
    target_apogee.y = initial_target_apogee.at(1);
    target_apogee.z = initial_target_apogee.at(2);

    // Initialize fsm
    client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    nh.getParam("mpc/mpc_period", period);
}


void DroneGuidanceNode::initTopics(ros::NodeHandle &nh) {
    // Subscribers
    rocket_state_sub = nh.subscribe("/simu_drone_state", 100, &DroneGuidanceNode::stateCallback, this);
    target_sub = nh.subscribe("/target_apogee", 100, &DroneGuidanceNode::targetCallback, this);

    // Publishers
    horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);

    // Debug
    sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
    qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
    horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("horizon", 10);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
}

void DroneGuidanceNode::run(){
    double loop_start_time = ros::Time::now().toSec();
    if (received_state) {
        fetchNewTarget();
        computeTrajectory();
        publishTrajectory();
        publishDebugInfo();
    }
}

void DroneGuidanceNode::stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
    current_state = *rocket_state;
    received_state = true;
}

void DroneGuidanceNode::targetCallback(const geometry_msgs::Vector3 &target) {
    target_apogee = target;
}

void DroneGuidanceNode::computeTrajectory() {
    time_compute_start = ros::Time::now();

    Drone::state x0;
    x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
            current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
            current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
            current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

    drone_mpc.drone->setParams(current_state.thrust_scaling,
                               current_state.torque_scaling,
                               current_state.servo1_offset, current_state.servo2_offset,
                               0, 0, 0,
                               0, 0, 0);

    drone_mpc.solve(x0);

    //TODO
    if (isnan(drone_mpc.solution_x_at(0)(0))) {
        ROS_ERROR("MPC ISSUE\n");
        DroneGuidanceMPC::ocp_state x0;
        x0 << 0, 0, 0,
                0, 0, 0;
        drone_mpc.x_guess(x0.cwiseProduct(drone_mpc.ocp().x_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));
        DroneGuidanceMPC::ocp_control u0;
        u0 << 0, 0, drone->getHoverSpeedAverage();
        drone_mpc.u_guess(u0.cwiseProduct(drone_mpc.ocp().u_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));

        DroneGuidanceMPC::dual_var_t dual;
        dual.setZero();
        drone_mpc.lam_guess(dual);
    }
}

void DroneGuidanceNode::publishTrajectory() {
    // Send optimal trajectory computed by control. Send only position for now
    drone_gnc::Trajectory trajectory_msg;
    drone_gnc::DroneTrajectory horizon_msg;

    for (int i = 0; i < drone_mpc.ocp().NUM_NODES; i++) {
        Drone::state state_val = drone_mpc.solution_x_at(i);

        drone_gnc::Waypoint point;
        point.time = drone_mpc.node_time(i);
        point.position.x = state_val(0);
        point.position.y = state_val(1);
        point.position.z = state_val(2);
        trajectory_msg.trajectory.push_back(point);

        drone_gnc::DroneState state_msg;
        state_msg.pose.position.x = state_val(0);
        state_msg.pose.position.y = state_val(1);
        state_msg.pose.position.z = state_val(2);

        state_msg.twist.linear.x = state_val(3);
        state_msg.twist.linear.y = state_val(4);
        state_msg.twist.linear.z = state_val(5);

        state_msg.pose.orientation.x = state_val(6);
        state_msg.pose.orientation.y = state_val(7);
        state_msg.pose.orientation.z = state_val(8);
        state_msg.pose.orientation.w = state_val(9);

        state_msg.twist.angular.x = state_val(10);
        state_msg.twist.angular.y = state_val(11);
        state_msg.twist.angular.z = state_val(12);

        Drone::control control_val = drone_mpc.solution_u_at(i);
        drone_gnc::DroneControl control_msg;
        control_msg.servo1 = control_val(0);
        control_msg.servo2 = control_val(1);
        control_msg.bottom = control_val(2);
        control_msg.top = control_val(3);

        drone_gnc::DroneWaypointStamped state_msg_stamped;
        state_msg_stamped.state = state_msg;
        state_msg_stamped.control = control_msg;
        state_msg_stamped.header.stamp = time_compute_start + ros::Duration(drone_mpc.node_time(i));
        state_msg_stamped.header.frame_id = ' ';

        horizon_msg.trajectory.push_back(state_msg_stamped);
    }
    horizon_viz_pub.publish(trajectory_msg);
    horizon_pub.publish(horizon_msg);
}


void DroneGuidanceNode::fetchNewTarget() {
    DroneGuidanceMPC::ocp_state target_state;
    DroneGuidanceMPC::ocp_control target_control;

    target_state << target_apogee.x, target_apogee.y, target_apogee.z,
            0, 0, 0;
    target_control << 0, 0, drone->getHoverSpeedAverage();

    drone_mpc.setTarget(target_state, target_control);
}

void DroneGuidanceNode::publishDebugInfo() {
    std_msgs::Int32 msg1; msg1.data = drone_mpc.info().iter;
    sqp_iter_pub.publish(msg1);
    std_msgs::Int32 msg2; msg2.data = drone_mpc.info().qp_solver_iter;
    qp_iter_pub.publish(msg2);
    std_msgs::Float64 msg3; msg3.data = drone_mpc.last_computation_time;
    computation_time_pub.publish(msg3);
}


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    std::shared_ptr<Drone> drone = make_shared<Drone>(nh);
    DroneGuidanceNode droneGuidanceNode(nh, drone);

//    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(droneGuidanceNode.period), [&](const ros::TimerEvent &) {
        droneGuidanceNode.run();
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

}
