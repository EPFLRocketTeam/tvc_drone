#include "ros/ros.h"

#include "drone_gnc/DroneControl.h"
#include "real_time_simulator/State.h"
#include "drone_gnc/DroneState.h"
#include "real_time_simulator/Control.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "drone_model.hpp"

using namespace Eigen;

ros::Publisher rocket_control_pub, drone_state_pub, command_pub, fake_optitrack_pub;

double CM_to_thrust_distance;


Matrix<double, 2, 2> sysA;
Matrix<double, 2, 1> sysB;
Matrix<double, 1, 2> sysC;
Matrix<double, 2, 1> x_servo1;
Matrix<double, 2, 1> x_servo2;

double CM_OFFSET_X = 0;

bool first_command = true;
double thrust_scaling, torque_scaling, servo1_offset, servo2_offset;
Drone *drone;

void publishConvertedControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {

    double thrust = thrust_scaling * drone->getThrust((drone_control->bottom + drone_control->top) * 0.5);
    double torque = torque_scaling * drone->getTorque(drone_control->top - drone_control->bottom);

    // ss model for fixed ts//TODO use integrator time step instead
    x_servo1 = sysA * x_servo1 + sysB * ((double) drone_control->servo1);
    double servo1 = sysC * x_servo1;

    x_servo2 = sysA * x_servo2 + sysB * ((double) drone_control->servo2);
    double servo2 = sysC * x_servo2;

    //quaternion representing the rotation of the servos around the Y-axis followed by the rotation around the X-axis
    Eigen::Quaterniond
            thrust_rotation(
            AngleAxisd(drone_control->servo1 + servo1_offset, Vector3d::UnitY()) *
            AngleAxisd(drone_control->servo2 + servo2_offset, Vector3d::UnitX())
    );

    //rotated thrust vector, in body frame
    Eigen::Vector3d thrust_direction = thrust_rotation._transformVector(Vector3d::UnitZ());
    Eigen::Vector3d thrust_vector = thrust_direction * thrust;

    Eigen::Vector3d propeller_torque = thrust_direction * torque;

    //compute the force and torque at the center of mass and publish them
    real_time_simulator::Control converted_control;


    converted_control.torque.x = thrust_vector.y() * CM_to_thrust_distance + propeller_torque.x();
    converted_control.torque.y =
            -thrust_vector.x() * CM_to_thrust_distance + propeller_torque.y() - CM_OFFSET_X * thrust_vector.z();
    converted_control.torque.z = torque + propeller_torque.z() + CM_OFFSET_X * thrust_vector.y();

    converted_control.force.x = thrust_vector.x();
    converted_control.force.y = thrust_vector.y();
    converted_control.force.z = thrust_vector.z();
    rocket_control_pub.publish(converted_control);

    if (first_command) {
        first_command = false;
        std_msgs::String msg;
        msg.data = "Launch";
        command_pub.publish(msg);
    }
}

//double t_start = 0;
void publishConvertedState(const real_time_simulator::State::ConstPtr &rocket_state) {

//    //simulator uses angular vel in inertial frame while mpc uses body frame
//    Eigen::Quaterniond attitude(rocket_state->pose.orientation.w, rocket_state->pose.orientation.x,
//                               rocket_state->pose.orientation.y, rocket_state->pose.orientation.z);
//    Eigen::Vector3d omega_inertial(rocket_state->twist.angular.x, rocket_state->twist.angular.y, rocket_state->twist.angular.z);
//    Eigen::Vector3d omega_body = attitude.inverse()._transformVector(omega_inertial);

//    converted_state.twist.angular.x = omega_inertial(0);
//    converted_state.twist.angular.y = omega_inertial(1);
//    converted_state.twist.angular.z = omega_inertial(2);
//    if(t_start == 0){
//        t_start = ros::Time::now().toSec();
//    }
//    double t = ros::Time::now().toSec() - t_start;
//    if(t < 1){
//        drone_gnc::DroneState state_msg;
//        state_msg.pose.position.x = sin(10*t);
//        state_msg.twist.linear.x = 10*cos(10*t);
//        state_msg.pose.orientation.w = 1;
//        state_msg.header.stamp = ros::Time(t);
//        drone_state_pub.publish(state_msg);
//
//        geometry_msgs::PoseStamped pose_msg;
//        pose_msg.pose.position.x = -sin(10*t);
//        pose_msg.pose.orientation.w = 1;
//        pose_msg.header.stamp = ros::Time(t);
//        fake_optitrack_pub.publish(pose_msg);
//        ROS_ERROR_STREAM("t: " << t);
//    }

    ros::Time now = ros::Time::now();
    drone_gnc::DroneState converted_state;
    converted_state.twist = rocket_state->twist;
    converted_state.pose = rocket_state->pose;
    converted_state.thrust_scaling = 1;
    converted_state.torque_scaling = 1;
    converted_state.header.stamp = now;

    drone_state_pub.publish(converted_state);

    geometry_msgs::PoseStamped optitrack_pose;

    optitrack_pose.pose.position.x = -rocket_state->pose.position.x;
    optitrack_pose.pose.position.y = -rocket_state->pose.position.y;
    optitrack_pose.pose.position.z = rocket_state->pose.position.z;
    optitrack_pose.pose.orientation = rocket_state->pose.orientation;
    optitrack_pose.header.stamp = now;
    fake_optitrack_pub.publish(optitrack_pose);
}

int main(int argc, char **argv) {
    //init state system (discrete ss model for ts = 0.03s)
    x_servo1 << 0, 0;
    x_servo2 << 0, 0;
    sysA << 1.359, -0.5178,
            1, 0;
    sysB << 0.5,
            0;
    sysC << 0.1758, 0.141;

    // Init ROS time keeper node
    ros::init(argc, argv, "drone_interface");
    ros::NodeHandle nh("control_interface");

    Drone drone_obj(nh);
    drone = &drone_obj;

    nh.param<double>("/rocket/estimated/thrust_scaling", thrust_scaling, 1);
    nh.param<double>("/rocket/estimated/torque_scaling", torque_scaling, 1);
    nh.param<double>("/rocket/estimated/servo1_offset", servo1_offset, 0);
    nh.param<double>("/rocket/estimated/servo2_offset", servo2_offset, 0);

    //TODO check if exists
    nh.getParam("/rocket/CM_to_thrust_distance", CM_to_thrust_distance);
    nh.getParam("/rocket/CM_offset_x", CM_OFFSET_X);

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = nh.subscribe("/drone_control", 10, publishConvertedControl);

    // Create control publisher
    rocket_control_pub = nh.advertise<real_time_simulator::Control>("/control_measured", 10);

    fake_optitrack_pub = nh.advertise<geometry_msgs::PoseStamped>("/optitrack_client/Drone/optitrack_pose", 10);


    // Subscribe to rocket state
    ros::Subscriber rocket_state_sub = nh.subscribe("/rocket_state", 10, publishConvertedState);

    // Create drone state publisher
    drone_state_pub = nh.advertise<drone_gnc::DroneState>("/simu_drone_state", 10);

    command_pub = nh.advertise<std_msgs::String>("/commands", 10);
    // Automatic callback of service and publisher from here
    ros::spin();
}
