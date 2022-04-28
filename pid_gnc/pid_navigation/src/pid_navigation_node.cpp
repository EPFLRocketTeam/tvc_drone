/*
* Node to estimate the rocket full state (position, velocity, attitude quaternion, angular rate and mass)
* from the sensor data and commanded thrust and torque of the rocket engine
*
* Inputs:
*   - Finite state machine from the pid_fsm :	     /gnc_fsm_pub
*   - State of the gimbal (position and thrust):	 	 /gimbal_state_0
*   - Sensor data (IMU and barometer) from av_interface: /sensor_pub
*
* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
#	- Kalman matrix: 		class NavigationNode
*
* Outputs:
*   - Complete estimated state : /kalman_rocket_state
*
*/

#include "ros/ros.h"

#include "pid_navigation.hpp"

#include <memory>
#include "rocket_types/ros_conversions.h"
#include "rocket_utils/Sensor.h"
#include "rocket_utils/State.h"
#include "ros_config_loader.h"

class RocketNavigationNode {
public:
    double frequency = 200;

    RocketNavigationNode() :
            rocket_fsm(RocketFSMState::IDLE) {
        ros::NodeHandle nh("~");

        PidNavigation::Config nav_config;

        //Get initial orientation and convert in Radians
        double roll_deg = 0, zenith_deg = 0, azimuth_deg = 0.0;
        nh.getParam("/environment/rocket_roll", roll_deg);
        nh.getParam("/environment/rail_zenith", zenith_deg);
        nh.getParam("/environment/rail_azimuth", azimuth_deg);

        nav_config.initial_roll = roll_deg * M_PI / 180;
        nav_config.rail_zenith = zenith_deg * M_PI / 180;
        nav_config.rail_azimuth = azimuth_deg * M_PI / 180;

        RocketProps rocket_props = loadRocketProps(nh);

        navigation = std::unique_ptr<PidNavigation>(new PidNavigation(nav_config, rocket_props));

        initTopics(nh);
    }

    void initTopics(ros::NodeHandle &nh) {
        // Create filtered rocket state publisher
        nav_pub = nh.advertise<rocket_utils::State>("/kalman_rocket_state", 10);

        // Subscribe to time_keeper for fsm and time
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &RocketNavigationNode::fsmCallback, this);

        // Subscribe to control for kalman estimator
        control_sub = nh.subscribe("/gimbal_state_0", 1, &RocketNavigationNode::controlCallback, this);

        // Subscribe to sensor for kalman correction
        sensor_sub = nh.subscribe("/sensor_pub", 1, &RocketNavigationNode::sensorCallback, this);
    }

    void run() {
        double time_now = ros::Time::now().toSec();
        double dT = time_now - last_predict_time;
        last_predict_time = time_now;

        switch (rocket_fsm) {
            case IDLE: {
                // Do nothing
                break;
            }

            case RAIL:
            case LAUNCH: {
                navigation->predict(dT, imu_acc, imu_gyro, gimbal_control);
                // Perform an EKF update if a new barometer measurement was received
                if (baro_flag) {
                    navigation->barometerUpdate(baro_height);
                    baro_flag = false;
                }
                break;
            }

            case COAST:
            case STOP: {
                // Do nothing
                break;
            }
        }

        // Convert the navigation state to ROS message and publish it
        rocket_utils::State state_msg = toROS(navigation->getState());
        state_msg.header.stamp = ros::Time(time_now);
        nav_pub.publish(state_msg);
    }

private:
    std::unique_ptr<PidNavigation> navigation;

    double last_predict_time;

    RocketGimbalControl gimbal_control;
    Vector3 imu_acc;
    Vector3 imu_gyro;
    double baro_height;
    bool baro_flag = false;

    // Last requested fsm
    RocketFSMState rocket_fsm;

    // List of subscribers and publishers
    ros::Publisher nav_pub;
    ros::Subscriber fsm_sub;
    ros::Subscriber control_sub;
    ros::Subscriber sensor_sub;

    /* ------------ Callbacks functions ------------ */

    // Callback function to store last received fsm
    void fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
        rocket_fsm = fromROS(*fsm);
    }

    // Callback function to store last received control
    void controlCallback(const rocket_utils::GimbalControl::ConstPtr &gimbal_control_msg) {
        gimbal_control = fromROS(*gimbal_control_msg);
    }

    // Callback function to store last received sensor data
    // TODO one callback for each sensor
    void sensorCallback(const rocket_utils::Sensor::ConstPtr &sensor) {
        imu_acc = fromROS(sensor->IMU_acc);
        imu_gyro = fromROS(sensor->IMU_gyro);
        baro_height = sensor->baro_height;
        baro_flag = true;
    }
};

int main(int argc, char **argv) {
    // Init ROS navigation node
    ros::init(argc, argv, "data_fusion");

    RocketNavigationNode navigation_node;

    ros::Rate loop_rate(navigation_node.frequency);

    while (ros::ok()) {
        ros::spinOnce();
        navigation_node.run();

        loop_rate.sleep();
    }
}