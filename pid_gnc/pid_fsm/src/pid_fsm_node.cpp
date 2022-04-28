/*
* Node to synchronize the GNC algorithms by estimating the current state of the rocket
* (Idle, Rail, Launch, Coast)
*
* Inputs: 
*   - Sensor data (IMU and barometer) from av_interface: /sensor_pub
*   - Estimated state from pid_navigation:		     /kalman_rocket_state
*	- Commanded gimbal state from pid_control		 /gimbal_command_0
*
* Important parameters:
*   - Threshold for rocket ignition detection (Rail phase): in FSM_thread
*
* Outputs:
*   - Estimated finite state machine from flight data:	/gnc_fsm_pub
*
*/

#include "ros/ros.h"
#include "rocket_utils/FSM.h"
#include "rocket_utils/State.h"
#include "rocket_utils/Sensor.h"
#include "rocket_utils/GimbalControl.h"

#include <time.h>

#include <sstream>
#include <string>

#include "std_msgs/String.h"

class FsmNode {
	private:

		// Current time and state machine
		rocket_utils::FSM rocket_fsm;
		ros::Time time_zero;

		// Last received sensor data
		rocket_utils::Sensor rocket_sensor;

		// Last received feedback control
		rocket_utils::GimbalControl gimbal_state;

		// Last received rocket state
		rocket_utils::State rocket_state;

		// List of subscribers and publishers
		ros::Publisher timer_pub;

		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;
		ros::Subscriber control_sub;
        ros::Subscriber command_sub;

		// Other parameters
		double rail_length = 0;

	public:
		FsmNode(ros::NodeHandle &nh)
		{
			// Initialize publishers and subscribers
        	initTopics(nh);

			// Initialize fsm
			rocket_fsm.state_machine = rocket_utils::FSM::IDLE;

			// Overwrite rocket mass to stay in launch mode at first iteration
			rocket_state.propeller_mass = 10;


			timer_pub.publish(rocket_fsm);

			nh.getParam("/environment/rail_length", rail_length);
		}

		void initTopics(ros::NodeHandle &nh)
		{
			// Create timer publisher and associated thread (100Hz)
			timer_pub = nh.advertise<rocket_utils::FSM>("gnc_fsm_pub", 10);

			// Subscribe to state message
			rocket_state_sub = nh.subscribe("kalman_rocket_state", 1, &FsmNode::rocket_stateCallback, this);

			// Subscribe to sensors message
			sensor_sub = nh.subscribe("sensor_pub", 1, &FsmNode::sensorCallback, this);

			// Subscribe to commanded control message
			control_sub = nh.subscribe("gimbal_command_0", 1, &FsmNode::controlCallback, this);

            // Subscribe to commands
            command_sub = nh.subscribe("/commands", 10, &FsmNode::processCommand, this);
		}

		void rocket_stateCallback(const rocket_utils::State::ConstPtr& new_rocket_state)
		{
			rocket_state.pose = new_rocket_state->pose;
			rocket_state.twist = new_rocket_state->twist;
			rocket_state.propeller_mass = new_rocket_state->propeller_mass;
		}

		// Callback function to store last received sensor data
		void sensorCallback(const rocket_utils::Sensor::ConstPtr& new_sensor)
		{
			rocket_sensor.IMU_acc = new_sensor->IMU_acc;
			rocket_sensor.IMU_gyro = new_sensor->IMU_gyro;
			rocket_sensor.baro_height = new_sensor->baro_height;
		}

		// Callback function to store last received commanded control
		void controlCallback(const rocket_utils::GimbalControl::ConstPtr& state)
		{
			gimbal_state.outer_angle = state->outer_angle;
			gimbal_state.inner_angle = state->inner_angle;
			gimbal_state.thrust = state->thrust;
		}

        void processCommand(const std_msgs::String &command) {
            if (command.data == "stop" || command.data == "Stop") {
                rocket_fsm.state_machine = rocket_utils::FSM::STOP;
            } else if (rocket_fsm.state_machine == rocket_utils::FSM::IDLE) {
				
                //received launch command
                time_zero = ros::Time::now();
                rocket_fsm.state_machine = rocket_utils::FSM::RAIL;
            }
        }

		void updateFSM()
		{
			if (rocket_fsm.state_machine.compare("Idle") == 0)
			{
                // Do nothing
			}

			else
			{
				// Update current time
				rocket_fsm.header.stamp = ros::Time::now();
                rocket_fsm.launch_time = time_zero;

				if (rocket_fsm.state_machine.compare("Rail") == 0)
				{
					// End of rail
					if(rocket_state.pose.position.z > rail_length)
					{
						rocket_fsm.state_machine = "Launch";
					}

				}

				else if (rocket_fsm.state_machine.compare("Launch") == 0)
				{
					// End of burn -> no more thrust
					if(rocket_state.propeller_mass < -1)
					{
						rocket_fsm.state_machine = "Coast";
					}

				}

				else if (rocket_fsm.state_machine.compare("Coast") == 0)
				{
					// Do nothing for now
				}

				// Publish time + state machine
				timer_pub.publish(rocket_fsm);
			}
		}


};







int main(int argc, char **argv)
{

	// Init ROS time keeper node
	ros::init(argc, argv, "gnc_fsm");
	ros::NodeHandle nh;

	FsmNode fsmNode(nh);

	// Thread to compute FSM. Duration defines interval time in seconds
	ros::Timer FSM_thread = nh.createTimer(ros::Duration(0.001),
	[&](const ros::TimerEvent&)
	{
		fsmNode.updateFSM();
	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
