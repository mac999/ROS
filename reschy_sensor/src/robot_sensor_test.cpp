// package: reschy
// module: root_sensor_IMU
// revision history
// date       | author      | description
//
//
#include <ros/ros.h>
// PCL specific includes
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
// #include <pcl/features/moment_of_inertia_estimation.h>
// boost includes
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/format.hpp>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
// sensor / actuator includes


// control
typedef enum 
{
	ActionStepStop = 0, 
	ActionStepFinish = 1, 
	ActionStepRun = 2, 
	ActionStepClimb = 3,
} RobotActionStep;
RobotActionStep _stepStatus = ActionStepStop;

// global variables

// publishder
ros::Publisher pub;

// subscriber
void 
subscribeActive(const std_msgs::StringPtr& state)
{
	if(state->data.size() == 0)
		return;
	if(state->data == "on")
	{
		_stepStatus = ActionStepRun;
	}
	else if(state->data == "off")
	{
		_stepStatus = ActionStepStop;
	}
}

void 
subscribeControl(const std_msgs::StringPtr& msg)
{
	if(msg->data.size() == 0)
		return;
	// left=speed, right=speed
	// ex) left=100, right=90
	std::string data = msg->data;
	// std::vector<std::string> list = parsing tokens by using separators such as '=,'
	// float leftSpeed = list[1];
	// float rightSpeed = list[3];
	// DXL.speed(#1, leftSpeed * factor);
}

class robot_sensor_test
{
public:
	void callbackTimer(const ros::TimerEvent& event)
	{
		ROS_INFO("timer");  
		// polling data from sensor
		// publish the data as topic

		float rotary = 0.0;
		// rotary = DXL.rotary(#1);
		std_msgs::String msg;
		msg.data = ""; // rotary;
		pub.publish(msg);  
	}
};

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "robot_sensor_test");
	ros::NodeHandle nh;

	robot_sensor_test test;

	// subscriber
	ros::Subscriber subActive = nh.subscribe ("reschy/sensor/active/test", 10, subscribeActive);
	ros::Subscriber subControl = nh.subscribe ("reschy/control/body", 10, subscribeControl);	// TBD
	
	// timer with 0.05 second (20 Hz) 
	ros::Timer timerRun = nh.createTimer(ros::Duration(0.05), &robot_sensor_test::callbackTimer, &test);

	// Create a ROS publisher
	pub = nh.advertise<std_msgs::String> ("reschy/control/control/state", 10);		// TBD

	// Spin
	ros::spin();
	/* while(ros::ok())
	{
		ros::spinOnce();
	} */
}

