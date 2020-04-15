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
#include <string.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <termios.h>
#include <sys/wait.h>
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

void executeProgram(const char* program, const char* command)
{
	pid_t mpid,pid;     // this holds the pid of the newly forked processes
	int status;

	mpid = fork();
	if (mpid == 0)          
	{
		pid = fork();
		if (pid == 0)
		{
			// this is the child
			execlp(program, program, command, 0);
			_exit(0);
		} 
		else 
		{
			// this is the parent
			_exit(0);
		}
	} 
	else 
	{ 
		// parent of first fork
		// waitpid(mpid,&status,WNOHANG);

		waitpid(mpid, &status, 0);
	}
}

void 
subscribeControl(const std_msgs::StringPtr& msg)
{
	if(msg->data.size() == 0)
		return;
	// ex) #1,2,3,4,5 S500 T1000 #6,7,8,9 P45 S500 
	std::string data = msg->data;

	// exec
	executeProgram("./dx.a", (const char*)data.c_str());
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
	ros::Subscriber subControl = nh.subscribe ("reschy/control/motor", 10, subscribeControl);	// TBD
	
	// timer with 0.05 second (20 Hz) 
	ros::Timer timerRun = nh.createTimer(ros::Duration(0.05), &robot_sensor_test::callbackTimer, &test);

	// Create a ROS publisher
	pub = nh.advertise<std_msgs::String> ("reschy/control/state", 10);		// TBD

	// Spin
	ros::spin();
	/* while(ros::ok())
	{
		ros::spinOnce();
	} */
}