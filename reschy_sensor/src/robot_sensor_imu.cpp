// package: reschy
// module: root_sensor_IMU
// revision history
// date       | author      | description
// 2015.10.03 | T.W. Kang   | IMU & kalman
// 2015.10.05 | T.W. Kang   | changed from sensor_msgs::PointCloud2 to std_msgs::String. 
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
// RTIMULib includes
#include "RTIMULib.h"

// control
typedef enum 
{
	ActionStepStop = 0, 
	ActionStepFinish = 1, 
	ActionStepRun = 2, 
	ActionStepClimb = 3,
} RobotActionStep;
RobotActionStep _stepStatus = ActionStepStop;

// sensor
RTIMU* _imu = NULL;
int _sampleCount = 0;
uint64_t _now;
uint64_t _displayTimer;
uint64_t _rateTimer;
int _sampleRate = 0;

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

		//  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
		//  Or, you can create the .ini in some other directory by using:
		//      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
		//  where <directory path> is the path to where the .ini file is to be loaded/saved

		RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

		_imu = RTIMU::createIMU(settings);

		if ((_imu == NULL) || (_imu->IMUType() == RTIMU_TYPE_NULL)) {
		    printf("No IMU found\n");
		    exit(1);
		}
		printf("active IMU sensor\n");

		//  This is an opportunity to manually override any settings before the call IMUInit

		//  set up IMU
		_imu->IMUInit();
		printf("init IMU\n");

		//  this is a convenient place to change fusion parameters

		_imu->setSlerpPower(0.02);
		_imu->setGyroEnable(true);
		_imu->setAccelEnable(true);
		_imu->setCompassEnable(true);

		//  set up for rate timer
		_rateTimer = _displayTimer = RTMath::currentUSecsSinceEpoch();
	}
	else if(state->data == "off")
	{
		_stepStatus = ActionStepStop;

		if(_imu)
		{
			delete _imu;
			_imu = NULL;
		}
		printf("inactive IMU sensor\n");
	}
}

class robot_sensor_imu
{
public:
	void callbackTimer(const ros::TimerEvent& event)
	{
		if(_imu == NULL)
			return;

		// poll at the rate recommended by the IMU
		usleep(_imu->IMUGetPollInterval() * 1000);

		std::string data = "";
		while (_imu->IMURead()) 
		{
			RTIMU_DATA imuData = _imu->getIMUData();
			_sampleCount++;

			_now = RTMath::currentUSecsSinceEpoch();

			//  display 10 times per second
			if ((_now - _displayTimer) > 100000) {
				// data = RTMath::displayDegrees("", imuData.fusionPose);
				// printf("Sample rate %d: %s\r", _sampleRate, data.c_str());
				// measuredPose = m_imuThread->getIMU()->getMeasuredPose();
				// fflush(stdout);

				RTFLOAT roll = imuData.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
				RTFLOAT pitch = imuData.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
				RTFLOAT yaw = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
				char szData[512];
				sprintf(szData, "%f, %f, %f", roll, pitch, yaw);
				data = szData;

				printf("Sample(roll, pitch, yaw) rate %d: %s\r", _sampleRate, data.c_str());
				fflush(stdout);

				_displayTimer = _now;
			}

			//  update rate every second
			if ((_now - _rateTimer) > 1000000) {
				_sampleRate = _sampleCount;
				_sampleCount = 0;
				_rateTimer = _now;
			}
		}

		// Convert To ROS data type  
		std_msgs::String msg;
		msg.data = data;
		pub.publish(msg);  

		// ROS_INFO("published it.");  
	}
};

int
main (int argc, char** argv)
{
	// Initialize ROS
	/* std_msgs::StringPtr state(new std_msgs::String);
	state->data = "on";
	subscribeActive(state); */

	robot_sensor_imu sensor_imu;

	ros::init (argc, argv, "robot_sensor_IMU");
	ros::NodeHandle nh;

	printf("begin IMU ROS node\n");
	
	// subscriber
	ros::Subscriber subActive = nh.subscribe ("reschy/sensor/active/imu", 1, subscribeActive);
	
	// timer with 0.05 second (20 Hz) 
	ros::Timer timerRun = nh.createTimer(ros::Duration(0.05), &robot_sensor_imu::callbackTimer, &sensor_imu);

	// Create a ROS publisher
	pub = nh.advertise<std_msgs::String> ("reschy/sensor/imu", 1);

	// Spin
	ros::spin();
	/* while(ros::ok())
	{
		ros::spinOnce();
		ros::TimerEvent event;
		imu.callbackTimer(event);
	} */
}

