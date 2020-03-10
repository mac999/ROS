// package: reschy
// module: robot_test_controller
// revision history
// date       | author      | description
//
//
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// boost includes
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

// publisher
ros::Publisher pubTest;

// subscriber
void subscribeStatus(const std_msgs::String& input)
{
	// ROS_INFO(input);  
}

// utility functions


// node functions
#define KEYCODE_D 0x64	// ASCII code. small letter
#define KEYCODE_L 0x6C
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72 
#define KEYCODE_S 0x73
#define KEYCODE_U 0x75
#define KEYCODE_X 0x78

#define KEYCODE_B_D 0x44	// ASCII code. big letter
#define KEYCODE_B_L 0x4C
#define KEYCODE_B_Q 0x51
#define KEYCODE_B_R 0x52 
#define KEYCODE_B_S 0x53
#define KEYCODE_B_U 0x55
#define KEYCODE_B_X 0x58

int _keyDesc = 0;
struct termios _keyCooked, _keyRaw;
int _key = 0;

class robot_test_controller
{
public:
	void keyLoop()
	{ 
		tcgetattr(_keyDesc, &_keyCooked);
		memcpy(&_keyRaw, &_keyCooked, sizeof(struct termios));
		_keyRaw.c_lflag &=~ (ICANON | ECHO);
		// Setting a new line, then end of file
		_keyRaw.c_cc[VEOL] = 1;
		_keyRaw.c_cc[VEOF] = 2;
		tcsetattr(_keyDesc, TCSANOW, &_keyRaw);
		 
		puts("Reading from keyboard");
		puts("---------------------------");
		puts("Use arrow keys to move the turtlebot.");
		 
		while (ros::ok())
		{
			unsigned char key;
			if(read(_keyDesc, &key, 1) < 0)
				continue;

			ROS_DEBUG("value: 0x%02X\n", key);
			_key = key;
	  		
			switch(key)
			{
			case KEYCODE_L:	// 
			case KEYCODE_B_L:
			{
				// std_msgs::String active;
				// active = std_msgs::Empty;
				// active.data = "toggle_led";
				std_msgs::Empty emp;
			
				pubTest.publish(emp); 

				break;
			}
			case KEYCODE_D:	// door mission
			case KEYCODE_B_D:
			{
				break;
			}
			case KEYCODE_S:	// stair mission
			case KEYCODE_B_S:	
			{
				break;
			}
			case KEYCODE_X: // stop autonomous mode
			{
				break;
			}
			default:
				break;
			}
		}
	}
};

// timer
void callbackTimerRun(const ros::TimerEvent& event)
{
	// std_msgs::String run;  

	// pubTest.publish(run);
}

// signal
void quit(int sig)
{
	tcsetattr(_keyDesc, TCSANOW, &_keyCooked);
	ros::shutdown();
	exit(0);
}

// main
int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "robot_test_controller");
	ros::NodeHandle nh;

	robot_test_controller robot_master;
	signal(SIGINT, quit);
	boost::thread keyboard_thread(boost::bind(&robot_test_controller::keyLoop, &robot_master));

	// subscriber
	ros::Subscriber subStatus = nh.subscribe ("reschy/test/status", 1, subscribeStatus);	// ex) ladder=finish, ladder=run, ladder=fail

	// publishder
  	pubTest = nh.advertise<std_msgs::Empty> ("toggle_led", 1); // reschy/test/control", 1);
	
	// timer with 0.2 second (5 Hz) for running each steps of mission considering sensor data sync 
	ros::Timer timerRun = nh.createTimer(ros::Duration(0.2), callbackTimerRun);

	// Spin
	ros::spin();	// Call event functions (subscribers)

	keyboard_thread.interrupt();
	keyboard_thread.join();
}

