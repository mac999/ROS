// package: reschy
// module: robot_master_controller
// revision history
// date       | author      | description
// 2015.10.1  | T.W. Kang   | draft version. compile success.
// 2015.10.7  | T.W. Kang   | revision. JointState -> String.
//
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
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
ros::Publisher pubActiveLadderMission;
ros::Publisher pubActiveDoorMission;
ros::Publisher pubActiveStairMission;

ros::Publisher pubRunLadderMission;
ros::Publisher pubRunDoorMission;
ros::Publisher pubRunStairMission;

// subscriber
void subscribeStatusLadder(const std_msgs::StringPtr& input)
{
	if(input->data.size() == 0)
		return;
	printf("subscribe=%s\n", input->data.c_str());  
}

// utility functions


// node functions
typedef enum 
{
	ManualMission = 0,
	AutoMissionLadder = 1, 
	AutoMissionDoor = 1,
	AutoMissionStair = 2
} MissionOperationType;
MissionOperationType _MissionOperationMode = ManualMission;

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

class robot_master_controller
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
		 
		while (ros::ok())
		{
			puts("Reschy robot autonomous Ladder(L), Door(D), Stair(S) and Exit(X) mission");
			unsigned char key;
			if(read(_keyDesc, &key, 1) < 0)
				continue;

			ROS_DEBUG("Key value: 0x%02X\n", key);
	  		
			switch(key)
			{
			case KEYCODE_L:	// ladder mission
			case KEYCODE_B_L:
			{
				std_msgs::String active;  
				active.data = "on";    
				pubActiveLadderMission.publish(active); 

				// rosrun openni2 openni2.launch. TBD(don't know operation frequency exactly.

				_MissionOperationMode = AutoMissionLadder;
				break;
			}
			case KEYCODE_D:	// door mission
			case KEYCODE_B_D:
			{
				std_msgs::String active;  
				active.data = "on";    
				pubActiveDoorMission.publish(active); 

				// rosrun openni2 openni2.launch. TBD(don't know operation frequency exactly.

				_MissionOperationMode = AutoMissionDoor;
				break;
			}
			case KEYCODE_S:	// stair mission
			case KEYCODE_B_S:	
			{
				std_msgs::String active;  
				active.data = "on";    
				pubActiveStairMission.publish(active); 

				// rosrun openni2 openni2.launch. TBD(don't know operation frequency exactly.

				_MissionOperationMode = AutoMissionStair;
				break;
			}
			case KEYCODE_X: // stop autonomous mode
			{
				std_msgs::String active;  
				active.data = "off";    
				pubActiveLadderMission.publish(active); 
				pubActiveDoorMission.publish(active); 
				pubActiveStairMission.publish(active); 

				_MissionOperationMode = ManualMission;
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
	std_msgs::String run;  
	run.data = "run";

	if(_MissionOperationMode == AutoMissionLadder)
		pubRunLadderMission.publish(run);
	else if(_MissionOperationMode == AutoMissionDoor)
		pubRunDoorMission.publish(run);
	else if(_MissionOperationMode == AutoMissionStair)
		pubRunStairMission.publish(run);
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
	ros::init (argc, argv, "robot_master_controller");
	ros::NodeHandle nh;

	robot_master_controller robot_master;
	signal(SIGINT, quit);
	boost::thread keyboard_thread(boost::bind(&robot_master_controller::keyLoop, &robot_master));

	// subscriber
	ros::Subscriber subStatusLadder = nh.subscribe ("reschy/status/run/ladder", 1, subscribeStatusLadder);	// ex) ladder=finish, ladder=run, ladder=fail

	// publishder
  	pubActiveLadderMission = nh.advertise<std_msgs::String> ("reschy/control/active/ladder", 1);	
  	pubActiveDoorMission = nh.advertise<std_msgs::String> ("reschy/control/active/door", 1);	
  	pubActiveStairMission = nh.advertise<std_msgs::String> ("reschy/control/active/stair", 1);	

  	pubRunLadderMission = nh.advertise<std_msgs::String> ("reschy/control/run/ladder", 1);	
  	pubRunDoorMission = nh.advertise<std_msgs::String> ("reschy/control/run/door", 1);	
  	pubRunStairMission = nh.advertise<std_msgs::String> ("reschy/control/run/stair", 1);	

	// timer with 0.2 second (5 Hz) for running each steps of mission considering sensor data sync 
	ros::Timer timerRun = nh.createTimer(ros::Duration(0.2), callbackTimerRun);

	// Spin
	ros::spin();	// Call event functions (subscribers)

	keyboard_thread.interrupt();
	keyboard_thread.join();
}
