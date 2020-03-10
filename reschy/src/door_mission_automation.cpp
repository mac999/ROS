// package: reschy
// module: root_sensor_RGBD
// revision history
// date       | author      | description
// 2015.10.1  | T.W. Kang   | draft version for autonomous operation related to door mission. 
// 2015.10.1  | T.W. Kang   | compile success.
// 2015.10.5  | T.W. Kang   | added PID.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
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
// STL
#include <string>
#include <deque>
#include <vector>
// PID
#include "robot_pid.h"
// Utility
#include "robot_util.h"

// control
typedef enum 
{
	ActionStepStop = 0, 
	ActionStepFinish = 1, 
	ActionStepRun = 2, 
	ActionStepClimb = 3,
} RobotActionStep;
RobotActionStep _stepStatus = ActionStepStop;

// publisher
ros::Publisher pubStatus;
ros::Publisher pubOpencvControl;

// subscriber
bool _subSegments = false;	// subscribed data sync flag
bool _subAABB = false;
bool _subOpenCV = false;

pcl::PointCloud<pcl::PointXYZI> _CurrentPointCloudSegments;
pcl::PointCloud<pcl::PointXYZI> _CurrentSegmentsAABB;	// Axis-aligned bounding box about each segments

void subscribeSegments(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(_stepStatus <= ActionStepFinish)
		return;
	_CurrentPointCloudSegments.clear();
	pcl::fromROSMsg(*input, _CurrentPointCloudSegments);
	_subSegments = true;
}

void subscribeAABB(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(_stepStatus <= ActionStepFinish)
		return;
	_CurrentSegmentsAABB.clear();
	pcl::fromROSMsg(*input, _CurrentSegmentsAABB);
	_subAABB = true;
}

float _current_imu_roll = 0.0;
float _current_imu_pitch = 0.0;
float _current_imu_yaw = 0.0;
void subscribeIMU(const std_msgs::StringPtr& input)
{
	parsingIMU(input->data, _current_imu_roll, _current_imu_pitch, _current_imu_yaw);
}

std::string _current_opencv_info_string = "";
struct OpencvInfo
{
	float area, posX, posY;
} _current_opencv_info;
void subscribeOpenCV(const std_msgs::StringPtr& input)
{
	_current_opencv_info_string = input->data;
	// TBD: refer to parsingIMU() function for implementing parsing function. 
	// parsingOpencvInfo(_current_opencv_info, _current_opencv_info);
}


// utility functions
int getSegmentsFromReschyPointCloud(pcl::PointCloud<pcl::PointXYZI>& pcd, std::vector<pcl::PointIndices>& segments)
{
	// pcd parameter is 3D point cloud sorted by intensity value which has segment index from 0 to N.
	if(pcd.size() == 0)
		return 0;

	int segmentIndex = -1;
	pcl::PointIndices segment;
	for(int i = 0; i < pcd.size(); i++)
	{
		pcl::PointXYZI pt = pcd[i];
		int index = (int)pt.intensity;
		if(segmentIndex < 0)
			segmentIndex = index;

		if(segmentIndex != index)
		{			
			segments.push_back(segment);
			segment.indices.clear();
			segmentIndex = index;			
		}
		segment.indices.push_back(i);
	}
	if(segment.indices.size())
		segments.push_back(segment);
	
	return segments.size();
}

int getPointCloudFromSegmentIndex(pcl::PointCloud<pcl::PointXYZI>& pcd, const std::vector<int>& segmentIndex, pcl::PointCloud<pcl::PointXYZI>& segment)
{
	for(int i = 0; i < segmentIndex.size(); i++)
	{
		int index = segmentIndex[i];
		pcl::PointXYZI pt;
		if(index >= pcd.size() || index < 0)
			continue;
		pt = pcd[index];
		segment.push_back(pt);
	}
	return segment.size();
}

struct Rect3D
{
	pcl::PointXYZ pt1, pt2;

	Rect3D()
	{
		clear();
	}

	void clear()
	{
		pt1.x = pt1.y = pt1.z = 9999999999.9;
		pt2.x = pt2.y = pt2.z = -9999999999.9;
	}
	
	void grow(pcl::PointXYZI& pt)
	{
		if(pt.x < pt1.x)
			pt1.x = pt.x;
		if(pt.y < pt1.y)
			pt1.y = pt.y;
		if(pt.z < pt1.z)
			pt1.z = pt.z;
		if(pt.x > pt2.x)
			pt2.x = pt.x;
		if(pt.y > pt2.y)
			pt2.y = pt.y;
		if(pt.z > pt2.z)
			pt2.z = pt.z;
	}

	bool getMaximum(pcl::PointCloud<pcl::PointXYZI>& pcd)
	{
		Rect3D rect;
		for(int i = 0; i < pcd.size(); i++)
		{
			pcl::PointXYZI pt = pcd[i];
		
			rect.grow(pt);
		}

		*this = rect;
		return pcd.size() > 0;
	}

	float width()
	{
		return fabs(pt2.x - pt1.x);
	}

	float height()
	{
		return fabs(pt2.y - pt1.y);
	}

	float depth()
	{
		return fabs(pt2.z - pt1.z);
	}
};

#define IsEqual(a, b, t) (fabs(a - b) <= t)

// node functions
void moveRobotBody(float leftSpeed, float leftMilliSecond, float rightSpeed, float rightMilliSecond)
{	if(_stepStatus <= ActionStepFinish)
		return;
	// publishMessage(...); // considering left and right side wheel.
}

void moveRobotBody(float leftDistance, float rightDistance)
{
	const float wheelRadius = 0.10;

	float leftSpeed = leftDistance / wheelRadius;
	float rightSpeed = rightDistance / wheelRadius;
	moveRobotBody(leftSpeed, 1000.0, rightSpeed, 1000.0);
}

int canSeeDoor(float nearDoorDistance, float farDoorDistance, float width = 0.3,  float height = 0.5, float depth = 0.5, float tolerance = 0.1)
{
	// TBD
	std::vector<pcl::PointIndices> clusterSegmentsIndices;
	getSegmentsFromReschyPointCloud(_CurrentPointCloudSegments, clusterSegmentsIndices);

	std::vector<pcl::PointIndices> clusterAABBIndices;
	getSegmentsFromReschyPointCloud(_CurrentSegmentsAABB, clusterAABBIndices);
	
	int SegmentId = -1;
	for(std::vector<pcl::PointIndices>::const_iterator it = clusterAABBIndices.begin(); it != clusterAABBIndices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI> pointsAABB;
		getPointCloudFromSegmentIndex(_CurrentSegmentsAABB, it->indices, pointsAABB);

		Rect3D MaxRect;	
		if(MaxRect.getMaximum(pointsAABB) == false)
			continue;

		if(MaxRect.pt2.z < nearDoorDistance - tolerance)
			continue;
		if(MaxRect.pt1.z > farDoorDistance + tolerance)
			continue;

		float segmentWidth = MaxRect.width();
		float segmentHeight = MaxRect.height();
		float segmentDepth = MaxRect.depth();
		if(IsEqual(segmentHeight, height, tolerance) && 
			IsEqual(segmentWidth, width, tolerance) && 
			IsEqual(segmentDepth, depth, tolerance))
		{ 
			SegmentId = pointsAABB[0].intensity;
			return SegmentId;
		}
	}
	return SegmentId;
}

bool canSeeFrontDoor(float width, float height, float tolerance = 0.1)
{
	// TBD
	// About each segments, detect door which has features such as door volume, distance etc.
	std::vector<pcl::PointIndices> clusterSegmentsIndices;
	getSegmentsFromReschyPointCloud(_CurrentPointCloudSegments, clusterSegmentsIndices);

	std::vector<pcl::PointIndices> clusterAABBIndices;
	getSegmentsFromReschyPointCloud(_CurrentSegmentsAABB, clusterAABBIndices);
	
	int SegmentId = -1;
	for(std::vector<pcl::PointIndices>::const_iterator it = clusterAABBIndices.begin(); it != clusterAABBIndices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI> pointsAABB;
		getPointCloudFromSegmentIndex(_CurrentSegmentsAABB, it->indices, pointsAABB);

		Rect3D MaxRect;	
		if(MaxRect.getMaximum(pointsAABB) == false)
			continue;

		float segmentWidth = MaxRect.width();
		float segmentHeight = MaxRect.height();
		float segmentDepth = MaxRect.depth();
		if(IsEqual(segmentHeight, height, tolerance) && 
			IsEqual(segmentWidth, width, tolerance))
		{ 
			SegmentId = pointsAABB[0].intensity;
			return SegmentId;
		}
	}
	return SegmentId;	
}

bool canSeeDoorTop()
{
	bool ret = canSeeFrontDoor(0.6, 1.0);
	return ret;
}

float _target_imu_yaw = 0.0;	// Yaw value

void 
subscribeActive(const std_msgs::StringPtr& state)
{
	if(state->data.size() == 0)
		return;
	if(state->data == "on")
	{
		_stepStatus = ActionStepRun;	
		initPID();
		_target_imu_yaw = _current_imu_yaw;	// I assume that current imu direction is target orientation.
		
		std_msgs::String control;  
		control.data = "on=auburn";
		pubOpencvControl.publish(control); 		
	}
	else if(state->data == "off")
		_stepStatus = ActionStepStop;
}

double _previousTime = 0.0;
void pidControl()	// PID control by using IMU yaw
{
	pidControl();
	float currentValue = _current_imu_yaw;

	double currentTime = ros::Time::now().toSec();
	if(_previousTime == 0.0)	// TBD
		_previousTime = currentTime;
	float samplingTime = _previousTime;
	float controlValue = calcPID(_target_imu_yaw, currentValue, samplingTime);	// To decide the left and right speed		
	float balance = convertValue(controlValue, 1.0, -1.0, 1.0);
	printf("IMU yaw balance = %f\n", balance);
	if(fabs(balance) > 0.01)
	{
		float speed = convertValue(controlValue, 1.0, 0.0, 1.0);
		if(balance < 0)	// rotate as left
			moveRobotBody(0.4 * (1.0 - speed), 0.4 * speed);
		else
			moveRobotBody(0.4 * speed, 0.4 * (1.0 - speed));
	}
	else
		moveRobotBody(0.4, 0.4);

	_previousTime = currentTime;
}

void 
subscribeRun(const std_msgs::StringPtr& state)
{
	if(_stepStatus <= ActionStepFinish)
		return;
	if(_subSegments == false || _subAABB == false)
		return;
	_subSegments = false;	// subscribed data sync flag
	_subAABB = false;

	if(_stepStatus == ActionStepRun)
	{
		const float nearDoorDistance = 0.5;
		const float farDoorDistance = 1.5;
		int SegmentId = canSeeDoor(nearDoorDistance, farDoorDistance);
		if(SegmentId < 0)	// didn't find Door segment under condition.
		{
			moveRobotBody(0.2, 0.2);
			return;
		}
		else		// found it
			_stepStatus = ActionStepClimb;
	}
	else if(_stepStatus == ActionStepClimb)
	{
		bool canSee = canSeeDoorTop();
		if(canSee == false)
		{
			moveRobotBody(0.2, 0.2);
			return;
		}

		const bool activeIMU = false;
		if(activeIMU)	// PID control
			pidControl();
		else
			moveRobotBody(0.4, 0.4);

		std_msgs::String state;  
		state.data = "finish";
		pubStatus.publish(state); 
	}
}

int
main (int argc, char** argv)
{
	// Initialize ROSreschy
	ros::init (argc, argv, "door_mission_automation");
	ros::NodeHandle nh;

	// subscriber
	ros::Subscriber subActive = nh.subscribe ("reschy/control/active/door", 1, subscribeActive);
	ros::Subscriber subSegments = nh.subscribe ("reschy/sensor/segments", 1, subscribeSegments);
	ros::Subscriber subAABB = nh.subscribe ("reschy/sensor/AABB", 1, subscribeAABB);
	ros::Subscriber subIMU = nh.subscribe ("reschy/sensor/imu", 1, subscribeIMU);
	ros::Subscriber subOpenCV = nh.subscribe ("reschy/sensor/vision/image_info", 1, subscribeOpenCV);
	
	// By using timer, robot_master_controller node should send control/run message for autonomous operation
	ros::Subscriber subRun = nh.subscribe ("reschy/control/run/door", 1, subscribeRun);	

	// publisher
	pubStatus = nh.advertise<std_msgs::String> ("reschy/status/run/door", 1);	// ex) door=finish, door=run, door=fail
	pubOpencvControl = nh.advertise<std_msgs::String> ("reschy/control/active/vision", 1);	// ex) on=red, on=blue, on=green, on=auburn, on=white, off
	
	// Spin
	ros::spin();	// Call event functions (subscribers)
}
