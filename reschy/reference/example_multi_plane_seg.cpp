#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  sensor_msgs::PointCloud2 output;
  for(int i = 0; i < 8; i++)
  {
  	if(cloud.size() < 10000)
 			break;
	  seg.setInputCloud (cloud.makeShared ());
	  
	  pcl::ModelCoefficients coefficients;
	  // pcl::PointIndices inliers;
	  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  
	  seg.segment (*inliers, coefficients); 
	  
	  // Create the filtering object
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	
	  // Extract the inliers
	  ROS_INFO("extract the inliers");  
	  pcl::PointCloud<pcl::PointXYZ> in_cloud;
	  extract.setInputCloud (cloud.makeShared ());
	  extract.setIndices (inliers);
	  extract.setNegative (false);
	  extract.filter (in_cloud);
	  extract.setNegative (true);
	  extract.filter (cloud);
	  
	  pcl::PointCloud<pcl::PointXYZ>::iterator index = in_cloud.begin();
	  for(; index != in_cloud.end(); index++)
	  {
		 pcl::PointXYZ pt = *index;
		 pcl::PointXYZI pt2;
		 pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
		 pt2.intensity = (float)(i + 1);

		 TotalCloud.push_back(pt2);
	  }
	  
	  ROS_INFO("%d. remained point cloud = %d", i, (int)cloud.size());  
  } 
   
	// Convert To ROS data type  
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);  
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "camera_depth_optical_frame";    
  pub.publish(output);  
  
  ROS_INFO("published it.");  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pclplaneoutput", 1);
  
  // Spin
  ros::spin ();
}
