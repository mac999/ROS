// package: reschy
// module: root_sensor_RGBD
// revision history
// date       | author      | description
// 2015.08.25 | T.W. Kang   | segmentation & AABB calculation
//
//
//
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
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
// #include <pcl/features/moment_of_inertia_estimation.h>

ros::Publisher pub, pub_obj;

typedef pcl::PointXYZ PointT;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);

	// Data containers used
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud.makeShared());
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

	// pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor; 

	pcl::PointCloud<pcl::PointXYZI> TotalCloud, ObjectCloud;  
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr segment(new pcl::PointCloud<pcl::PointXYZI> ());

		pcl::PointXYZI min_point_AABB, max_point_AABB, mass_center;
		// Eigen::Vector3f mass_center;
		min_point_AABB.x = min_point_AABB.y = min_point_AABB.z = 10000000000.0;
		max_point_AABB.x = max_point_AABB.y = max_point_AABB.z = -10000000000.0;

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			pcl::PointXYZ pt = cloud_filtered->points[*pit];
			pcl::PointXYZI pt2;
			pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
			pt2.intensity = (float)(j + 1);

			TotalCloud.push_back(pt2);
			segment->push_back(pt2);

			min_point_AABB.x = std::min(min_point_AABB.x, pt.x);
			min_point_AABB.y = std::min(min_point_AABB.y, pt.y);
			min_point_AABB.z = std::min(min_point_AABB.z, pt.z);
			min_point_AABB.intensity = (float)(j + 1);

			max_point_AABB.x = std::max(max_point_AABB.x, pt.x);
			max_point_AABB.y = std::max(max_point_AABB.y, pt.y);
			max_point_AABB.z = std::max(max_point_AABB.z, pt.z);
			max_point_AABB.intensity = (float)(j + 1);			
		}

		/* feature_extractor.setInputCloud (segment);
		feature_extractor.compute ();

		feature_extractor.getAABB (min_point_AABB, max_point_AABB);  
		feature_extractor.getMassCenter (mass_center);   */

		mass_center.x = (max_point_AABB.x - min_point_AABB.x) / 2.0 + min_point_AABB.x;
		mass_center.y = (max_point_AABB.y - min_point_AABB.y) / 2.0 + min_point_AABB.y;
		mass_center.z = (max_point_AABB.z - min_point_AABB.z) / 2.0 + min_point_AABB.z;
		mass_center.intensity = (float)(j + 1);				

		ObjectCloud.push_back(min_point_AABB);
		ObjectCloud.push_back(max_point_AABB);
		ObjectCloud.push_back(mass_center);

		j++;
	}

	// Convert To ROS data type  
	pcl::PCLPointCloud2 cloud_p;
	pcl::toPCLPointCloud2(TotalCloud, cloud_p); 

	sensor_msgs::PointCloud2 output;  
	pcl_conversions::fromPCL(cloud_p, output);
	output.header.frame_id = "camera_depth_optical_frame";    
	pub.publish(output);  

	// Convert To ROS data type  
	pcl::PCLPointCloud2 cloud_p2;
	pcl::toPCLPointCloud2(ObjectCloud, cloud_p2); 

	sensor_msgs::PointCloud2 output2;  
	pcl_conversions::fromPCL(cloud_p2, output2);
	output2.header.frame_id = "camera_depth_optical_frame";    
	pub_obj.publish(output2);    

	ROS_INFO("published it.");  
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "robot_sensor_RGBD");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

	// Create a ROS publisher for the segments
	pub = nh.advertise<sensor_msgs::PointCloud2> ("reschy/sensor/segments", 1);

	// Create a ROS publisher for the objects
	pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("reschy/sensor/AABB", 1);  

	// Spin
	ros::spin ();
}
