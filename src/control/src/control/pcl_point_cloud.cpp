#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

uint32_t queue_size = 1;

class pointCloudSegmenter{
	public:  
		bool execute;
		pointCloudSegmenter();
		void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
};
pointCloudSegmenter::pointCloudSegmenter () {
	execute = true;
}
pointCloudSegmenter pcs = pointCloudSegmenter();

// std::string topic = nh.resolveName("point_cloud");
// //void callback(const sensor_msgs::PointCloud2ConstPtr&);
void pointCloudSegmenter::callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
	if (!execute) {
		return;
	}
	ROS_INFO( "Converting pointcloud2 to pcl_poinctcloud xyz");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    execute = false;
	ROS_INFO("Executed callback, will not execute callback again");
	// do stuff with temp_cloud here
}
int main(int argc, char** argv) {
   	ros::init(argc, argv, "pcl_node");
	ros::NodeHandle nh;
	ROS_INFO("Node initialize");
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (
		"/camera/depth/points", queue_size, 
		&pointCloudSegmenter::callback, &pcs);
   	ROS_INFO("Node Subscribed");
	ros::spin();
}