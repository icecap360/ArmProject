#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

uint32_t queue_size = 1;

class pointCloudSegmenter{
	public:  
		int a;
		pointCloudSegmenter(int);
		void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
};
pointCloudSegmenter::pointCloudSegmenter (int aq) {
	a = aq;
	std::cout << a;
}
pointCloudSegmenter pcs = pointCloudSegmenter(5);

// std::string topic = nh.resolveName("point_cloud");
// //void callback(const sensor_msgs::PointCloud2ConstPtr&);
void pointCloudSegmenter::callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
	ROS_INFO( "%d", pcs.a);
    pcl::PCLPointCloud2 pcl_pc2;
//     pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
//     //do stuff with temp_cloud here
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
