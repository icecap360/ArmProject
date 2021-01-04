#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::NodeHandle nh;
std::string topic = nh.resolveName("point_cloud");
uint32_t queue_size = 1;
//void callback(const sensor_msgs::PointCloud2ConstPtr&);
void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_node");
	// to create a subscriber, you can do this (as above):
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, callback);
  ros::spin();
}