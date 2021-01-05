#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ros::NodeHandle nh;
// std::string topic = nh.resolveName("point_cloud");
// uint32_t queue_size = 1;
// //void callback(const sensor_msgs::PointCloud2ConstPtr&);
// void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
//
//     pcl::PCLPointCloud2 pcl_pc2;
//     pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
//     //do stuff with temp_cloud here
//     }
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "pcl_node");
// 	// to create a subscriber, you can do this (as above):
// 	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, callback);
//   ros::spin();
// }

#include <iostream>
#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);

  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}
