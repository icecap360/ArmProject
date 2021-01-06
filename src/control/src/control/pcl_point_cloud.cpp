#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// hull
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/point_cloud.h>
#include <control/cluster_points.h>
uint32_t queue_size = 1;
class pointClouodSegmenter;

class pointCloudSegmenter{
	public:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
		bool execute;
		pointCloudSegmenter();
		void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
		pcl::PointCloud<pcl::PointXYZ>::Ptr concave_hull (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
		int segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};
// constructor
pointCloudSegmenter::pointCloudSegmenter () {
	execute = true;
  pub = nh.advertise<control::cluster_points>("cloud_hull", 10);
  sub = nh.subscribe<sensor_msgs::PointCloud2> (
		"/camera/depth/points", queue_size,
		&pointCloudSegmenter::callback, this);
   	ROS_INFO("Node Subscribed");
}

void pointCloudSegmenter::callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
	if (!execute) {
		return;
	}
	std::cout<< "Converting pointcloud2 to pcl_poinctcloud xyz";
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  execute = false;
  std::cout<<"temp_cloud has: "<< temp_cloud->size() <<" size ";

	int status = segment(temp_cloud);
	std::cout<<"Finished executed callback, will not execute callback again!";
	// do stuff with temp_cloud here
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudSegmenter::concave_hull (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);
	std::cerr << "PointCloud after segmentation has: "
						<< inliers->indices.size () << " inliers." << std::endl;

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	// proj.setIndices (inliers);
	proj.setInputCloud (cloud_filtered);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	std::cerr << "PointCloud after projection has: "
						<< cloud_projected->size () << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud (cloud_projected);
	// change alpha for edge precision
	// smaller for more precision
	chull.setAlpha (0.01);
	chull.reconstruct (*cloud_hull);

	return cloud_hull;
}

int pointCloudSegmenter::segment (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

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
	//std::cout << "\n" << cluster_indices.size() << '\n';

  int j = 0;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> cent_x;
  std::vector<float> cent_y;
  float end_cluster = std::numeric_limits<float>::infinity();

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
			// std::cout << *pit << '\n';
      cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull = concave_hull(cloud_cluster);

    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

		cloud_hull->width = cloud_hull->size ();
    cloud_hull->height = 1;
    cloud_hull->is_dense = true;

    Eigen::Vector4f centroid;
		//pcl::compute3DCentroid<pcl::PointXYZ, Eigen::Vector4f> centroid_compute;
    std::cout<<"points used "<<pcl::compute3DCentroid(*cloud_hull,centroid)<<'\n';
		//centroid_compute.compute3DCentroid(cloud_hull, centroid);
    std::cout<<"Centroid of this cluster: x "<<end_cluster<< '\n';
    float centroid_x =centroid[0], centroid_y=centroid[1];

    cent_x.push_back(centroid_x);
    cent_y.push_back(centroid_y);
    cent_x.push_back(end_cluster);
    cent_y.push_back(end_cluster);
    
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it=cloud_hull->points.begin();it !=cloud_hull->points.end();++it) {
      x.push_back( it->x);
      y.push_back( it->y); 
    }
    x.push_back(end_cluster);
    y.push_back(end_cluster);
    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    // std::stringstream ss;
    // ss << "cloud_cluster_" << j << ".pcd";
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		std::cout << "PointCloud representing the Cluster: " << cloud_hull->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_hull_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_hull, false); //*
    j++;
  }

  return (0);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_node");
	pointCloudSegmenter pcs = pointCloudSegmenter();
	ROS_INFO("Node initialize");

	ros::spin();
}
