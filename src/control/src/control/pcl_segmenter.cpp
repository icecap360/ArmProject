// general inports
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
// segmentation imports
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
// concave hull imports
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
// service imports
#include <control/cluster_points.h>
#include <control/doService.h>
#include <control/segmentComplete.h>

// global definitions
uint32_t queue_size = 1;
class pclSegmenter;

class pclSegmenter{
	public:
		// attributes of class
		ros::NodeHandle nh;
    ros::Publisher cloud_hull_pub;
    ros::Subscriber pcl_sub;
    ros::ServiceServer get_hulls_serv;
    // ros::ServiceClient pcl_segment_complete_serv;
		bool go_segment_and_publish;

		// constructor
		pclSegmenter();
		// helpers
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample (
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			float downsampleSize);
		pcl::PointCloud<pcl::PointXYZ>::Ptr removePlane (
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
			float planeTolerance);
		std::vector<pcl::PointIndices> cluster (
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
			float clusterTolerance);
		pcl::PointCloud<pcl::PointXYZ>::Ptr concave_hull (
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
			float hullAlpha);
		// downsampleSize and tolerances are in (m)
    void segment_and_publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			float downsampleSize, float planeTolerance,
			float clusterTolerance, float hullAlpha);
		// callbacks
		void pcl_subcb(
			const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
		bool get_hulls_servcb(control::doService::Request &req,
			 control::doService::Response &res);
};

// constructor
pclSegmenter::pclSegmenter () {
	// init segment_and_publish to off
	go_segment_and_publish = false;
	// init publishers, subscribers, and services
  cloud_hull_pub = nh.advertise<control::cluster_points>("cloud_hulls", 10);
  pcl_sub = nh.subscribe<sensor_msgs::PointCloud2> (
		"/camera/depth/points", queue_size,
		&pclSegmenter::pcl_subcb, this);
   	ROS_INFO("Node Subscribed");
  get_hulls_serv = nh.advertiseService("get_pcl_hulls", &pclSegmenter::get_hulls_servcb, this);
  // pcl_segment_complete_serv = nh.serviceClient<control::segmentComplete>("pcl_segment_complete");
}

// service callback
bool pclSegmenter::get_hulls_servcb (
  control::doService::Request &req,
  control::doService::Response &res)
{
  go_segment_and_publish=true;
  res.success=true;
  return true;
}

// subscriber callback
// called whenever subscribed message received
void pclSegmenter::pcl_subcb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
	if (!go_segment_and_publish) {
		return;
	}
	// converts pointcloud2 to pcl::PointXYZ
	std::cout<< "Converting pointcloud2 to pcl_pointcloud xyz";
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  std::cout<<"temp_cloud has: "<< temp_cloud->size() <<" size ";

	// segment into clusters and publish concave hull points
	segment_and_publish(temp_cloud, 0.01f, 0.02, 0.03, 0.01);
	// stop publishing until service called again
  go_segment_and_publish = false;

	// // because the segmentation topics are updated, call the service
	// control::segmentComplete srv; //the request for segmentComplete is empty
	// if (pcl_segment_complete_serv.call(srv)) {
  //    ROS_INFO("success");
  //  } else {
  //    ROS_ERROR("Failed to call service");
  //  }

  std::cout<<"Finished executed callback"<<'\n';
}

// voxel downsample
pcl::PointCloud<pcl::PointXYZ>::Ptr pclSegmenter::downsample (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	float downsampleSize)
{
	// Create the filtering object: downsample the dataset using a leaf size of downsampleSize (m)
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (downsampleSize, downsampleSize, downsampleSize);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()
		<< " data points." << std::endl;
	return cloud_filtered;
}

// removes main plane
pcl::PointCloud<pcl::PointXYZ>::Ptr pclSegmenter::removePlane (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
	float planeTolerance)
{
	// Create the segmentation object for the planar model and set all the parameters
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (planeTolerance);

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
    std::cout << "PointCloud representing the planar component: "
			<< cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
	return cloud_filtered;
}

// cluster points using KdTree and GNN
std::vector<pcl::PointIndices> pclSegmenter::cluster (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
	float clusterTolerance)
{
	// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (clusterTolerance);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

	return cluster_indices;
}

// constructs 2D concave hull
pcl::PointCloud<pcl::PointXYZ>::Ptr pclSegmenter::concave_hull (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
	float hullAlpha)
{
	// projects points onto plane before computing hull
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
	chull.setAlpha (hullAlpha);
	chull.reconstruct (*cloud_hull);

	return cloud_hull;
}

// main segmenter
// tolerances are in m units
void pclSegmenter::segment_and_publish (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	float downsampleSize, float planeTolerance,
	float clusterTolerance, float hullAlpha)
{
  // pcl clusters later saved into files
	pcl::PCDWriter writer;

	// voxel downsample
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = downsample(cloud, downsampleSize);
	// remove main planar surface
	cloud_filtered = removePlane(cloud_filtered, planeTolerance);
	// cluster using KdTrees
	std::vector<pcl::PointIndices> cluster_indices = cluster(cloud_filtered, clusterTolerance);

	// for each cluster
  int j = 0;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> cent_x;
  std::vector<float> cent_y;
	// signifier for end of cluster points
  float end_cluster = std::numeric_limits<float>::infinity();

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
		std::cout << "Cluster " << j << '\n';

		// append point into cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_cluster->push_back ((*cloud_filtered)[*pit]);
		}

		// create concave hull from cluster
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull = concave_hull(cloud_cluster, hullAlpha);
		cloud_hull->width = cloud_hull->size ();
    cloud_hull->height = 1;
    cloud_hull->is_dense = true;

    //compute the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_hull,centroid);
    float centroid_x = centroid[0];
		float centroid_y = centroid[1];

    // update the msg data
		// add centroid of cluster
    cent_x.push_back(centroid_x);
    cent_y.push_back(centroid_y);
    cent_x.push_back(end_cluster);
    cent_y.push_back(end_cluster);
		// add cloud_hull points of cluster
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it=cloud_hull->points.begin();
			it !=cloud_hull->points.end(); ++it)
		{
      x.push_back( it->x);
      y.push_back( it->y);
    }
    x.push_back(end_cluster);
    y.push_back(end_cluster);

		// save hull as file for later viewing
		std::cout << "PointCloud representing the Cluster: " << cloud_hull->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_hull_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_hull, false);
    j++;

  }

  // creating the msg from the vectors and publish it
  control::cluster_points msg;
  msg.hull_x = x;
  msg.hull_y = y;
  msg.centroid_x = cent_x;
  msg.centroid_y = cent_y;
  cloud_hull_pub.publish(msg);
  std::cout <<"XYCentroid topics updated \n";
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_segmenter");
	pclSegmenter pcl_segmenter = pclSegmenter();
	ROS_INFO("Node initialized");

	ros::spin();
}
