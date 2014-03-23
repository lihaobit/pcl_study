#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <unistd.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

float z_pass_min_ (-0.500f);
float z_pass_max_ (1.200f);
float y_pass_min_ (-1.00f);
float y_pass_max_ (1.00f);
float x_pass_min_ (-0.1f);
float x_pass_max_ (0.1f);
float sample_size_ (0.01f);


float descr_rad_ (0.02f);

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
boost::mutex cloud_mutex;
pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> descr_est;
pcl::NormalEstimationOMP<PointT, NormalType> norm_est;
void
parseCommandLine (int argc, char *argv[])
{
  //General parameters
  pcl::console::parse_argument (argc, argv, "--z_max", z_pass_max_);
  pcl::console::parse_argument (argc, argv, "--z_min", z_pass_min_);
  pcl::console::parse_argument (argc, argv, "--y_max", y_pass_max_);
  pcl::console::parse_argument (argc, argv, "--y_min", y_pass_min_);
  pcl::console::parse_argument (argc, argv, "--x_max", x_pass_max_);
  pcl::console::parse_argument (argc, argv, "--x_min", x_pass_min_); 
  pcl::console::parse_argument (argc, argv, "--sample_size", sample_size_);
}

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("../bottle.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
   parseCommandLine(argc, argv);
   std::cout << "argc:" << argc << " argv:" << *argv << std::endl;
   std::cout << "x_min:" << x_pass_min_ << " x_max:" << x_pass_max_ << " y_min:" << y_pass_min_ << " y_max:"<<y_pass_max_<<std::endl;

	/*apply pass through filter*/
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setFilterLimitsNegative (true);
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_pass_min_, z_pass_max_);
	pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y_pass_min_, y_pass_max_);
	pass.filter (*cloud_filtered);
	
        pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x_pass_min_, x_pass_max_);
	pass.filter (*cloud_filtered);

	viewer.addPointCloud<PointT> (cloud_filtered, "input_cloud");
        #if 0	
       // while (!viewer.wasStopped ())
	{
	//	viewer.spinOnce ();
	} 
        return(0);
        #endif	
        
	viewer.removeAllPointClouds();
        cloud = cloud_filtered;

    
#if 0
#if 0
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
#endif
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

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
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
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  viewer.addPointCloud<PointT> (cloud_filtered, "input_cloud");
  while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	} 	
#endif
#if 1

  pcl::PCDWriter writer;
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

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cout << "ss:" << j << std::endl;  
    
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    cout << "ss:" << j << std::endl;  
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
#endif
}
