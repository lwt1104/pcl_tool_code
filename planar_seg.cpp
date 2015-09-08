#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>


int
 main (int argc, char** argv)
{
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  for (size_t index = 0; index < filenames.size(); index++) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[filenames[index]], *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s.pcd \n", argv[filenames[index]]);
      continue;
    }

    // pcl::visualization::CloudViewer viewer_input("Original cloud");
    // viewer_input.showCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }
    std::cerr << "Total number of points before segment plane: " << cloud->size() << std::endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_extract(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
    for (size_t i = 0; i < inliers->indices.size(); i++) {
      pcl::PointXYZRGBA &p = cloud_extract->points[inliers->indices[i]];
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
    }
    std::string pcd_filename = argv[filenames[index]];
    pcd_filename.replace(pcd_filename.length () - 4, 8, "_seg.pcd");
    pcl::io::savePCDFile(pcd_filename, *cloud_extract);
  }
  return (0);
}