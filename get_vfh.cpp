/*
 * =====================================================================================
 *
 *       Filename:  get_vfh.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/13/2015 10:17:01 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Wentao Luan (wluan), wluan@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/histogram_visualizer.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

int
main(int argc, char** argv) {
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType> ());

  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  for (size_t index = 0; index < filenames.size(); index ++) {
    if (pcl::io::loadPCDFile<PointType> (argv[filenames[index]], *cloud) == -1) {
      PCL_ERROR ("Couldn't read file %s.pcd \n", argv[filenames[index]]);
      continue;
    }
    //
    std::cout << "object density " << cloud->is_dense << std::endl;
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    std::vector<int> filter_index;
    pcl::removeNaNFromPointCloud (*cloud, *cloud_filtered, filter_index);
    cloud = cloud_filtered;


    //
    //  Compute Normals
    //
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (cloud);
    norm_est.compute (*cloud_normals);

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<PointType, NormalType, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

    std::cout << "points number " << vfhs->points.size () << std::endl;// should be of size 1*
    std::cout << "points number " <<  pcl::VFHSignature308::descriptorSize() << std::endl;// should be of size 1*
    
    // std::string pcd_filename = argv[filenames[index]];
    // pcd_filename.replace(pcd_filename.length () - 4, 10, "_vfh_t.pcd");
    // pcl::io::savePCDFile(pcd_filename, *vfhs);

  
    pcl::visualization::PCLHistogramVisualizer hist; 
    // const std::string id = argv[filenames[index]]; 
    const std::string id = "vfh_viewer"; 
    hist.addFeatureHistogram (*vfhs, 308, id, 600, 200); 
    hist.spin ();
  }
}
