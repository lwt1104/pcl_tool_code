
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <boost/filesystem.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

int
main (int argc, char** argv) {

  int k = 9;
  pcl::visualization::PCLVisualizer::Ptr p(new pcl::visualization::PCLVisualizer(argc, argv, "Batch viewer"));
  int y_s = (int)floor (sqrt ((double)k));
  int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
  double x_step = (double)(1 / (double)x_s);
  double y_step = (double)(1 / (double)y_s);
  int viewport = 0, l = 0, m = 0;
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

  for (size_t index = 0; index < filenames.size(); index ++) {
  	std::string cloud_name = argv[filenames[index]];
    if (pcl::io::loadPCDFile<PointType> (cloud_name, *cloud) == -1) {
      PCL_ERROR ("Couldn't read file %s.pcd \n", argv[filenames[index]]);
      continue;
    }

    (*p).createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s) {
      l = 0;
      m++;
    }
    // Demean the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    pcl::PointCloud<PointType>::Ptr cloud_demean (new pcl::PointCloud<PointType>);
    pcl::demeanPointCloud<PointType> (*cloud, centroid, *cloud_demean);

    (*p).addPointCloud (cloud_demean, cloud_name, viewport);
    (*p).addText (argv[filenames[index]], 20, 10, cloud_name, viewport);
    (*p).setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, cloud_name, viewport);

    if (index % k == k - 1 || index == filenames.size() - 1) {
      // Add coordianate systems to all viewports
      (*p).addCoordinateSystem (0.1, "global", 0);
      (*p).spin();
      l = 0;
      m = 0;
      (*p).removeAllPointClouds(viewport);
      (*p).removeAllShapes(viewport);
      // pcl::visualization::PCLVisualizer::Ptr p2(new pcl::visualization::PCLVisualizer(argc, argv, "Batch viewer"));
      // p = p2;
    }

  }

}