
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

  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  for (size_t index = 0; index < filenames.size(); index ++) {
    if (pcl::io::loadPCDFile<PointType> (argv[filenames[index]], *cloud) == -1) {
      PCL_ERROR ("Couldn't read file %s.pcd \n", argv[filenames[index]]);
      continue;
    }
    float zsum = 0;
    float xsum = 0;
    float ysum = 0;
    int num = 0;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointType &p = cloud->points[i];
      if (!pcl::isFinite(p)) {
        continue;
      }
      zsum += p.z;
      xsum += p.x;
      ysum += p.y;
      num++;
    }

   std::cout << argv[filenames[index]] <<" average depth: " << zsum / num << "average x: " << 
     xsum / num << "average y: " << ysum / num <<std::endl;

  }

  return 0;
}