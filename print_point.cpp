#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
//#include <Eigen/Geometry>


int
 main (int argc, char** argv)
{


std::vector<int> filenames;
filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

for (size_t index = 0; index < filenames.size(); index++) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[filenames[index]], *cloud_1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file 1.pcd \n");
    return (-1);
  }

  printf("w: %d,  h: %d", cloud_1->width, cloud_1 ->  height);
  
  float xmin = std::numeric_limits<float>::max (), ymin = std::numeric_limits<float>::max (),
        xmax = std::numeric_limits<float>::min (), ymax = std::numeric_limits<float>::min ();
  
  for (size_t i = 0; i < cloud_1 -> size(); i++) {
    const pcl::PointXYZRGBA & p = cloud_1->points[i];
    if (p.x != p.x ||
        p.y != p.y ||
        p.z != p.z) {
      continue;
    }

    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.y);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);

    printf("x:%f  y:%f  z:%f rgb:%d \n", p.x, p.y, p.z, p.rgba);
  }

  printf("xmin:%f  xmax:%f, ymin:%f  ymax:%f   \n", xmin, xmax, ymin, ymax); 
}
return (0);

}
