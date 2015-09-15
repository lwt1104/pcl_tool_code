

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "png++/png.hpp"
#include <sstream>

typedef pcl::PointXYZRGB PointType;
int main(int argc, char** argv) {

  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

  std::string pcd_filename;
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  for (size_t index = 0; index < filenames.size(); index ++) {
    if (pcl::io::loadPCDFile<PointType> (argv[filenames[index]], *cloud) == -1) {
      PCL_ERROR ("Couldn't read file %s.pcd \n", argv[filenames[index]]);
      continue;
    }
  // Visualization code for testing purposes (requires libpng++)
    png::image<png::rgb_pixel> image (cloud->width, cloud->height);
    int i = 0;
    for (size_t y = 0; y < image.get_height (); ++y) {
	  for (size_t x = 0; x < image.get_width (); ++x) {
	    const PointType & p = cloud->points[i++];
	    image[y][x] = png::rgb_pixel(p.r, p.g, p.b);
	  }
    }
    pcd_filename = argv[filenames[index]];
    pcd_filename.replace(pcd_filename.length () - 4, 4, ".png");

    image.write(pcd_filename);
  }

  return 0;
}