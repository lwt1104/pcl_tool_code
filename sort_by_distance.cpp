#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

float get_depth(const pcl::PointCloud<PointType>::Ptr& cloud) {
  float zsum = 0;
  int num = 0;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    PointType &p = cloud->points[i];
    if (!pcl::isFinite(p)) {
      continue;
    }
    zsum += p.z;
    num++;
  }
  float distance = zsum / num;
  return distance;
}

std::string get_folder_name(float d, int scale) {
  float step = (float)scale / 100.0;
  int level = d / step;
  std::stringstream ss;
  ss << level * scale << "-" << level * scale + scale - 1;
  return ss.str();
}

bool
sort_by_distance(const boost::filesystem::path &base_dir, int step) {
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir)) {
    PCL_ERROR ("Couldn't find the directory %s.\n", base_dir.string().c_str());
    return false;
  }

  for (boost::filesystem::directory_iterator it(base_dir); it != boost::filesystem::directory_iterator (); ++it) {
    if (!boost::filesystem::is_regular_file (it->status ()) || boost::filesystem::extension (it->path ()) != ".pcd") {
      continue;
    }
    std::string pcd_name = it->path().filename().string(), png_name = pcd_name, box_name = pcd_name;
    png_name.replace(pcd_name.length () - 4, 4, ".png");
    box_name.replace(pcd_name.length () - 4, 7, "box.png");
    std::string current_dir = it->path().parent_path().string();
    if (!boost::filesystem::is_regular_file(current_dir + "/" + png_name) || !boost::filesystem::is_regular_file(current_dir + "/" + box_name)) {
      PCL_ERROR ("Couldn't find png or box.png files for %s.\n", pcd_name.c_str());
      continue; 
    }
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (current_dir + "/" + pcd_name, *cloud) == -1) {
      PCL_ERROR ("Couldn't read file %s.pcd \n", pcd_name.c_str());
      continue;
    }
    
    float depth = get_depth(cloud);
    std::string folder_name = get_folder_name(depth, step);
    folder_name = it->path().parent_path().string() + "/" + folder_name;
    if (!boost::filesystem::is_directory (folder_name)) {
      boost::filesystem::create_directory(folder_name);
    }
    boost::filesystem::copy_file(current_dir + "/" + pcd_name, folder_name + "/" + pcd_name, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::copy_file(current_dir + "/" + png_name, folder_name + "/" + png_name, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::copy_file(current_dir + "/" + box_name, folder_name + "/" + box_name, boost::filesystem::copy_option::overwrite_if_exists);
  }
  return true;
}


int
main(int argc, char** argv) {
  if (argc < 2) {
    PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
    return (-1);
  }

  int step = 20;
  pcl::console::parse_argument (argc, argv, "-step", step);
  
  sort_by_distance(argv[1], step);

  return 0;
}