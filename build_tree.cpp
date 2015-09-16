/*
 * =====================================================================================
 *
 *       Filename:  build_tree.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/12/2015 12:20:17 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Wentao Luan (wluan), wluan@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/filter.h>

typedef std::pair<std::string, std::vector<float> > vfh_model;
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{

  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType> ());

  if (pcl::io::loadPCDFile<PointType> (path.string(), *cloud) == -1) {
    PCL_ERROR ("Couldn't read file %s.pcd \n", path.string().c_str());
    return false;
  }
  pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
  std::vector<int> filter_index;
  pcl::removeNaNFromPointCloud (*cloud, *cloud_filtered, filter_index);
  cloud = cloud_filtered;

  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (15);
  norm_est.setInputCloud (cloud);
  norm_est.compute (*cloud_normals);

  pcl::VFHEstimation<PointType, NormalType, pcl::VFHSignature308> vfh_est;
  vfh_est.setInputCloud (cloud);
  vfh_est.setInputNormals (cloud_normals);

  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  vfh_est.setSearchMethod (tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  vfh_est.compute (*vfhs);

  vfh.second.resize (308);

  for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); ++i)
  {
    vfh.second[i] = vfhs->points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}

/** \brief Load a set of VFH features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension containing the VFH features
  * \param models the resultant vector of histogram models
  */
void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                   std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it(base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory(it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels(it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      vfh_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
    return (-1);
  }

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";

  std::vector<vfh_model> models;

  // Load the model histograms
  loadFeatureModels (argv[1], extension, models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", 
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

  // Convert data into FLANN format
  flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = models[i].second[j];

  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
    fs << models[i].first << "\n";
  fs.close ();
 
  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  // flann::Index<flann::HellingerDistance<float> > index (data, flann::LinearIndexParams ());
  
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();

  return (0);
}

