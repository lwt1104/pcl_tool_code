#define PCL_NO_PRECOMPILE
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



struct _PointXYZRGBUV
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  uint32_t u;   // col number
  uint32_t v;   // row number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

 struct EIGEN_ALIGN16 PointXYZRGBUV : public _PointXYZRGBUV
  {
    inline PointXYZRGBUV(const _PointXYZRGBUV &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgb = p.rgb;
      u = p.u; v = p.v;
    }

    inline PointXYZRGBUV ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = a = 0;
      u = v = 0;
    }
    inline PointXYZRGBUV (const pcl::PointXYZRGB &p, uint32_t uu, uint32_t vv)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgb = p.rgb;
      u = uu; v = vv;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBUV& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBUV,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (uint32_t, u, u)
                                   (uint32_t, v, v)
)

typedef pcl::PointXYZRGB PointType;

bool
remove_plane(pcl::PointCloud<PointType>::Ptr cloud, float min_depth , float max_depth ) {
    for (size_t i = 0; i < cloud->size(); i++) {
      PointType &p = cloud->points[i];
      if (p.z < min_depth || p.z > max_depth) {
        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
      }
    }
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointType> seg;
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
    for (size_t i = 0; i < inliers->indices.size(); i++) {
      PointType &p = cloud->points[inliers->indices[i]];
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
    }

  return true;
}

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  reader.read (argv[1], *cloud);
  std::string pcd_filename;

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  remove_plane(cloud, 0.2, 2.0);

  pcd_filename = argv[1];
  pcd_filename.replace(pcd_filename.length () - 4, 8, "plane.pcd");
  pcl::io::savePCDFile(pcd_filename, *cloud);

  std::cout << "PointCloud after removing the plane has: " << cloud->points.size () << " data points." << std::endl; //*

  pcl::PointCloud<PointXYZRGBUV>::Ptr cloud_uv (new pcl::PointCloud<PointXYZRGBUV>);
  for (size_t index = 0; index < cloud->points.size(); index++) {
    const pcl::PointXYZRGB & p = cloud->points[index];
    if (p.x != p.x || p.y != p.y || p.z != p.z) { // if current point is invalid
      continue;
    }

    PointXYZRGBUV cp = PointXYZRGBUV(p, index % cloud-> width, index / cloud->width);
    cloud_uv->points.push_back (cp); 
  }
  cloud_uv->width = cloud_uv->points.size ();
  cloud_uv->height = 1;
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointXYZRGBUV>::Ptr tree (new pcl::search::KdTree<PointXYZRGBUV>);
  tree->setInputCloud (cloud_uv);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGBUV> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (300);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_uv);
  ec.extract (cluster_indices);


  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        PointXYZRGBUV& p = cloud_uv->points[*pit];
        pcl::PointXYZRGB cp_rgb;
        cp_rgb.x = p.x; cp_rgb.y = p.y; cp_rgb.z = p.z;
        cp_rgb.rgb = p.rgb; 
        cloud_cluster->points.push_back(cp_rgb); 
    }
    cloud_cluster->is_dense = true;
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    pcd_filename = argv[1];
    std::stringstream ss;
    ss << "cluster_" << j++ << ".pcd";
    pcd_filename.replace(pcd_filename.length () - 4, ss.str().length(), ss.str());
    pcl::io::savePCDFile(pcd_filename, *cloud_cluster);

    // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
  }

  return (0);
}