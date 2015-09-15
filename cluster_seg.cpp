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
#include "png++/png.hpp"


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



int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

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
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_uv);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_cluster->width = 640;
    cloud_cluster->height = 480;
    cloud_cluster->points.resize(cloud_cluster->width * cloud_cluster->height);
    for (size_t i = 0; i < cloud_cluster->points.size(); i++) {
      cloud_cluster->points[i].x = std::numeric_limits<float>::quiet_NaN ();
      cloud_cluster->points[i].y = std::numeric_limits<float>::quiet_NaN ();
      cloud_cluster->points[i].z = std::numeric_limits<float>::quiet_NaN ();
      cloud_cluster->points[i].rgb = std::numeric_limits<float>::quiet_NaN ();
    }
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        PointXYZRGBUV& p = cloud_uv->points[*pit];
        pcl::PointXYZRGB cp_rgb;
        cp_rgb.x = p.x; cp_rgb.y = p.y; cp_rgb.z = p.z;
        cp_rgb.rgb = p.rgb; 
        cloud_cluster->points[p.v * cloud_cluster->width + p.u] = cp_rgb; 
    }
    
    
    
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
