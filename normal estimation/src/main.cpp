#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include "websocket.h"

int user_data;


pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string viewerName)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewerName.c_str()));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(0.005);
    viewer->initCameraParameters();
    return(viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    nimbus::WebSocketClient wbClient((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal>);

    cloud->width = 352 * 286;
    cloud->height = 1;
    cloud->is_dense = true;
    std::vector<std::vector<float>> res(3, std::vector<float>(286*352, 0));
    res = wbClient.getImage();
    std::uint8_t r(255), g(15), b(15);
    for(int i = 0; i < res[0].size(); i++)
    {
        pcl::PointXYZRGB basic_points;
        basic_points.x = res[0][i];
        basic_points.y = res[1][i];
        basic_points.z = res[2][i];
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
              static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        basic_points.rgb = *reinterpret_cast<float*>(&rgb);
        cloud->points.push_back(basic_points);
    }
    // http://pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
    std::vector<int> indices(std::floor(cloud->points.size()/10));
    for(size_t i = 0; i < indices.size(); ++i) indices[i] = i;
    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
    //ne.setIndices (indicesptr);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);

    viewer = normalsVis(cloud, cloud_normals);

    while (!viewer->wasStopped())
    {
        user_data ++;
        viewer->spinOnce();
    }
    return 0;
}