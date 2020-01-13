#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
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
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    nimbus::WebSocketClient wbClient((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal>);

    cloud->width = 352 * 286;
    cloud->height = 1;
    cloud->is_dense = true;
    std::vector<std::vector<float>> res(3, std::vector<float>(286*352, 0));
    res = wbClient.getImage();
    
    for(int i = 0; i < res[0].size(); i++)
    {
        pcl::PointXYZ basic_points;
        basic_points.x = res[0][i];
        basic_points.y = res[1][i];
        basic_points.z = res[2][i];
        cloud->points.push_back(basic_points);
    }
    // http://pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.05);
    ne.compute(*cloud_normals);

    viewer = normalsVis(cloud, cloud_normals);

    while (!viewer->wasStopped())
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // std::vector<std::vector<float>> resl(3, std::vector<float>(286*352, 0));
        // resl = wbClient.getImage();
        // if(!resl.empty())
        // {
        //     for(int i = 0; i < resl[0].size(); i++)
        //     {
        //         pcl::PointXYZ loadPoints;
        //         loadPoints.x = resl[0][i];
        //         loadPoints.y = resl[1][i];
        //         loadPoints.z = resl[2][i];
        //         newCloud->points.push_back(loadPoints);
        //     }
        //     ne.setInputCloud(newCloud);
        //     ne.compute(*cloud_normals);
        //     if(!viewer->updatePointCloud(newCloud,  "sample cloud"))
        //     {
        //         viewer->addPointCloud(newCloud, "sample cloud");
        //         viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(newCloud, cloud_normals, 10, 0.05, "normals");
        //     }
        // }        
        viewer->spinOnce();
    }
    return 0;
}