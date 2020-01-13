#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>

#include <pcl/range_image/range_image.h>
#include "websocket.h"

int user_data;
float angular_resoulution_x = 0.5f, 
        angular_resoulution_y = angular_resoulution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f scene_senor_pose (Eigen::Affine3f::Identity());
    pcl::visualization::PCLVisualizer::Ptr viewer;
    nimbus::WebSocketClient wbClient((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal>);

    angular_resoulution_x = pcl::deg2rad(angular_resoulution_x);
    angular_resoulution_y = pcl::deg2rad(angular_resoulution_y);

    cloud_ptr->width = 352 * 286;
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
    std::vector<std::vector<float>> res(3, std::vector<float>(286*352, 0));
    res = wbClient.getImage();
    
    for(int i = 0; i < res[0].size(); i++)
    {
        pcl::PointXYZ basic_points;
        basic_points.x = res[0][i];
        basic_points.y = res[1][i];
        basic_points.z = res[2][i];
        cloud_ptr->points.push_back(basic_points);
    }
    pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;

    scene_senor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud.sensor_origin_[0],
                                                            cloud.sensor_origin_[1],
                                                            cloud.sensor_origin_[2])) *
                        Eigen::Affine3f(cloud.sensor_orientation_);
    // TODo :-) http://pointclouds.org/documentation/tutorials/range_image_visualization.php#range-image-visualization
    return 0;
}