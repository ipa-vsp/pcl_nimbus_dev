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
#include <pcl/filters/uniform_sampling.h>

#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("cylinder.pcd", *cloud_cyl);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    nimbus::WebSocketClient wbClient((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);



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

    /** Compute Normals */
    // http://pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_cyl);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cyl(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals_cyl);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normal);

    /** Downsample cloud to extract Key Points (Filters) */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl_keypoints (new pcl::PointCloud<pcl::PointXYZ>());

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(0.03f);
    uniform_sampling.filter(*cloud_keypoints);
    std::cout << "Scense total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud(cloud_cyl);
    uniform_sampling.setRadiusSearch(0.01f);
    uniform_sampling.filter(*cloud_cyl_keypoints);
    std::cout << "Model total points: " << cloud_cyl->size () << "; Selected Keypoints: " << cloud_cyl_keypoints->size () << std::endl;

    /** Compute descriptor for keypoints */
    pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor (new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::SHOT352>::Ptr cloud_cyl_descriptor (new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTColorEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descriptor_estimation;
    descriptor_estimation.setRadiusSearch(0.02f);

    descriptor_estimation.setInputCloud(cloud_keypoints);
    descriptor_estimation.setInputNormals(cloud_normal);
    descriptor_estimation.setSearchSurface(cloud);
    descriptor_estimation.compute(*cloud_descriptor);

    descriptor_estimation.setInputCloud(cloud_cyl_keypoints);
    descriptor_estimation.setInputNormals(cloud_normals_cyl);
    descriptor_estimation.setSearchSurface(cloud_cyl);
    descriptor_estimation.compute(*cloud_cyl_descriptor);

    viewer = normalsVis(cloud, cloud_normal);

    while (!viewer->wasStopped())
    {
               
        viewer->spinOnce();
    }
    return 0;
}