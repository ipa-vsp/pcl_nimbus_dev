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
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("cube.pcd", *model);

    model->sensor_origin_[0] = 0;
    model->sensor_origin_[1] = 0;
    model->sensor_origin_[2] = 1;
    //model->sensor_origin_[3] = 10;
    for(int i = 0; i < model->size(); i++){
        model->points[i].x = model->points[i].x / 100;
        model->points[i].y = model->points[i].y / 100;
        model->points[i].z = model->points[i].z / 100;
        if (i >= 7489){
            model->points[i].x = NAN;
            model->points[i].y = NAN;
            model->points[i].z = NAN;   
        }
    }

    /** Compute Normals */
    // http://pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setRadiusSearch(0.5f);
    ne.setSearchMethod(tree);
    ne.setInputCloud(model);
    ne.compute(*model_normals);
    std::cout << "Size of normal cloud:" << model_normals->size() << std::endl;
    viewer = normalsVis(model, model_normals);
    pcl::io::savePCDFile("scaledCube.pcd", *model);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}