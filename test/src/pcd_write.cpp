#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>


int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
     pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //Fill the cloud data
    cloud->width = 356;
    cloud->height = 286;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    for(std::size_t i=0; i < cloud->points.size(); ++i){
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    viewer.showCloud(cloud);
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

    return (0);
}