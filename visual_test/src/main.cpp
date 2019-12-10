#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include "websocket.h"

int user_data;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return(viewer);
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
}

void viewPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop" << count ++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
    user_data ++;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("ism_test_cat.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Simple cloud viewer");
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

    std::cout << "Size of the cloud is: " << cloud->points.size() << std::endl;

    viewer.showCloud(cloud);
    pcl::io::savePCDFileASCII ("test_pcd_my.pcd", *cloud);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    viewer.runOnVisualizationThread(viewPsycho);

    while (!viewer.wasStopped())
    {
        user_data ++;
        // res = wbClient.getImage();
        // if(!res.empty()){
        //     for(int i = 0; i < res[0].size(); i++)
        //     {
        //         pcl::PointXYZ basic_points;
        //         basic_points.x = res[0][i];
        //         basic_points.y = res[1][i];
        //         basic_points.z = res[2][i];
        //         cloud->points.push_back(basic_points);
        //     }
        //     viewer.
        //     viewer.showCloud(cloud);
        // }
    }
    return 0;
}