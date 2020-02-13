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

#include "websocket.h"


pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string viewerName)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewerName.c_str()));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(0.05);
    viewer->initCameraParameters();
    return(viewer);
}

/** @brief Remove the present of the cloud 
 * @param res Input blob point from nimbus camera
 * @param persent Persentage between 0.0 to 1.0 
 * @return edited cloud of default type pcl::PointXYZI
 * Currently only pcl::PointXYZI will work
*/
template <typename T >
void editCloud(std::vector<std::vector<float>> res, float persent, pcl::PointCloud<T> &edited){
    int height = 286;   // Row
    int width = 352;    // Column
    int hLower = (height * persent)/2;
    int hUpper = height - hLower;
    int wLower = (width * persent)/2;
    int wUpper = width - wLower;  
    std::vector<std::vector<float>> pointX;
    std::vector<std::vector<float>> pointY;
    std::vector<std::vector<float>> pointZ;
    std::vector<std::vector<float>> ampl;
    int counter = 0;
    for(int i = 0; i < height ; i++){
        std::vector<float> tempX;
        std::vector<float> tempY;
        std::vector<float> tempZ;
        std::vector<float> amplt;
        for(int j = 0; j < width; j++){
            if((i > hLower & i < hUpper) & (j > wLower & j < wUpper)){
                tempX.push_back(res[0][counter]);
                tempY.push_back(res[1][counter]);
                tempZ.push_back(res[2][counter]);
                amplt.push_back(res[3][counter]);
                counter ++;
            }else{
                counter ++;
            }
        }
        if(i > hLower & i < hUpper){
            pointX.push_back(tempX);
            pointY.push_back(tempY);
            pointZ.push_back(tempZ);
            ampl.push_back(amplt);
        }
    }
    counter = 0;
    for(int i = 0; i < pointX.size() ; i++){
        for(int j = 0; j < pointX[0].size(); j++){
            pcl::PointXYZI temp;
            temp.x = pointX[i][j];
            temp.y = pointY[i][j];
            temp.z = pointZ[i][j];
            temp.intensity = ampl[i][j];
            edited.points.push_back(temp);
            counter++;
        }
    }
}


int main(int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer;
    // pcl::io::loadPCDFile("cube.pcd", *model);
    nimbus::WebSocketClient wbClient("192.168.0.69", false, 8080, 8383, 3, 10);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scene(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<std::vector<float>> res;
    res = wbClient.getImage();
    editCloud<pcl::PointXYZI>(res, 0.7, *scene);
    viewer = simpleVis(scene, "Cloud Blob");
    // pcl::io::savePCDFile("cloudEdit.pcd", *scene);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}