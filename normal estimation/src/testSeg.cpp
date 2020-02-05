#include <iostream>
#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "websocket.h"


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


/** Mean Filter
 * @brief Perform the arithmetic mean of valid points
 * @param blob Point cloud frames
 * @param numFrame number of frames for mean validation of points
 * @return resulted frame
 * Delete the oldest point cloud
*/
void meanFilter(std::queue<std::vector<std::vector<float>>> blob, int numFrame, std::vector<std::vector<float>> * res){
    // Seperate the x, y, z;
    std::vector<std::vector<float>> buf; 
    int meanDivider = blob.size();
    std::vector<float> addX(286*352, 0);
    std::vector<float> addY(286*352, 0);
    std::vector<float> addZ(286*352, 0);
    std::vector<float> conf(286*352, 0);
    std::vector<float> meanCounter(286*352, 0);
    if(meanDivider <= numFrame){
        while(!blob.empty()){
            buf = blob.front();
            blob.pop();
            for(int i = 0; i < buf[0].size(); i++){
                if(isnan(buf[0][i]) || isnan(buf[1][i]) || isnan(buf[2][i]))
                    conf[i] = 1;
                else{
                    addX[i] += buf[0][i];
                    addY[i] += buf[1][i];
                    addZ[i] += buf[2][i];
                    meanCounter[i] += 1;
                }
            }
        }
        int confCounter = 0;
        for(int i = 0; i < conf.size(); i++){
            if(meanCounter[i] <= 1){
                addX[i] = NAN;
                addY[i] = NAN;
                addZ[i] = NAN;
                confCounter ++;
            }else{
                addX[i] = addX[i] / meanCounter[i];
                addY[i] = addY[i] / meanCounter[i];
                addZ[i] = addZ[i] / meanCounter[i];
            }
        }
        std::cout << "Confidence Matrix element counter: " << confCounter << std::endl;
        res->push_back(addX);
        res->push_back(addY);
        res->push_back(addZ);
       
    }else{
        return;
    }

}


int main(int argc, char** argv)
{
    nimbus::WebSocketClient wbClient("192.168.0.69", false, 8080, 8383, 3, 10);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);

    scene->width = 352 * 286;
    scene->height = 1;
    scene->is_dense = true;
    std::vector<std::vector<float>> res; //(3, std::vector<float>(286*352, 0));
    std::queue<std::vector<std::vector<float>>> blob;
    int numFrame = 0;
    while (numFrame < 500){
        res = wbClient.getImage();
        if(!res.empty()){
            blob.push(res);
            numFrame ++;
        }else{
            std::cout << "Empty frame" << std::endl; 
        }
    }

    std::vector<std::vector<float>> resFil;
    meanFilter(blob, 500, &resFil);
    for(int i = 0; i < res[0].size(); i++)
    {
        pcl::PointXYZ basic_points;
        basic_points.x = resFil[0][i];
        basic_points.y = resFil[1][i];
        basic_points.z = resFil[2][i];
        scene->points.push_back(basic_points);
    }

   
    std::cerr << "Point Cloud after filtering: " << scene->width * scene->height << " data Points" << std::endl;

    // pcl::io::loadPCDFile("kinect_cloud.pcd", *scene);

    pcl::toPCLPointCloud2<pcl::PointXYZ>(*scene, *cloud_blob);

    std::cerr << "Point Cloud before filtering: " << cloud_blob->width * cloud_blob->height << " data Points" << std::endl;

    // Create the filtering object : down sampling the dataset using the leaf size 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // Conver to the templated point cloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr viewer_filtered;
    pcl::visualization::PCLVisualizer::Ptr viewer_p;
    pcl::visualization::PCLVisualizer::Ptr viewer_f;
    std::cerr << "Point Cloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data Points" << std::endl;

    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr Inliner (new pcl::PointIndices());
    // Create segmented object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fil = cloud_filtered;

    int i = 0, nr_point = (int) cloud_filtered->points.size();
    while(cloud_filtered->points.size() > 0.3 * nr_point){
        // Segment the largest plannar component from the remaining cloud
        seg.setInputCloud(cloud_fil);
        seg.segment(*Inliner, *coefficient);
        if(Inliner->indices.size() == 0){
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        //Extract the inliners
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(Inliner);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_fil.swap(cloud_f);

        i++;
    }

    viewer = simpleVis(scene, "Cloud Blob");
    viewer_filtered = simpleVis(cloud_filtered, "Filtered Cloud");
    viewer_p = simpleVis(cloud_p, "False Negetive indice extration");
    viewer_f = simpleVis(cloud_f, "Negetive indice extration");
    while(!viewer->wasStopped() & !viewer_filtered->wasStopped() & !viewer_p->wasStopped() & !viewer_f->wasStopped()){
        viewer->spinOnce();
        viewer_filtered->spinOnce();
        //viewer_p->spinOnce();
        //viewer_f->spinOnce();
    }
}