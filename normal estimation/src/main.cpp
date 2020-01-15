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

#include <pcl/surface/mls.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include "websocket.h"

int user_data;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;

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
        //cloud->points.push_back(basic_points);
    }
    pcl::io::loadPCDFile("cloud.pcd", *cloud);

    /** Compute Normals */
    // http://pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_cyl);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cyl(new pcl::PointCloud<pcl::Normal>());
    ne.setRadiusSearch(0.03f);
    ne.compute(*cloud_normals_cyl);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normal);

    /** Or use Moving Least Square to compute the normals */
    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal;
    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_cyl;
    // pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // mls.setComputeNormals(true);
    // mls.setInputCloud (cloud);
    // mls.setPolynomialOrder(2);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.3f);
    // mls.process(*cloud_normal);

    /** Downsample cloud to extract Keypoints  */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl_keypoints (new pcl::PointCloud<pcl::PointXYZ>());

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(0.05f);
    uniform_sampling.filter(*cloud_keypoints);
    std::cout << "Scense total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud(cloud_cyl);
    uniform_sampling.setRadiusSearch(0.01f);
    uniform_sampling.filter(*cloud_cyl_keypoints);
    std::cout << "Model total points: " << cloud_cyl->size () << "; Selected Keypoints: " << cloud_cyl_keypoints->size () << std::endl;

    /** Compute descriptor for keypoints */
    pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor (new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::SHOT352>::Ptr cloud_cyl_descriptor (new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descriptor_estimation;
    descriptor_estimation.setRadiusSearch(0.3f);

    descriptor_estimation.setInputCloud(cloud_keypoints);
    descriptor_estimation.setInputNormals(cloud_normal);
    descriptor_estimation.setSearchSurface(cloud);
    descriptor_estimation.compute(*cloud_descriptor);

    descriptor_estimation.setInputCloud(cloud_cyl_keypoints);
    descriptor_estimation.setInputNormals(cloud_normals_cyl);
    descriptor_estimation.setSearchSurface(cloud_cyl);
    descriptor_estimation.compute(*cloud_cyl_descriptor);

    /** Find Model-Scene Correspondence with kd-tree*/
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(cloud_descriptor);
    // For each scene keypoint decriptor, find nearest neighbor into model descriptor cloud and add it to the correspondese vector
    for (std::size_t i = 0; i < cloud_cyl_descriptor->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!std::isfinite (cloud_cyl_descriptor->at (i).descriptor[0])) //skipping NaNs
            continue;
        int found_neighs = match_search.nearestKSearch (cloud_cyl_descriptor->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    // Actual Clustring
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    /** Using Hough3d*/
    // Compute (keypoints) reference frame only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (0.015f);

    rf_est.setInputCloud (cloud_keypoints);
    rf_est.setInputNormals (cloud_normal);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (cloud_cyl_keypoints);
    rf_est.setInputNormals (cloud_normals_cyl);
    rf_est.setSearchSurface (cloud_cyl);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (0.01f);
    clusterer.setHoughThreshold (5.0f);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (cloud_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (cloud_cyl_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);
    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);

    //
    //  Output results
    //
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }
    
    viewer = normalsVis(cloud, cloud_normal);

    while (!viewer->wasStopped())
    {
               
        viewer->spinOnce();
    }
    return 0;
}