/* 
Vinicius Conti da Costa
Nanodegree Sensor Fusion Engineer
Project 1 - Lidar Obstacle Detection
2nd try - April 05 2021
*/

#include <iostream>
#include <cmath>
#include <ios>
#include <string>
#include "render/render.h"
#include "main_processPcl.cpp"
#include "main_processPcl.h"
// #include "kdtree.h"



// Define the point processor <XYZI> (Vector XYZ + Intensity of each point)
void pclData (pcl::visualization::PCLVisualizer::Ptr& viewer, Main_ProcessPcl<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // 1 - Filtering the original cloud. Defining the coordinates of illustration.

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-15, -5, -2.1, 1), Eigen::Vector4f (35, 7, 5, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    // 2 - Segmenting (Obstacles & Floor)

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.MainRansac(filterCloud, 100, 0.2);

    // 3 - Clustering (Color & Box)
    // Some issues on that stage, hope it works properly.
    
    KdTree* tree = new KdTree;
  
    for (int i=0; i<segmentCloud.first->points.size(); i++) 
    	tree->insert(segmentCloud.first->points[i],i);


    // renderPointCloud(viewer,segmentCloud.first,"ObstCloud", Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,0));
    // Time segmentation process
  	// auto startTime = std::chrono::steady_clock::now();
  	
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.euclideanCluster(segmentCloud.first, tree, 0.4, 10, 500);
    renderPointCloud(viewer, segmentCloud.first, "Obstacles", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Floor", Color(0, 1, 0));
    
    

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudCluster = pointProcessor.Clustering (segmentCloud.first, 0.4, 10, 500);
    std::vector<Color> colors = {(Color(1,0,0), Color(1,1,0), Color(0,0,1))};

    int clusterId = 0;

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudCluster) {

        // 3.1 - Cluster colors
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "Obstacles"+std::to_string(clusterId), colors[clusterId]);

        // 3.2 - Cluster boxes
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }  
}

// Camera Parameters (angle, POV)
void camera_params (CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer -> setBackgroundColor (0, 0, 0);
    viewer -> initCameraParameters ();

    // Possibility to pick the desired angle & POV. **NEW**: driverPOV.
    switch (setAngle) {
    
        case driverPOV: viewer -> setCameraPosition (-7, 1, -1, 0, 0, 1);
        break;
        
        case XY : viewer->setCameraPosition (-16, -16, 16, 1, 1, 0);
        break;
        
        case TopDown : viewer->setCameraPosition (0, 0, 16, 1, 0, 1);
        break;
        
        case Side : viewer->setCameraPosition (0, -16, 0, 0, 0, 1);
        break;
        
        case FPS : viewer->setCameraPosition (-10, 0, 0, 0, 0, 1);
    }

    // Is axis necessary? 
        if (setAngle == TopDown || setAngle == Side || setAngle == XY)
            viewer -> addCoordinateSystem (1.0);

}

int main (int argc, char** argv)
{
    std::cout << "Starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    camera_params (setAngle, viewer);

    Main_ProcessPcl<pcl::PointXYZI> pointProcessor;
    // = new Main_ProcessPcl<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd ("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin ();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer -> wasStopped ()) {

        // Clear viewer
        viewer -> removeAllPointClouds ();
        viewer -> removeAllShapes ();

        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor.loadPcd((*streamIterator).string ());
        pclData (viewer, pointProcessor, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end ())
            streamIterator = stream.begin ();

        viewer -> spinOnce ();
    } 
}
