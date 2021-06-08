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
    renderPointCloud(viewer,inputCloud, "inputCloud");
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

    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd ("/home/vant3d/catkin_ws/pcd_data_from_bag");
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
