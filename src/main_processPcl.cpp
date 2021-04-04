#include "processPointClouds.h"
#include <iostream>
#include <cmath>
#include <string>

using namespace pcl;
using namespace std;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename PointCloud<PointT>::Ptr cloud)
{
    cout << cloud -> points.size() << endl;
}

template<typename PointT>
typename PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = chrono::steady_clock::now();

    VoxelGrid<PointT> voxel;
    typename PointCloud<PointT>::Ptr cloudFiltered(new PointCloud<PointT>);

    voxel.setInputCloud(cloud);
    voxel.setLeafSize(filterRes, filterRes, filterRes);
    voxel.filter(*cloudFiltered);

    typename PointCloud<PointT>::Ptr cloudRegion (new PointCloud<PointT>);

    CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    vector<int> indices;

    CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    auto endTime = chrono::steady_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "filtering took " << elapsedTime.count() << " milliseconds" << endl;

    return cloudRegion;
}


template<typename PointT>
pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(PointIndices::Ptr inliers, typename PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename PointCloud<PointT>::Ptr obstCloud (new PointCloud<PointT> ());
    typename PointCloud<PointT>::Ptr planeCloud (new PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    // pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = chrono::steady_clock::now();
	// PointIndices::Ptr inliers; // comment line because I used it in another part of the code.

    // TODO:: Fill in the function to segment cloud into two parts, the driveable plane and obstacles
    SACSegmentation<PointT> seg;
    PointIndices::Ptr inliers {new PointIndices};
    ModelCoefficients::Ptr coefficients {new ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        cout << "Could not estimate a planar model for the given dataset." << endl;
    }
    
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = chrono::steady_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << endl;

    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
vector<typename PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = chrono::steady_clock::now();

    vector<typename PointCloud<PointT>::Ptr> clusters;

    typename search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    vector<PointIndices> clusterIndices;
    EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(PointIndices getIndices: clusterIndices)
    {
        typename PointCloud<PointT>::Ptr cloudCluster (new PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    
    }

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = chrono::steady_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename PointCloud<PointT>::Ptr cloud, string file)
{
    io::savePCDFileASCII (file, *cloud);
    cerr << "Saved " << cloud->points.size () << " data points to "+file << endl;
}


template<typename PointT>
typename PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(string file)
{

    typename PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);

    if (io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    cerr << "Loaded " << cloud->points.size () << " data points from "+file << endl;

    return cloud;
}


template<typename PointT>
vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(string dataPath)
{

    vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}