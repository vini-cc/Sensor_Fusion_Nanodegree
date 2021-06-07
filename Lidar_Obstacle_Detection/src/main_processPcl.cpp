

#include "main_processPcl.h"
#include <iostream>
#include <cmath>
#include <string>
#include <unordered_set>
// #include "kdtree.h"


//constructor:
template<typename PointT>
Main_ProcessPcl<PointT>::Main_ProcessPcl() {}


//de-constructor:
template<typename PointT>
Main_ProcessPcl<PointT>::~Main_ProcessPcl() {}


template<typename PointT>
void Main_ProcessPcl<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud -> points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr Main_ProcessPcl<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

   /* //VoxelGrid
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    typename pcl::VoxelGrid<PointT>::Ptr vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
    typename pcl::CropBox<PointT>::Ptr region(true);
    region.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    region.setNegative(true);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    typename pcl::PointCloud<PointT>::Ptr cloudCropped (new pcl::PointCloud<PointT>());
    typename pcl::CropBox<PointT>::Ptr roof(true);
    roof.setInputCloud(cloudRegion);
    roof.setMin(minPoint);
    roof.setMax(maxPoint); 
    roof.filter(*cloudCropped);

    */


    // std::vector<int> indices;

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    //std::cout << typeid(vg).name() << endl;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // ----------------------------------------------------
    // --------------------Filtering-----------------------
    // -------------------- Crop Box ----------------------
    // ----------------------------------------------------

    // Create new PointCloud ---> cloudRegion
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    // Region = true to work with points inside the Crop Box
    pcl::CropBox<PointT> region(true);

    // Setting the arguments as the parameters of the function declaration
    region.setMin(minPoint);
    region.setMax(maxPoint);

    // Set the input data as the result of the voxel grid method
    region.setInputCloud(cloudFiltered);
    // save the result inside cloudRegion
    region.filter(*cloudRegion);

    // ----------------------------------------------------
    // --------------------Filtering-----------------------
    // ------ Removing points from the roof of the car ----
    // ----------------------------------------------------

    //Create a vector of ints    
    std::vector<int> indices;

    // Remove the roof point from the ego car
    // To know the exact dimension to remove roof points, plot a box that contains all the points
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Main_ProcessPcl<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    // pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


// template<typename PointT>
// pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> Main_ProcessPcl<PointT>::SegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = chrono::steady_clock::now();
// 	// PointIndices::Ptr inliers; // comment line because I used it in another part of the code.

//     // TODO:: Fill in the function to segment cloud into two parts, the driveable plane and obstacles
//     SACSegmentation<PointT> seg;
//     PointIndices::Ptr inliers {new PointIndices};
//     ModelCoefficients::Ptr coefficients {new ModelCoefficients};

//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(SACMODEL_PLANE);
//     seg.setMethodType(SAC_RANSAC);
//     seg.setMaxIterations(maxIterations);
//     seg.setDistanceThreshold(distanceThreshold);

//     seg.setInputCloud(cloud);
//     seg.segment(*inliers, *coefficients);
//     if(inliers->indices.size() == 0)
//     {
//         cout << "Could not estimate a planar model for the given dataset." << endl;
//     }
    
//     // TODO:: Fill in this function to find inliers for the cloud.

//     auto endTime = chrono::steady_clock::now();
//     auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
//     cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << endl;

//     pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
//     return segResult;
// }

// Trying to implement RANSAC. (Start)

// PointCloud<PointXYZ>::Ptr CreateData3D()
// {
// 	Main_ProcessPcl<PointXYZ> pointProcessor;
// 	return pointProcessor.loadPcd("../../../sensors/data/pcd/data1");

// 	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
//   	// Add inliers
//   	float scatter = 0.6;
//   	for(int i = -5; i < 5; i++)
//   	{
//   		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
//   		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
// 		double rz = 2*(((double) rand() / (RAND_MAX))-0.5);

//   		PointXYZ point;
//   		point.x = i+scatter*rx;
//   		point.y = i+scatter*ry;
//   		point.z = i+scatter*rz;

//   		cloud->points.push_back(point);
//   	}
//   	// Add outliers
//   	int numOutliers = 10;
//   	while(numOutliers--)
//   	{
//   		double rx = 2*(((double) rand() / (RAND_MAX))-0.01);
//   		double ry = 2*(((double) rand() / (RAND_MAX))-0.01);
// 		double rz = 2*(((double) rand() / (RAND_MAX))-0.01);

//   		PointXYZ point;
//   		point.x = 5*rx;
//   		point.y = 5*ry;
//   		point.z = 5*rz;

//   		cloud->points.push_back(point);

//   	}
//   	cloud->width = cloud->points.size();
//   	cloud->height = 0.5;

//   	return cloud;

// }

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Main_ProcessPcl<PointT>::MainRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{


    // A basic copy of the exercise ransac2d.cpp, modified to 3d.
	auto startTime = std::chrono::steady_clock::now();


	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
        std::cout << maxIterations << std::endl;
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) // x, y, z
			inliers.insert(rand()%(cloud->points.size()));
		
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float D = -(a*x1 + b*y1 + c*z1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index)>0)
				continue;

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs(a * x4 + b * y4 + c * z4 + D)/sqrt(a*a + b*b + c*c);

			if (d <= distanceTol)
				inliers.insert(index);
		}
    
		if (inliers.size()>inliersResult.size())
			inliersResult = inliers;
    }
        // Due to a problem that I had when I compiled, the first try was change PointXYZ to PointXYZI
        // Update: PointT (typename)
        typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

        for(int index = 0; index < cloud->points.size(); index++)
	    {
            PointT point = cloud->points[index];
            if(inliersResult.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
	    }
        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> newSegment(cloudOutliers,cloudInliers);
        return newSegment;
	
    
    // Not necessary right now.
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds> (endTime - startTime);
	std::cout << "Ransac took" <<elapsedTime.count() << " milliseconds" << std::endl;

	// return inliersResult;
}

// Trying to implement RANSAC. (End)

// Trying to implement Euclidean Cluster. (Start)


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Main_ProcessPcl<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    
    }

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// I'm a little bit confused. Would I follow the steps specifically for the Euclidean Cluster?
// ClusterHelper first?

template <typename PointT>
void Main_ProcessPcl<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{

	// vector<bool> processed(points.size(), false);



	processed[idx] = true;
	cluster.push_back(idx);

	std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
			clusterHelper (id, cloud, cluster, processed, tree, distanceTol);
	}
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Main_ProcessPcl<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol,int minSize,int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // Need review.
    std::vector<bool>processed(cloud->points.size(),false);

    // int i = 0;
	// while (i < points->size())
	// {
	// 	if (processed[i])
	// 	{
	// 		i++;
	// 		continue;
	// 	}

	// 	std::vector<int> cluster;
	// 	clusterHelper(i, points, cluster, processed, tree, distanceTol);
	// 	clusters.push_back(cluster);
	// 	i++;
		
	// }




	// return clusters;
    
    for(size_t idx=0;idx<cloud->points.size();++idx)
    {
        if(processed[idx]==false)
        {
            std::vector<int>cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
            clusterHelper(idx,cloud,cluster_idx,processed,tree,distanceTol);

            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
            {
                    for(int i=0;i< cluster_idx.size();i++)
                    {
                        PointT point;
                        point = cloud->points[cluster_idx[i]];
                        cloudCluster->points.push_back(point);
                    }
                    cloudCluster->width=cloudCluster->points.size();
                    cloudCluster->height=1;
                    clusters.push_back(cloudCluster);
            }
            else
            {
                for(int i;i<cluster_idx.size();i++)
                {
                    processed[cluster_idx[i]]=false;
                }
            }
            
        }
    }
        
    return clusters;
}



//Trying to implement Euclidean Cluster. (end)

template<typename PointT>
Box Main_ProcessPcl<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void Main_ProcessPcl<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr Main_ProcessPcl<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> Main_ProcessPcl<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}