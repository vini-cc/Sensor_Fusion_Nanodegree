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
    cout << cloud->points.size() << endl;
}