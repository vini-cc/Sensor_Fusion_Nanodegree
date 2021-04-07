// PCL lib Functions for processing point clouds 

#ifndef MAINPROCESSPCL_H_
#define MAINPROCESSPCL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// #include "main_kdtree.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
// #include "kdtree.h"


struct Node
{
	// std::vector<float> point;
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
	// I'm not seeing application to  ~Node()
	// ~Node()
	// {
	// 	delete left;
	// 	delete right;
	// }
};
// --- The struct returned as "Ambiguous"
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{

		if (*node == NULL)
			*node = new Node(point,id);
		else {
			int cd = depth % 2; // uint to int?
			// It seem that changes to PointXYZI are related to datatype.
			// Something must be implemented here !
			// Plausible solution:
			// i need to declare that cd == 0, because it's the unique parameter that
			// confirm the starter root.
			if (cd == 0)
			{
				if (point.x < ((*node) -> point.x))
					insertHelper(&((*node) -> left), depth + 1, point, id);
				else
					insertHelper(&((*node) -> right), depth + 1, point, id);
			}
			else{
				if (point.y < ((*node) -> point.y))
					insertHelper(&((*node) -> left), depth + 1, point, id);
				else
					insertHelper(&((*node) -> right), depth + 1, point, id);
			}
		}
	}
	// ~MainKdTree()
	// {
	// 	delete root;
	// }

	void insert(pcl::PointXYZI point, int id)
	{
	
		insertHelper(&root,0,point,id);
		
	}

	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!= NULL)
		{
			// All [0] now is x, and all [1] now is y.
			if ((node -> point.x >= (target.x - distanceTol) && node -> point.x <= (target.x + distanceTol)) && (node -> point.y >= (target.y - distanceTol) && node -> point.y <= (target.y + distanceTol)))
			{
				float distance = sqrt (( node -> point.x - target.x) * (node -> point.x - target.x) + ( node -> point.y - target.y) * ( node -> point.y - target.y));
				if (distance <= distanceTol)
					ids.push_back(node -> id);
			}

			// Didn't understand why, but following the logic...
			if(depth % 2 == 0 )
			{
					if((target.x - distanceTol) < node->point.x)
						searchHelper(target,node->left,depth + 1,distanceTol,ids);
					if((target.x + distanceTol) > node->point.x)
						searchHelper(target,node->right,depth + 1,distanceTol,ids);
			}
			else
			{	
					if((target.y - distanceTol) < node->point.y)
						searchHelper(target,node->left,depth + 1,distanceTol,ids);
					if((target.y + distanceTol) > node->point.y)
						searchHelper(target,node->right,depth + 1,distanceTol,ids);
			}
		}
	}

	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
}
	

};


template<typename PointT>
class Main_ProcessPcl {
public:

    //constructor
    Main_ProcessPcl();
    //deconstructor
    ~Main_ProcessPcl();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    // **NEW**:
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> MainRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    // **NEW**:
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol,int minSize,int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* MAINPROCESSPCL_H_ */