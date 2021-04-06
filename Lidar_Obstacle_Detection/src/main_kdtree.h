/* \author Aaron Brown
I created a new kdtree.h, because it seems that some modifications were necessary. So, here it is.
Below, some gits and Documentation that helped me to develop it. (Wasn't easy!)
Leonardo Citraro - https://github.com/lcit/KDtree/blob/master/KDtree.hpp
Junjie Dong - https://github.com/junjiedong/KDTree/blob/master/src/KDTree.h
PCL Docs - https://pointclouds.org/documentation/tutorials/kdtree_search.html
vision3dtech - https://vision3dtech.blogspot.com/2019/12/how-to-search-given-3d-point-and-its.html?m=1
*/

#include "render/render.h"

using namespace std;
using namespace pcl;


// Structure to represent node of kd tree --- The struct returned as "Ambiguous"
struct Node
{
	// std::vector<float> point;
	PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(PointXYZI arr, int setId):
		point(arr), id(setId), left(NULL), right(NULL)
	{}
	// I'm not seeing application to  ~Node()
	// ~Node()
	// {
	// 	delete left;
	// 	delete right;
	// }
};

struct KdTree
{
Node* root;

KdTree()
: root(NULL)
{}

void insertHelper(Node** node, uint depth, PointXYZI point, int id)
{

	if (*node == NULL)
		*node = new Node(point,id);
	else {
		int cd = depth % 2; // uint to int?
		// It seem that changes to PointXYZI are related to datatype.
		// Something must be implemented here !
		// Plausible solution: Following some repositories on github,
		// i need to declare that cd = 0, because it's the unique parameter that
		// confirm the starter root.
		if (cd == 0)
		{
			if (point.x < ((*node) -> point.x))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
		else{
		if (point.y < ((*node) -> point.y))
			insertHelper(&((*node)->left), depth + 1, point, id);
		else
			insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}
}
// ~KdTree()
// {
// 	delete root;
// }

void insert(PointXYZI point, int id)
{

	insertHelper(&root,0,point,id);
	
}

void searchHelper(PointXYZI target, Node* node, int depth, float distanceTol, vector<int>& ids)
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

vector<int> search(PointXYZI target, float distanceTol)
{
	vector<int> ids;
	searchHelper(target, root, 0, distanceTol, ids);

	return ids;
}
	

};



