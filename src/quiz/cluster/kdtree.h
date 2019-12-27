/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point,id);

	}

	//dim_comp describes the dimension to compare agains, x or y dimension represented by index 0 or 1 within the point vector
	void insertHelper(Node *&node, unsigned char dim_comp, std::vector<float> point, int id)
	{

		if(NULL == node)
		{
			node = new Node(point, id);
		}
		else if(point[dim_comp] < node->point[dim_comp])
		{
			insertHelper(node->left, !dim_comp, point, id);
		}
		else
		{
			insertHelper(node->right, !dim_comp, point, id);
		}

	}
	
	//dim_comp describes the dimension to compare agains, x or y dimension represented by index 0 or 1 within the point vector
	void searchHelper(Node *&node, unsigned char dim_comp, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{

		if(NULL != node)
		{

			//reference point is within box on one dimension
			if(node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)
				&& node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol) )
			{
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0])
									+ (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if(distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}


			if((target[dim_comp]-distanceTol) < node->point[dim_comp])
			{
				searchHelper(node->left, !dim_comp, target, distanceTol, ids);
			}
			if((target[dim_comp]+distanceTol) > node->point[dim_comp])
			{
				searchHelper(node->right, !dim_comp, target, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
};




