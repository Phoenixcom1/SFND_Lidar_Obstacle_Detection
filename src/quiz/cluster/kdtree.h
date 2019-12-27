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
		std::cout << "node" << node << std::endl;
		std::cout << "dim_comp" << dim_comp << std::endl;

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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




