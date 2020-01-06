#include <cmath>
#include <pcl/common/common.h>


// Structure to represent node of kd tree
struct Node
{
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI arr, int setId)
            :	point(arr), id(setId), left(nullptr), right(nullptr)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
            : root(nullptr)
    {}

    ~KdTree()
    {
        deleteNode(root);
    }

    void deleteNode(Node* node)
    {
        if(nullptr != node)
        {
            deleteNode(node->left);
            deleteNode(node->right);
            delete node;
        }
    }


    void insert(pcl::PointXYZI point, int id)
    {
        insertHelper(root, 0, point,id);

    }

    //depth describes the dimension to compare against, x,y or z dimension represented by index 0,1 or 2 within the point vector and alternated on each layer of the tree
    void insertHelper(Node *&node, unsigned int depth, pcl::PointXYZI point, int id)
    {

        if(nullptr == node)
        {
            node = new Node(point, id);
        }
        else if(point.data[depth%3] < node->point.data[depth%3])
        {
            insertHelper(node->left, depth+1, point, id);
        }
        else
        {
            insertHelper(node->right, depth+1, point, id);
        }

    }

    //depth describes the dimension to compare against, x,y or z dimension represented by index 0,1 or 2 within the point vector and alternated on each layer of the tree
    void searchHelper(Node *&node, unsigned int depth, const pcl::PointXYZI& target, float distanceTol, std::vector<int>& ids)
    {
        if(nullptr == node)
        {
            return;
        }

        if(node->point.data[depth%3]-target.data[depth%3] > distanceTol)
        {
            searchHelper(node->left, depth+1, target, distanceTol, ids);
        }
        else if(node->point.data[depth%3]-target.data[depth%3] < -distanceTol)
        {
            searchHelper(node->right, depth+1, target, distanceTol, ids);
        }
        else
        {
            if(fabs(node->point.data[(depth+1)%3]-target.data[(depth+1)%3]) <= distanceTol && fabs(node->point.data[(depth+2)%3]-target.data[(depth+2)%3]) <= distanceTol)
            {
                double distance = sqrt(pow(node->point.data[0]-target.data[0], 2) + pow(node->point.data[1]-target.data[1], 2) + pow(node->point.data[2]-target.data[2], 2));
                if(distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            searchHelper(node->left, depth+1, target, distanceTol, ids);
            searchHelper(node->right, depth+1, target, distanceTol, ids);
        }


    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const pcl::PointXYZI& target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, 0, target, distanceTol, ids);

        return ids;
    }
};




