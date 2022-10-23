#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>


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


template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
  
	//The double pointer is just the memory address of pointer node
	void insertHelper(Node** node, uint depth, PointT point, int id)
	{
      uint axis = depth%3;
      //If node is Null insert point creating a new node.
  	  if(*node == NULL)
      {
        std::vector<float> point_vector(point.data, point.data+3);
      	*node = new Node(point_vector, id);
      }
      else if(point.data[axis] < ((*node)->point[axis]))
      {
        insertHelper(&((*node)->left), depth+1, point, id);
      }
      else
      {
        insertHelper(&((*node)->right), depth+1, point, id);       
      }
    }

  void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    for(uint id = 0; id < cloud->points.size(); id++)
    {
      insertHelper(&root, 0, cloud->points[id], id);
    }
  }
  
  void insert(std::vector<float> point, int id)
	{
	  // TODO: Fill in this function to insert a new point into the tree
	  // the function should create a new node and place correctly with in the root 
	  insertHelper(&root,0,point,id);
	}

  	void searchHelper(PointT target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
      uint id = depth%3;
      
      if(node != NULL)
      {
        //check if current node is within the target box
        if((node->point[0]>=(target.data[0]-distanceTol) && node->point[0]<=(target.data[0]+distanceTol)) && (node->point[1]>=(target.data[1]-distanceTol) && node->point[1]<=(target.data[1]+distanceTol)) && (node->point[2]>=(target.data[2]-distanceTol) && node->point[2]<=(target.data[2]+distanceTol)))
        {
          //check distance from target to 
          float distance = sqrt(pow(node->point[0]-target.data[0], 2.0) + pow(node->point[1]-target.data[1], 2.0) + pow(node->point[2]-target.data[2], 2.0));
          if(distance <= distanceTol)
          {
            ids.push_back(node->id);
          }
        }
        //check if we want to flow in the tree to the left or right of the boundary
        //check if box is to the left of the boundary
        if((target.data[id]-distanceTol)<node->point[id])
        {
          searchHelper(target, node->left, depth+1, distanceTol, ids);
        }
        //check if box is to the right of the boundary
        if((target.data[id]+distanceTol)>node->point[id])
        {
          searchHelper(target, node->right, depth+1, distanceTol, ids);
        }
      }
    }
	// return a list of point ids in the tree that are within distance of target
    //target is x,y
	std::vector<int> search(PointT target, float distanceTol)
	{
	  std::vector<int> ids;
      int depth = 0;
      searchHelper(target, root, depth, distanceTol, ids);
	  return ids;
	}
};

