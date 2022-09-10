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
  
	//The double pointer is just the memory address of pointer node
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{  
  	  if(*node == NULL)
      {
      	*node = new Node(point, id);
      }
      else
      {
        //Calculate current dim, is the depth even or odd
        uint cd = depth % 2;
        if(point[cd] < ((*node)->point[cd]))
        {
          insertHelper(&((*node)->left), depth+1, point, id);
        }
		else
        {
          insertHelper(&((*node)->right), depth+1, point, id);          
        }
      }
    }
	void insert(std::vector<float> point, int id)
	{
	  // TODO: Fill in this function to insert a new point into the tree
	  // the function should create a new node and place correctly with in the root 
	  insertHelper(&root,0,point,id);
	}

  	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
      if(node != NULL)
      {
        //check if current node is within the target box
        if((node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) && (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)))
        {
          //check distance from target to 
          float distance = sqrt(pow(node->point[0]-target[0], 2.0) + pow(node->point[1]-target[1], 2.0));
          if(distance <= distanceTol)
          {
            ids.push_back(node->id);
          }
        }
        //check if we want to flow in the tree to the left or right of the boundary
        //check if box is to the left of the boundary
        if((target[depth%2]-distanceTol)<node->point[depth%2])
        {
          searchHelper(target, node->left, depth+1, distanceTol, ids);
        }
        //check if box is to the right of the boundary
        if((target[depth%2]+distanceTol)>node->point[depth%2])
        {
          searchHelper(target, node->right, depth+1, distanceTol, ids);
        }
      }
    }
	// return a list of point ids in the tree that are within distance of target
    //target is x,y
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
	  std::vector<int> ids;
      searchHelper(target, root, 0, distanceTol, ids);
	  return ids;
	}
	

};




