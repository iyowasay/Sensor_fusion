/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <memory>

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

	void insert_helper(std::vector<float> point, int id, Node** cur_node, int cur_depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		if(*cur_node==NULL)
		{
			//std::shared_ptr<Node> nod = std::make_shared<Node>(point, id);
			//Node* nod = new Node(point, id);
			*cur_node = new Node(point, id);
		}
		else
		{
			uint xy = cur_depth%2;
			if((*cur_node)->point[xy] > point[xy])	
				insert_helper(point, id, &((*cur_node)->left), cur_depth+1);
			else
				insert_helper(point, id, &((*cur_node)->right), cur_depth+1);
		}


	}

	//The order of these functions matters!! Otherwise declare it at the begining
	void insert(std::vector<float> point, int id)
	{
		insert_helper(point, id, &root, 0);
	}

	
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, distanceTol, root, 0,ids);
		return ids;
	}
	// return a list of point ids in the tree that are within distance of target
	void search_helper(std::vector<float> target, float distanceTol, Node* cur_node, int cur_depth,std::vector<int>& ids)
	{
		if(cur_node!= NULL)
		{
			uint xy = cur_depth%2; // index of divide direction, use x or y to compare 
			int x = 0, y = 1;
			float cur_x = cur_node->point[x];
			float cur_y = cur_node->point[y];
			
			if(cur_x <= target[x]+distanceTol && cur_x >= target[x]-distanceTol && cur_y <= target[y]+distanceTol && cur_y >= target[y]-distanceTol)
			{
				float dx = target[x]-cur_x;
				float dy = target[y]-cur_y;
				float dis = sqrt(dx*dx+dy*dy);
				if(dis <= distanceTol)
				{
					ids.push_back(cur_node->id);
				}
			}

			float t1 = target[xy]-distanceTol;
			float t2 = target[xy]+distanceTol;
			cur_depth++;
			if(t1 < cur_node->point[xy])
				search_helper(target, distanceTol, cur_node->left, cur_depth, ids);
			if(t2 > cur_node->point[xy])
				search_helper(target, distanceTol, cur_node->right, cur_depth, ids);
		}

		return;
	}



};










