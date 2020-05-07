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

	void insert(std::vector<float> point, int id)
	{
		real_insert(root, 0, point, id);
	}
	void real_insert(Node*& cur, int cur_depth, std::vector<float> point, int id)
	{
		if(cur==NULL)
		{
			cur = new Node(point, id);
		}
		else if(cur_depth%2==0)
		{
			int x = 0;
			if(cur->point[x] > point[x])
				real_insert(cur->left, cur_depth+1, point, id);
			else
				real_insert(cur->right, cur_depth+1, point, id);
			
		}
		else
		{
			int y = 1;
			if(cur->point[y] > point[y])
				real_insert(cur->left, cur_depth+1, point, id);
			else
				real_insert(cur->right, cur_depth+1, point, id);

		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root, 0, ids, target, distanceTol);
		return ids;
	}

	 void search_helper(Node* cur, int cur_depth, std::vector<int>& ids, std::vector<float> target, float distanceTol)
	{	
		if(cur!=NULL){

			uint xy = cur_depth%2;
			int x = 0; 
			int y = 1;
			float t1 = target[xy]-distanceTol;
			float t2 = target[xy]+distanceTol;
			if((cur->point[x] > (target[x]-distanceTol)) && (cur->point[x] < (target[x]+distanceTol)) && (cur->point[y] > (target[y]-distanceTol)) && (cur->point[y] < (target[y]+distanceTol)))
			{
				//in the box 
				float dx = target[x]-cur->point[x];
				float dy = target[y]-cur->point[y];
				float distance = sqrt(dx*dx+dy*dy);
				if(distance <= distanceTol)
					ids.push_back(cur->id);
			}
			if(t2 > cur->point[xy])
			{
				search_helper(cur->right, cur_depth+1, ids, target, distanceTol);
			}
			if(t1 < cur->point[xy])
			{
				search_helper(cur->left, cur_depth+1, ids, target, distanceTol);
			}
		}
		
	}
	

};




