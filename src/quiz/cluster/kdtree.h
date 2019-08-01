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

	void insertNode(Node **node, uint depth, std::vector<float> point, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		else {
			uint cd = depth % 2;

			if (point[cd] < ((*node)->point[cd])) {
				insertNode(&((*node)->left), depth+1, point, id);
			}
			else {
				insertNode(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertNode(&root, 0, point, id);

	}

/************************************************/
	void searchForKDTreeNeighbors(std::vector<int> &ids, std::vector<float> target,
				float distanceTol, Node* parent_node, int depth) {

		if (parent_node != NULL) {
			float delta_x = parent_node->point[0] - target[0];
			float delta_y = parent_node->point[1] - target[1];

			if (-distanceTol<=delta_x && distanceTol>=delta_x && -distanceTol<=delta_y && distanceTol>=delta_y) {
				float distance = sqrt(delta_x*delta_x + delta_y*delta_y);
				if (distance<=distanceTol) {
					ids.push_back(parent_node->id);
				}
			}

			//
			int lr_id = depth % 2;
			if ((target[lr_id]-distanceTol) < parent_node->point[lr_id]) {
				searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->left, depth+1);
			}
			if ((target[lr_id]+distanceTol) > parent_node->point[lr_id]) {
				searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->right, depth+1);
			}

		}
	}
/************************************************/

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchForKDTreeNeighbors(ids, target, distanceTol, root, 0);
		return ids;
	}


};
