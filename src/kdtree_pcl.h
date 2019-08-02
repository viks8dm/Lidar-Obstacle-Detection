// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertNode(Node **node, int depth, pcl::PointXYZI point, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		else {
			int cd = depth % 3;

			if (cd==0) {
				if (point.x < (*node)->point.x)
					insertNode(&((*node)->left), depth+1, point, id);
				else
					insertNode(&((*node)->right), depth+1, point, id);
			}
			else if (cd ==1) {
				if (point.y < (*node)->point.y)
					insertNode(&((*node)->left), depth+1, point, id);
				else
					insertNode(&((*node)->right), depth+1, point, id);
			}
			else {
				if (point.z < (*node)->point.z)
					insertNode(&((*node)->left), depth+1, point, id);
				else
					insertNode(&((*node)->right), depth+1, point, id);
			}
		}
	}


	void insert(pcl::PointXYZI point, int id)
	{
		// the function should create a new node and place correctly with in the root
		insertNode(&root, 0, point, id);

	}

	/************************************************/
	void searchForKDTreeNeighbors(std::vector<int> &ids, pcl::PointXYZI target,
				float distanceTol, Node* parent_node, int depth) {

		if (parent_node != NULL) {
			float delta_x = parent_node->point.x - target.x;
			float delta_y = parent_node->point.y - target.y;
			float delta_z = parent_node->point.z - target.z;

			if ((-distanceTol<=delta_x && distanceTol>=delta_x) &&
								(-distanceTol<=delta_y && distanceTol>=delta_y) &&
									(-distanceTol<=delta_z && distanceTol>=delta_z)) {
				float distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
				// check distance
				if (distance<=distanceTol) {
					ids.push_back(parent_node->id);
				}
			}

			if (depth % 3 == 0) // 3 dim kd-tree - x-axis
			{
				if (-distanceTol < delta_x)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->left, depth+1);
				if (distanceTol > delta_x)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->right, depth+1);
			}
			else if (depth % 3 == 1) // y-axis
			{
				if (-distanceTol < delta_y)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->left, depth+1);
				if (distanceTol > delta_y)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->right, depth+1);
			}
			else // z-axis
			{
				if (-distanceTol < delta_z)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->left, depth+1);
				if (distanceTol > delta_z)
					searchForKDTreeNeighbors(ids, target, distanceTol, parent_node->right, depth+1);
			}

		}
	}
	/************************************************/

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol) {
		std::vector<int> ids;
		searchForKDTreeNeighbors(ids, target, distanceTol, root, 0);
		return ids;
	}


};
