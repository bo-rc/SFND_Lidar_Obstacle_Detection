/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <memory>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

template<size_t Dim>
struct KdTree
{
	std::unique_ptr<Node> root;

	KdTree()
	: root(nullptr)
	{}

	void _insert(std::unique_ptr<Node>& subroot, uint depth, std::vector<float> point, int id)
	{
		if (!subroot)
		{
			subroot = std::make_unique<Node>(point, id);
		}
		else
		{
			// current compare dimension
			uint cd = depth % Dim;
			if (point[id] < subroot->point[cd])
				_insert(subroot->left, depth+1, point, id);
			else
			{
				_insert(subroot->right, depth+1, point, id);
			}
		}

	}
	void insert(std::vector<float> point, int id)
	{
		// node, depth, point, id
		_insert(root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




