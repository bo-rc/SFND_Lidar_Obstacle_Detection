/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <memory>
#include <numeric>
#include <functional>
#include <cmath>
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
		if (subroot == nullptr)
		{
			std::cout << "inserting new node: " << id << 
			" depth: " << depth << " point:[" << point[0] << "," << point[1] << "] " << std::endl;
			subroot = std::make_unique<Node>(point, id);
		}
		else
		{
			// current compare dimension
			uint cd = depth % Dim;
			if (point[cd] < subroot->point[cd])
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

	bool _insideBoundary(const std::vector<float>& point, const std::vector<float>& target, float distanceTol) const
	{
		bool res = true;
		for (size_t dim = 0; dim < target.size(); ++dim)
		{
			if ( (point[dim] < target[dim] - distanceTol) or (point[dim] > target[dim] + distanceTol) )
			{
				res = false;
				break;
			}
		}
		return res;
	}

	// same as insideBoundary but using algorithm
	bool _insideBox(const std::vector<float>& point, const std::vector<float>& target, float distanceTol) const
	{
		return std::inner_product(point.cbegin(), point.cend(), target.cbegin(), true,
			[](bool subSum, bool newValue){
				return subSum && newValue;
			},
			[&](float p, float t){
				return (p >= t - distanceTol) && (p <= t + distanceTol);
			});
	}

	float _distance(const std::vector<float>& p1, const std::vector<float>& p2) const
	{
		float squaredSum = std::inner_product(p1.cbegin(), p1.cend(), p2.cbegin(), 0.0,
			std::plus<>(),
			[](float p1x, float p2x)
			{
				return std::pow(p1x - p2x, 2);
			});
		return std::sqrt(squaredSum);
	}

	bool _outLeft(const std::vector<float>& point, const std::vector<float>& target, int depth, float distanceTol) const
	{	
		// needs a safety margin tolerance because the point might be within 
		// both left and right node although it is outside the subroot node
		return target[depth%Dim] - distanceTol < point[depth%Dim];
	}

	bool _outRight(const std::vector<float>& point, const std::vector<float>& target, int depth, float distanceTol) const
	{	
		// needs a safety margin tolerance because the point might be within 
		// both left and right node although it is outside the subroot node
		return target[depth%Dim] + distanceTol > point[depth%Dim];
	}

	void _search(const std::vector<float>& target, const std::unique_ptr<Node>& node, int depth, float distanceTol, std::vector<int>& ids) const
	{
		if (node)
		{
			std::cout << "target: [" << target[0] << "," << target[1] << "] -> search node " 
			<< node->id << " point [" << node->point[0] << "," << node->point[1] << "] " << std::endl;
			
			if ( _insideBox(node->point, target, distanceTol) )
			{
				std::cout << "	inside boundary of id: " << node->id << std::endl;
				if (_distance(node->point, target) <= distanceTol)
					ids.push_back(node->id);
			}
			
			if (_outLeft(node->point, target, depth, distanceTol))
			{
				_search(target, node->left, depth+1, distanceTol, ids);
			}
			
			if (_outRight(node->point, target, depth, distanceTol))
			{
				_search(target, node->right, depth+1, distanceTol, ids);
			}
			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol) const
	{
		std::vector<int> ids;
		_search(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




