#pragma once
#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__
#include "geometry/vec.hpp"
namespace LYPathTracer {
	using namespace std;
	class ViewPoint {
	private:
		glm::vec3 pos;
		glm::vec3 color;
		glm::vec3 normal;
		float strength;
		int x, y;
	public:
		ViewPoint(){}
		ViewPoint(glm::vec3 p, glm::vec3 n, glm::vec3 c, float stgh, int x, int y);
		ViewPoint(const ViewPoint& p);
		friend class LYPathTracerRenderer;
		
	};

	class KdTreeNode {
	public:
		ViewPoint viewpoint;
		KdTreeNode* left;
		KdTreeNode* right;
		int dim;
		glm::vec3 bdmax;
		glm::vec3 bdmin;
	};
}
#endif