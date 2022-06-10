#include "kdTree.hpp"

namespace LYPathTracer {
	ViewPoint::ViewPoint(glm::vec3 p, glm::vec3 n, glm::vec3 c, float stgh, int x, int y) {
		this->pos = p;
		this->normal = n;
		this->color = c;
		this->strength = stgh;
		this->x = x;
		this->y = y;
	}

	ViewPoint::ViewPoint(const ViewPoint& p) {
		this->pos = p.pos;
		this->normal = p.normal;
		this->color = p.color;
		this->strength = p.strength;
		this->x = p.x;
		this->y = p.y;
	}
}