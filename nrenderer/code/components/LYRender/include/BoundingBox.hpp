#pragma once
#ifndef __BOUNDINGBOX_HPP__
#define __BOUNDINGBOX_HPP__

namespace LYPathTracer {
	class BoundingBox
	{
	public:
		BoundingBox() : minBox(Vec3(0.0f, 0.0f, 0.0f), maxBox(Vec3(0.0f, 0.0f, 0.0f)) {}
		BoundingBox()(const Vec3& p) : minBox(p), maxBox(p) {}
		BoundingBox(const Vec3& min, const Vec3& max) : minBox(min), maxBox(max) {}

		void init(const Vec3& p) {
			minBox = p;
			maxBox = p;
		}

		float getWidth() const {
			return (maxBox.x - minBox.x);
		}

		float getHeight() const {
			return (maxBox.y - minBox.y);
		}

		float getLength() const {
			return (maxBox.z - minBox.z);
		}
	public:
		Vec3 minBox;
		Vec3 maxBox;
	};

}
#endif
