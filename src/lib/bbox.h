
#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <ostream>
#include <vector>

#include "mat4.h"
#include "ray.h"
#include "vec2.h"
#include "vec3.h"
#include <iostream>

struct BBox {

	/// Default min is max float value, default max is negative max float value
	BBox() : min(FLT_MAX), max(-FLT_MAX) {
	}
	/// Set minimum and maximum extent
	explicit BBox(Vec3 min, Vec3 max) : min(min), max(max) {
	}

	BBox(const BBox&) = default;
	BBox& operator=(const BBox&) = default;
	~BBox() = default;

	/// Rest min to max float, max to negative max float
	void reset() {
		min = Vec3(FLT_MAX);
		max = Vec3(-FLT_MAX);
	}

	/// Expand bounding box to include point
	void enclose(Vec3 point) {
		min = hmin(min, point);
		max = hmax(max, point);
	}
	void enclose(BBox box) {
		min = hmin(min, box.min);
		max = hmax(max, box.max);
	}

	/// Get center point of box
	Vec3 center() const {
		return (min + max) * 0.5f;
	}

	// Check whether box has no volume
	bool empty() const {
		return min.x > max.x || min.y > max.y || min.z > max.z;
	}

	/// Get surface area of the box
	float surface_area() const {
		if (empty()) return 0.0f;
		Vec3 extent = max - min;
		return 2.0f * (extent.x * extent.z + extent.x * extent.y + extent.y * extent.z);
	}

	/// Transform box by a matrix
	BBox& transform(const Mat4& trans) {
		Vec3 amin = min, amax = max;
		min = max = trans[3].xyz();
		for (uint32_t i = 0; i < 3; i++) {
			for (uint32_t j = 0; j < 3; j++) {
				float a = trans[j][i] * amin[j];
				float b = trans[j][i] * amax[j];
				if (a < b) {
					min[i] += a;
					max[i] += b;
				}
				else {
					min[i] += b;
					max[i] += a;
				}
			}
		}
		return *this;
	}

	bool hit(const Ray& ray, Vec2& times) const {
		//A3T3 - bbox hit

		// Implement ray - bounding box intersection test
		// If the ray intersected the bounding box within the range given by
		// [times.x,times.y], update times with the new intersection times.
		// This means at least one of tmin and tmax must be within the range

		float tmin, tmax, tymin, tymax, tzmin, tzmax;

		Vec3 invdir = 1.0f / ray.dir;
		tmin = (min.x - ray.point.x) * invdir.x;
		tmax = (max.x - ray.point.x) * invdir.x;
		if (tmin > tmax) std::swap(tmin, tmax);

		tymin = (min.y - ray.point.y) * invdir.y;
		tymax = (max.y - ray.point.y) * invdir.y;
		if (tymin > tymax) std::swap(tymin, tymax);

		if ((tmin > tymax) || (tymin > tmax)){
			//std::cout << "Miss on Y-axis bounds check." << std::endl;
			return false;
		}

		if (tymin > tmin) {
			tmin = tymin;
		}
		if (tymax < tmax) {
			tmax = tymax;
		}

		tzmin = (min.z - ray.point.z) * invdir.z;
		tzmax = (max.z - ray.point.z) * invdir.z;
		if (tzmin > tzmax) std::swap(tzmin, tzmax);

		if ((tmin > tzmax) || (tzmin > tmax)) {
			//std::cout << "Miss on Z-axis bounds check." << std::endl;
			return false;
		}

		if (tzmin > tmin) {
			tmin = tzmin;
		}
		if (tzmax < tmax) {
			tmax = tzmax;
		}

		// Check for overlap with the input range [times.x, times.y]
		float new_tmin = std::max(tmin, times.x);
		float new_tmax = std::min(tmax, times.y);

		if (new_tmin > new_tmax) {
			//std::cout << "No intersection within range." << std::endl;
			return false;  // No intersection within the desired range
		}

		// Update times with the intersection interval
		times.x = new_tmin;
		times.y = new_tmax;
		//std::cout << "Intersection found: tmin=" << new_tmin << ", tmax=" << new_tmax << std::endl;
		return true;
	}

	/// Get the eight corner points of the bounding box
	std::vector<Vec3> corners() const {
		std::vector<Vec3> ret(8);
		ret[0] = Vec3(min.x, min.y, min.z);
		ret[1] = Vec3(max.x, min.y, min.z);
		ret[2] = Vec3(min.x, max.y, min.z);
		ret[3] = Vec3(min.x, min.y, max.z);
		ret[4] = Vec3(max.x, max.y, min.z);
		ret[5] = Vec3(min.x, max.y, max.z);
		ret[6] = Vec3(max.x, min.y, max.z);
		ret[7] = Vec3(max.x, max.y, max.z);
		return ret;
	}

	/// Given a screen transformation (projection), calculate screen-space ([-1,1]x[-1,1])
	/// bounds that will always contain the bounding box on screen
	void screen_rect(const Mat4& transform, Vec2& min_out, Vec2& max_out) const {

		min_out = Vec2(FLT_MAX);
		max_out = Vec2(-FLT_MAX);
		auto c = corners();
		bool partially_behind = false, all_behind = true;
		for (auto& v : c) {
			Vec3 p = transform * v;
			if (p.z < 0) {
				partially_behind = true;
			} else {
				all_behind = false;
			}
			min_out = hmin(min_out, Vec2(p.x, p.y));
			max_out = hmax(max_out, Vec2(p.x, p.y));
		}

		if (partially_behind && !all_behind) {
			min_out = Vec2(-1.0f, -1.0f);
			max_out = Vec2(1.0f, 1.0f);
		} else if (all_behind) {
			min_out = Vec2(0.0f, 0.0f);
			max_out = Vec2(0.0f, 0.0f);
		}
	}

	Vec3 min, max;
};

inline std::ostream& operator<<(std::ostream& out, BBox b) {
	out << "BBox{" << b.min << "," << b.max << "}";
	return out;
}
