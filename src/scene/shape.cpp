
#include "shape.h"
#include "../geometry/util.h"
#include <iostream>
namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
	PT::Trace ret;
	float a = dot(ray.dir, ray.dir);
	float b = 2.0f * dot(ray.point, ray.dir);
	float c = dot(ray.point, ray.point) - radius * radius;

	float determinant = b * b - 4 * a * c;
	if (determinant < 0.0f) { // no solution
		ret.hit = false;
		return ret;
	}
	float t1 = (-b - sqrtf(determinant)) / (2 * a);
	float t2 = (-b + sqrtf(determinant)) / (2 * a);

	float t = 0.0f;
	// check if t1 and t2 are in bounds 
	if (t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y) {
		t = t1;
	}
	else if (t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y) {
		t = t2;
	}
	else {
		// both intersections out of bounds
		ret.hit = false;
		return ret;
	}
	ret.origin = ray.point;
	ret.hit = true;       // was there an intersection?
	ret.distance = t;   // at what distance did the intersection occur?
	ret.position = ray.at(t); // where was the intersection?
	ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
	ret.uv = uv(ret.position); 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
	return ret;
	
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
