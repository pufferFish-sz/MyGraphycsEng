
#include "samplers.h"
#include "../util/rand.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling

    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()
	float randx = rng.unit();
	float randy = rng.unit();

    return Vec2(randx * size.x, randy * size.y);
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec2 Circle::sample(RNG &rng) const {
	//A3EC - bokeh - circle sampling

    // Return a point selected uniformly at random from a circle defined by its
	// center and radius.
    // Useful function: rng.unit()

    return Vec2{};
}

float Circle::pdf(Vec2 at) const {
	//A3EC - bokeh - circle pdf

	// Return the pdf of sampling the point 'at' for a circle defined by its
	// center and radius.

    return 1.f;
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float cos_theta = 2.0f * Xi1 - 1.0f;
	float sin_theta = sqrtf(1.0f - cos_theta * cos_theta);
	float phi = 2.0f * PI_F * Xi2;

	float xs = sin_theta * std::cos(phi);
	float ys = cos_theta;
	float zs = sin_theta * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
	//initialize 
	_pdf = std::vector<float>(w * h, 0.0f);
	_cdf = std::vector<float>(w * h, 0.0f);

	float total = 0.0f;
	for (uint32_t y = 0; y < h; ++y) {
		for (uint32_t x = 0; x < w; ++x) {
			uint32_t i = y * w + x;
			float flux = image.at(i).luma(); // luminance for each pixel

			float theta = PI_F * (y + 0.5f) / h;
			flux *= sin(theta);

			_pdf[i] = flux;
			total += flux;

			if (i == 0) {
				_cdf[i] = _pdf[i];
			}
			else {
				_cdf[i] = _cdf[i - 1] + _pdf[i];
			}
		}
	}

	for (uint32_t i = 0; i < w * h; ++i) {
		_pdf[i] /= total;
		_cdf[i] /= total;
	}
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
		Sphere::Uniform u;
		return u.sample(rng);
	} else {
		// Step 2: Importance sampling
		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
		float p = rng.unit();
		auto i = std::upper_bound(_cdf.begin(), _cdf.end(), p);
		
		uint32_t index = (uint32_t)std::distance(_cdf.begin(), i);
		
		uint32_t y = index / w;
		uint32_t x = index % w;

		float theta = PI_F * (h - y + 0.5f) / h;
		float phi = 2.0f * PI_F * (x + 0.5f) / w;

		// convert spherical to cartesian
		float sin_theta = std::sin(theta);
		float xs = sin_theta * std::cos(phi);
		float ys = std::cos(theta);
		float zs = sin_theta * std::sin(phi);

    	return Vec3(xs, ys, zs);
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		Sphere::Uniform u;
		return u.pdf(dir);
	} else {
		// A3T7 - image sampler importance sampling pdf
		// What is the PDF of this distribution at a particular direction?
		float theta = acosf(dir.y);
		float phi = atan2f(dir.z, dir.x);
		if (phi < 0) phi += 2 * PI_F;

		uint32_t y = (uint32_t)(theta * h / PI_F);
		uint32_t x = (uint32_t)(phi * w / (2 * PI_F));

		uint32_t i = y * w + x;

		float Jacobian = (w * h) / (2 * PI_F * PI_F * sin(theta));
		return _pdf[i] * Jacobian;
	}
}

} // namespace Samplers
