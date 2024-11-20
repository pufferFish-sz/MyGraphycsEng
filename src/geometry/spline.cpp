
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...
	
	auto initial = knots.begin();
	auto last = std::prev(knots.end());
	// the edge cases:
	if (knots.empty()) {
		return T();
	}
	else if (knots.size() == 1) {
		return initial->second;
	}
	else if (time <= initial->first) {
		return initial->second;
	}
	else if (time >= last->first) {
		return last->second;
	}

	// guarenteed 2 or more knots
	auto k2 = knots.upper_bound(time);
	auto k1 = std::prev(k2);

	T p1 = k1->second;
	T p2 = k2->second;
	float t1 = k1->first;
	float t2 = k2->first;

	T p0;
	float t0;
	float time_interval = t2 - t1;

	if (k1->first == initial->first) {
		//need to find virtual knot using mirroring
		t0 = t1 - (t2 - t1);
		p0 = p1 - (p2 - p1);
	}
	else {
		auto k0 = std::prev(k1);
		t0 = k0->first;
		p0 = k0->second;
	}

	T p3;
	float t3;
	if (k2->first == last->first) {
		//need to find virtual knot using mirroring
		t3 = t2 + (t2 - t1);
		p3 = p2 + (p2 - p1);
	}
	else {
		auto k3 = std::next(k2);
		t3 = k3->first;
		p3 = k3->second;
	}

	// then need to find tangents

	T m0 = (p2 - p0) / (t2 - t0) * time_interval;

	T m1 = (p3 - p1) / (t3 - t1) * time_interval;

	float normalized_time = (time - t1) / time_interval;

	return cubic_unit_spline(normalized_time, p1, p2,m0, m1);
}

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.

	float time_cubed = powf(time, 3);
	float time_squared = time * time;
	float h00 = 2 * time_cubed - 3 * time_squared + 1;
	float h10 = time_cubed - 2 * time_squared + time;
	float h01 = -2 * time_cubed + 3 * time_squared;
	float h11 = time_cubed - time_squared;

	return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
