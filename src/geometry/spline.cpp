
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...

	return cubic_unit_spline(0.0f, T(), T(), T(), T());
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

	T hermite = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;

	return hermite;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
