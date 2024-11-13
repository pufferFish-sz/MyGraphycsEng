
#include "material.h"
#include "../util/rand.h"

namespace Materials {

Vec3 reflect(Vec3 dir) {
	//A3T5 Materials - reflect helper

    // Return direction to incoming light that would be
	// reflected out in direction dir from surface
	// with normal (0,1,0)
	Vec3 normal(0, 1, 0);
	Vec3 reflected = -dir + 2 * dot(dir, normal) * normal;
    return reflected;
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {
	//A3T5 Materials - refract helper

	// Use Snell's Law to refract out_dir through the surface.
	// Return the refracted direction. Set was_internal to true if
	// refraction does not occur due to total internal reflection,
	// and false otherwise.

	// The surface normal is (0,1,0)
	if (index_of_refraction == 1.0f) {
		was_internal = false;
		return -out_dir;
	}

	Vec3 normal(0, 1, 0);

	float cos_theta_t = dot(out_dir, normal);

	float eta_t = 1.0f; // vacuum
	float eta_i = index_of_refraction;
	if (cos_theta_t < 0) { // ray is exiting
		std::swap(eta_i, eta_t);
		normal = -normal;
		cos_theta_t = -cos_theta_t;
	}

	float eta = eta_t / eta_i;

	//using Snell's Law
	float sin_theta_i_squared = eta * eta * (1.0f - cos_theta_t * cos_theta_t);

	//check for total internal reflection
	if (sin_theta_i_squared > 1.0f) {
		was_internal = true;
		return Vec3(0.0f, 0.0f, 0.0f);
	}
	was_internal = false;

	float cos_theta_i = sqrtf(1.0f - sin_theta_i_squared);

	Vec3 refracted = eta * (-out_dir) + (eta * cos_theta_t - cos_theta_i) * normal;

	return refracted;
}

float schlick(Vec3 in_dir, float index_of_refraction) {
	//A3T5 Materials - Schlick's approximation helper

	// Implement Schlick's approximation of the Fresnel reflection factor.
	float n1 = 1.0f;
	float n2 = index_of_refraction;

	Vec3 normal(0, 1, 0);
	float cos_theta = dot(normal, in_dir);

	if (cos_theta < 0) {
		std::swap(n1, n2);
		cos_theta = -cos_theta;
	}

	float R0_ratio = (n1 - n2) / (n1 + n2);
	float R0 = R0_ratio * R0_ratio;

	float ret = R0 + (1 - R0) * powf((1 - cos_theta), 5);
	return ret;
}

Spectrum Lambertian::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF evaluation

    // Compute the ratio of outgoing/incoming radiance when light from in_dir
    // is reflected through out_dir: (albedo / PI_F) * cos(theta).
    // Note that for Scotty3D, y is the 'up' direction.
	Vec3 normal(0, 1, 0);
	float cos_theta = dot(normal, in);

	// ensure cos_theta non-negative
	if (cos_theta <= 0) return Spectrum(0.0f);

    return (albedo.lock()->evaluate(uv) / PI_F);
}

Scatter Lambertian::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF scattering
	//Select a scattered light direction at random from the Lambertian BSDF

	Samplers::Hemisphere::Cosine sampler; //this will be useful

	Scatter ret;
	//TODO: sample the direction the light was scatter from from a cosine-weighted hemisphere distribution:
	ret.direction = sampler.sample(rng);

	//TODO: compute the attenuation of the light using Lambertian::evaluate():
	ret.attenuation = evaluate(out, ret.direction, uv);

	return ret;
}

float Lambertian::pdf(Vec3 out, Vec3 in) const {
	//A3T4: Materials - Lambertian BSDF probability density function
    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
	[[maybe_unused]] Samplers::Hemisphere::Cosine sampler; //this might be handy!
	Vec3 normal(0, 1, 0);

	return sampler.pdf(normal);
}

Spectrum Lambertian::emission(Vec2 uv) const {
	return {};
}

std::weak_ptr<Texture> Lambertian::display() const {
	return albedo;
}

void Lambertian::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(albedo);
}

Spectrum Mirror::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Mirror::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5: mirror

	// Use reflect to compute the new direction
	// Don't forget that this is a discrete material!
	// Similar to albedo, reflectance represents the ratio of incoming light to reflected light

    Scatter ret;
    ret.direction = reflect(out);
    ret.attenuation = reflectance.lock()->evaluate(uv);
    return ret;
}

float Mirror::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Mirror::emission(Vec2 uv) const {
	return {};
}

std::weak_ptr<Texture> Mirror::display() const {
	return reflectance;
}

void Mirror::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
}

Spectrum Refract::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Refract::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - refract

	// Use refract to determine the new direction - what happens in the total internal reflection case?
    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
	// Don't forget that this is a discrete material!
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation
	//  You do not need to scale by the Fresnel Coefficient - you'll only need to account for the correct ratio of indices of refraction

    Scatter ret;
	bool was_internal;
    ret.direction = refract(out,ior,was_internal);

	if (was_internal) {
		ret.direction = reflect(-out);
		ret.attenuation = transmittance.lock()->evaluate(uv); // no attenuation if total internal reflection
		return ret;
	}

	Vec3 normal(0, 1, 0);
	float cos_theta_t = dot(out, normal);

	float eta_t = 1.0f; // vacuum
	float eta_i = ior;
	if (cos_theta_t < 0) { // ray is exiting the medium
		std::swap(eta_i, eta_t);
	}

	float eta = eta_t / eta_i;
    ret.attenuation = (transmittance.lock()->evaluate(uv)) * (eta * eta);
    return ret;
}

float Refract::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Refract::emission(Vec2 uv) const {
	return {};
}

bool Refract::is_emissive() const {
	return false;
}

bool Refract::is_specular() const {
	return true;
}

bool Refract::is_sided() const {
	return true;
}

std::weak_ptr<Texture> Refract::display() const {
	return transmittance;
}

void Refract::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(transmittance);
}

Spectrum Glass::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Glass::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - glass

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
    // When debugging Glass, it may be useful to compare to a pure-refraction BSDF
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation
	//  You do not need to scale by the Fresnel Coefficient - you'll only need to account for the correct ratio of indices of refraction
	Scatter ret;
	Vec3 normal(0, 1, 0);
	bool was_internal;
	float cos_theta_t = dot(normal, out);

	float eta_t = 1.0f; // vacuum
	float eta_i = ior;
	if (cos_theta_t < 0) { // ray is exiting medium
		std::swap(eta_i, eta_t);
	}

	float eta = eta_t / eta_i;

	float fresnel_reflectance = schlick(out, ior);

	if (rng.coin_flip(fresnel_reflectance)) {
		ret.direction = reflect(out);
		ret.attenuation = reflectance.lock()->evaluate(uv);
	}
	else {
		// refract, total internal reflection
		ret.direction = refract(out, ior, was_internal);

		if (was_internal) {
			ret.direction = reflect(-out);
			ret.attenuation = transmittance.lock()->evaluate(uv);
		}
		else {
			ret.attenuation = transmittance.lock()->evaluate(uv) * (eta * eta);
		}
	}
    return ret;
}

float Glass::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Glass::emission(Vec2 uv) const {
	return {};
}

bool Glass::is_emissive() const {
	return false;
}

bool Glass::is_specular() const {
	return true;
}

bool Glass::is_sided() const {
	return true;
}

std::weak_ptr<Texture> Glass::display() const {
	return transmittance;
}

void Glass::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
	f(transmittance);
}

Spectrum Emissive::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Emissive::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	Scatter ret;
	ret.direction = {};
	ret.attenuation = {};
	return ret;
}

float Emissive::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Emissive::emission(Vec2 uv) const {
	return emissive.lock()->evaluate(uv);
}

bool Emissive::is_emissive() const {
	return true;
}

bool Emissive::is_specular() const {
	return true;
}

bool Emissive::is_sided() const {
	return false;
}

std::weak_ptr<Texture> Emissive::display() const {
	return emissive;
}

void Emissive::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(emissive);
}

} // namespace Materials

bool operator!=(const Materials::Lambertian& a, const Materials::Lambertian& b) {
	return a.albedo.lock() != b.albedo.lock();
}

bool operator!=(const Materials::Mirror& a, const Materials::Mirror& b) {
	return a.reflectance.lock() != b.reflectance.lock();
}

bool operator!=(const Materials::Refract& a, const Materials::Refract& b) {
	return a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Glass& a, const Materials::Glass& b) {
	return a.reflectance.lock() != b.reflectance.lock() ||
	       a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Emissive& a, const Materials::Emissive& b) {
	return a.emissive.lock() != b.emissive.lock();
}

bool operator!=(const Material& a, const Material& b) {
	if (a.material.index() != b.material.index()) return false;
	return std::visit(
		[&](const auto& material) {
			return material != std::get<std::decay_t<decltype(material)>>(b.material);
		},
		a.material);
}
