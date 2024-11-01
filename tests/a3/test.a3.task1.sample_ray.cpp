#include "test.h"
#include "pathtracer/samplers.h"
#include "scene/camera.h"
#include "util/rand.h"


constexpr uint32_t max_depth = 0;

//Create a camera with the following setups
static std::pair<Camera, Mat4> setup_cam(Vec2 wh, Vec3 cent, Vec3 pos, float fov, float ar) {
	Camera c;
	c.aspect_ratio = ar;
	c.vertical_fov = fov;
	c.film.width = static_cast<uint32_t>(wh.x);
	c.film.height = static_cast<uint32_t>(wh.y);
	c.film.samples = 1;
	c.film.max_ray_depth = max_depth;
	c.near_plane = 0.01f;
	Mat4 world_to_camera = Mat4::look_at(pos, cent, Vec3{0, 1, 0});
	return {c, world_to_camera.inverse()};
}

Test test_a3_task1_sample_ray_simple("a3.task1.sample_ray.simple", []() {
	// Create a camera and get the transform matrix from camera to world
	auto [cam, iV] = setup_cam(Vec2(1, 1), Vec3(0, 0, -1), Vec3(), Degrees(2.0f * std::atan(0.5f)), 1.0f);

	// Create a plane from a point and a normal
	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

	// Number of rays to sample
	constexpr uint32_t N = 100000;

	RNG rng;
	for (uint32_t i = 0; i < N; i++) {
		auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
		ret.transform(iV);

		Line l(ret.point, ret.dir);
		Vec3 hitp;
		if (!p.hit(l, hitp)) {
			throw Test::error("Ray did not hit image plane!");
		}

		Vec2 uv = Vec2{hitp.x, hitp.y} + Vec2{0.5f};
		if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) {
			throw Test::error("Ray hit outside image plane!");
		}
	}
});

Test test_a3_task1_sample_ray_miss("a3.task1.sample_ray.miss", []() {
	// Create a camera and get the transform matrix from camera to world
	auto [cam, iV] = setup_cam(Vec2(1, 1), Vec3(0, 0, 1), Vec3(), Degrees(2.0f * std::atan(0.5f)), 1.0f);

	// Create a plane from a point and a normal
	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

	// Number of rays to sample
	constexpr uint32_t N = 100000;

	RNG rng;
	for (uint32_t i = 0; i < N; i++) {
		auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
		ret.transform(iV);

		Line l(ret.point, ret.dir);
		Vec3 hitp;
		if (p.hit(l, hitp)) {
			throw Test::error("Ray did hit image plane!");
		}
	}
});

Test test_a3_task1_sample_ray_center("a3.task1.sample_ray.center", []() {
	// Create a camera and get the transform matrix from camera to world
	auto [cam, iV] = setup_cam(Vec2(2, 2), Vec3(0, 0, -1), Vec3(), Degrees(2.0f * std::atan(0.5f)), 1.0f);

	// Create a plane from a point and a normal
	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

	// Number of rays to sample
	constexpr uint32_t N = 100000;

	RNG rng;
	for (uint32_t i = 0; i < N; i++) {
		auto [ret, pdf] = cam.sample_ray(rng, 1, 1);
		ret.transform(iV);

		Line l(ret.point, ret.dir);
		Vec3 hitp;
		if (!p.hit(l, hitp)) {
			throw Test::error("Ray did not hit image plane!");
		}

		Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ 0.5f };

		if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) {
			throw Test::error("Ray hit outside image plane!");
		}
	}
	});

Test test_a3_task1_sample_ray_fov1("a3.task1.sample_ray.fov1", []() {
	// Create a camera and get the transform matrix from camera to world
	auto [cam, iV] = setup_cam(Vec2(1, 1), Vec3(0, 0, -1), Vec3(), Degrees(1.0f * std::atan(0.5f)), 1.0f);

	// Create a plane from a point and a normal
	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

	// Number of rays to sample
	constexpr uint32_t N = 100000;

	RNG rng;
	for (uint32_t i = 0; i < N; i++) {
		auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
		ret.transform(iV);

		Line l(ret.point, ret.dir);
		Vec3 hitp;
		if (!p.hit(l, hitp)) {
			throw Test::error("Ray did not hit image plane!");
		}

		Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ 0.5f };

		if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) {
			throw Test::error("Ray hit outside image plane!");
		}
	}
	});

//Test test_a3_task1_sample_ray_fov2("a3.task1.sample_ray.fov2", []() {
//	// Create a camera and get the transform matrix from camera to world
//	auto [cam, iV] = setup_cam(Vec2(3, 3), Vec3(0, 0, -1), Vec3(), Degrees(3.0f * std::atan(0.5f)), 1.0f);
//
//	// Create a plane from a point and a normal
//	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));
//
//	// Number of rays to sample
//	constexpr uint32_t N = 100000;
//
//	RNG rng;
//	for (uint32_t i = 0; i < N; i++) {
//		auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
//		ret.transform(iV);
//
//		Line l(ret.point, ret.dir);
//		Vec3 hitp;
//		if (!p.hit(l, hitp)) {
//			throw Test::error("Ray did not hit image plane!");
//		}
//
//		Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ 0.5f };
//
//		if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) {
//			throw Test::error("Ray hit outside image plane!");
//		}
//	}
//	});

//Test test_a3_task1_sample_ray_1080p_simple("a3.task1.sample_ray.1080p.simple", []() {
//	// Create a camera and get the transform matrix from camera to world
//	auto [cam, iV] = setup_cam(Vec2(16, 9), Vec3(0, 0, -1), Vec3(), Degrees(2.0f * std::atan(0.5f)), 1920.0f / 1080.0f);
//
//	// Create a plane from a point and a normal
//	Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));
//
//	// Number of rays to sample
//	constexpr uint32_t N = 100000;
//
//	RNG rng;
//	for (uint32_t i = 0; i < N; i++) {
//		auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
//		ret.transform(iV);
//
//		Line l(ret.point, ret.dir);
//		Vec3 hitp;
//		if (!p.hit(l, hitp)) {
//			throw Test::error("Ray did not hit image plane!");
//		}
//
//		Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ 0.5f };
//
//		if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) {
//			throw Test::error("Ray hit outside image plane!");
//		}
//	}
//	});

Test test_a3_task1_sample_ray_custom_template("a3.task1.sample_ray.custom_template", []() {
    // Create a camera and get the transform matrix from camera to world
    auto fw = 1; //modify this!
    auto fh = 1; //modify this! ratio of fw fh affects aspect_ratio
    auto h = 1.0f; //modify this! this affects vertical_fov

    //not sure whehter I should care about the case where center is off the origin
    auto center = Vec3{ 0.0f, 0.0f, 0.0f }; //leave the z to be 0
    auto ar = static_cast<float>(fw) / static_cast<float>(fh);
    auto w = ar * h;
    //changing Vec3(0, 0, -1) to Vec3(0, 0, 1) to check failing cases
    auto [cam, iV] = setup_cam(Vec2(fw, fh), Vec3(0, 0, -1), center, Degrees(2.0f * std::atan(h / 2.0f)), ar);

    // Create a plane from a point and a normal
    Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

    // Number of rays to sample
    constexpr uint32_t N = 100000;

    RNG rng;
    for (uint32_t i = 0; i < N; i++) {
        auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
        ret.transform(iV);

        Line l(ret.point, ret.dir);
        Vec3 hitp;
        if (!p.hit(l, hitp)) {
            throw Test::error("Ray did not hit image plane!");
        }

        Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ w / 2.0f,h / 2.0f };
        if (uv.x < center.x || uv.x > w + center.x || uv.y < center.y || uv.y > h + center.y) {
            throw Test::error("Ray hit outside image plane!");
        }
    }
    });

Test test_a3_task1_sample_ray_wide("a3.task1.sample_ray.wide", []() {
    // Create a camera and get the transform matrix from camera to world
    auto fw = 100; //modify this!
    auto fh = 1; //modify this!
    auto h = 5.0f; //modify this!

    //not sure whehter I should care about the case where center is off the origin
    auto center = Vec3{ 0.0f, 0.0f, 0.0f }; //leave the z to be 0
    auto ar = static_cast<float>(fw) / static_cast<float>(fh);
    auto w = ar * h;
    //changing Vec3(0, 0, -1) to Vec3(0, 0, 1) to check failing cases
    auto [cam, iV] = setup_cam(Vec2(fw, fh), Vec3(0, 0, -1), center, Degrees(2.0f * std::atan(h / 2.0f)), ar);

    // Create a plane from a point and a normal
    Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

    // Number of rays to sample
    constexpr uint32_t N = 100000;

    RNG rng;
    for (uint32_t i = 0; i < N; i++) {
        auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
        ret.transform(iV);

        Line l(ret.point, ret.dir);
        Vec3 hitp;
        if (!p.hit(l, hitp)) {
            throw Test::error("Ray did not hit image plane!");
        }

        Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ w / 2.0f,h / 2.0f };
        if (uv.x < center.x || uv.x > w + center.x || uv.y < center.y || uv.y > h + center.y) {
            throw Test::error("Ray hit outside image plane!");
        }
    }
    });

Test test_a3_task1_sample_ray_high("a3.task1.sample_ray.high", []() {
    // Create a camera and get the transform matrix from camera to world
    auto fw = 1; //modify this!
    auto fh = 100; //modify this!
    auto h = 100.0f; //modify this!

    //not sure whehter I should care about the case where center is off the origin
    auto center = Vec3{ 0.0f, 0.0f, 0.0f }; //leave the z to be 0
    auto ar = static_cast<float>(fw) / static_cast<float>(fh);
    auto w = ar * h;
    //changing Vec3(0, 0, -1) to Vec3(0, 0, 1) to check failing cases
    auto [cam, iV] = setup_cam(Vec2(fw, fh), Vec3(0, 0, -1), center, Degrees(2.0f * std::atan(h / 2.0f)), ar);

    // Create a plane from a point and a normal
    Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

    // Number of rays to sample
    constexpr uint32_t N = 100000;

    RNG rng;
    for (uint32_t i = 0; i < N; i++) {
        auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
        ret.transform(iV);

        Line l(ret.point, ret.dir);
        Vec3 hitp;
        if (!p.hit(l, hitp)) {
            throw Test::error("Ray did not hit image plane!");
        }

        Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ w / 2.0f,h / 2.0f };
        if (uv.x < center.x || uv.x > w + center.x || uv.y < center.y || uv.y > h + center.y) {
            throw Test::error("Ray hit outside image plane!");
        }
    }
    });

Test test_a3_task1_sample_ray_small("a3.task1.sample_ray.small", []() {
    // Create a camera and get the transform matrix from camera to world
    auto fw = 1; //modify this!
    auto fh = 1; //modify this!
    auto h = 0.05f; //modify this!

    //not sure whehter I should care about the case where center is off the origin
    auto center = Vec3{ 0.0f, 0.0f, 0.0f }; //leave the z to be 0
    auto ar = static_cast<float>(fw) / static_cast<float>(fh);
    auto w = ar * h;
    //changing Vec3(0, 0, -1) to Vec3(0, 0, 1) to check failing cases
    auto [cam, iV] = setup_cam(Vec2(fw, fh), Vec3(0, 0, -1), center, Degrees(2.0f * std::atan(h / 2.0f)), ar);

    // Create a plane from a point and a normal
    Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

    // Number of rays to sample
    constexpr uint32_t N = 100000;

    RNG rng;
    for (uint32_t i = 0; i < N; i++) {
        auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
        ret.transform(iV);

        Line l(ret.point, ret.dir);
        Vec3 hitp;
        if (!p.hit(l, hitp)) {
            throw Test::error("Ray did not hit image plane!");
        }

        Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ w / 2.0f,h / 2.0f };
        if (uv.x < center.x || uv.x > w + center.x || uv.y < center.y || uv.y > h + center.y) {
            throw Test::error("Ray hit outside image plane!");
        }
    }
    });

Test test_a3_task1_sample_ray_big("a3.task1.sample_ray.big", []() {
    // Create a camera and get the transform matrix from camera to world
    auto fw = 1; //modify this!
    auto fh = 1; //modify this!
    auto h = 1000.0f; //modify this!

    //not sure whehter I should care about the case where center is off the origin
    auto center = Vec3{ 0.0f, 0.0f, 0.0f }; //leave the z to be 0
    auto ar = static_cast<float>(fw) / static_cast<float>(fh);
    auto w = ar * h;
    //changing Vec3(0, 0, -1) to Vec3(0, 0, 1) to check failing cases
    auto [cam, iV] = setup_cam(Vec2(fw, fh), Vec3(0, 0, -1), center, Degrees(2.0f * std::atan(h / 2.0f)), ar);

    // Create a plane from a point and a normal
    Plane p(Vec3(0, 0, -1), Vec3(0, 0, 1));

    // Number of rays to sample
    constexpr uint32_t N = 100000;

    RNG rng;
    for (uint32_t i = 0; i < N; i++) {
        auto [ret, pdf] = cam.sample_ray(rng, 0, 0);
        ret.transform(iV);

        Line l(ret.point, ret.dir);
        Vec3 hitp;
        if (!p.hit(l, hitp)) {
            throw Test::error("Ray did not hit image plane!");
        }

        Vec2 uv = Vec2{ hitp.x, hitp.y } + Vec2{ w / 2.0f,h / 2.0f };
        if (uv.x < center.x || uv.x > w + center.x || uv.y < center.y || uv.y > h + center.y) {
            throw Test::error("Ray hit outside image plane!");
        }
    }
    });