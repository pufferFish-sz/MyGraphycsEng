// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"
template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:

	std::vector<Fragment> fragments;

	// helper used to put output of rasterization functions into fragments:
	auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };

	// actually do rasterization:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2) {
			rasterize_line(clipped_vertices[i], clipped_vertices[i + 1], emit_fragment);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3) {
			rasterize_triangle(clipped_vertices[i], clipped_vertices[i + 1], clipped_vertices[i + 2], emit_fragment);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// depth test + shade + blend fragments:
	uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
							   // (indicates something is wrong with clipping)
	for (auto const& f : fragments) {

		// fragment location (in pixels):
		int32_t x = (int32_t)std::floor(f.fb_position.x);
		int32_t y = (int32_t)std::floor(f.fb_position.y);

		// if clipping is working properly, this condition shouldn't be needed;
		// however, it prevents crashes while you are working on your clipping functions,
		// so we suggest leaving it in place:
		if (x < 0 || (uint32_t)x >= framebuffer.width || 
		    y < 0 || (uint32_t)y >= framebuffer.height) {
			++out_of_range;
			continue;
		}

		// local names that refer to destination sample in framebuffer:
		float& fb_depth = framebuffer.depth_at(x, y, 0);
		Spectrum& fb_color = framebuffer.color_at(x, y, 0);


		// depth test:
		if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
			// "Always" means the depth test always passes.
		} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
			// "Never" means the depth test never passes.
			continue; //discard this fragment
		} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
			// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
			// A1T4: Depth_Less
			// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
			if (f.fb_position.z < fb_depth) {
				// Depth passes, so keep processing
			}
			else {
				continue; // discard fragment
			}

		} else {
			static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
		}

		// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
		if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
			fb_depth = f.fb_position.z;
		}

		// shade fragment:
		ShadedFragment sf;
		sf.fb_position = f.fb_position;
		Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

		// write color to framebuffer if color writes aren't disabled:
		if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
			// blend fragment:
			if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
				fb_color = sf.color;
			} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
				// A1T4: Blend_Add
				// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
				fb_color += sf.color * sf.opacity; //<-- replace this line
			} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
				// A1T4: Blend_Over
				// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
				// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
				fb_color = sf.color * sf.opacity + fb_color * (1.0f - sf.opacity); //<-- replace this line
			} else {
				static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
			}
		}
	}
	if (out_of_range > 0) {
		if constexpr (primitive_type == PrimitiveType::Lines) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_line function.",
			     out_of_range);
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_triangle function.",
			     out_of_range);
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	emit_vertex(va);
	emit_vertex(vb);
	emit_vertex(vc);
}

// -------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */



template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}
	// A1T2: rasterize_line
	// Find what quadrant the point is in 
	auto quadrant_num = [](Vec2 pixel_center, Vec2 line_point) -> int {	
		if (line_point.x < pixel_center.x) {
			if (line_point.y > pixel_center.y) {
				return 2;
			}
			return 3;
		}
		else {
			if (line_point.y > pixel_center.y) {
				return 1;
			}
			return 4;
		}
		};

	// color the pixel
	auto fill_pixel = [&](Vec2 pixel_center, Vec2 line_point, ClippedVertex const& va, ClippedVertex const& vb, float w) {
		Fragment frag;
		frag.fb_position = Vec3(pixel_center.x, pixel_center.y, va.fb_position.z + w * (vb.fb_position.z - va.fb_position.z));
		frag.attributes = va.attributes;
		frag.derivatives.fill(Vec2(0.0f, 0.0f));
		emit_fragment(frag);
	};

	// the rule to check if the point is within the diamond
	auto exit_from_diamond = [&](Vec2 pixel_center, Vec2 line_point) -> bool {
		Vec2 bot_point = Vec2(pixel_center.x + 0.5f, pixel_center.y); // bot
		Vec2 left_point = Vec2(pixel_center.x, pixel_center.y + 0.5f); // left
		/*std::cout << "current point is: " << line_point << std::endl;
		std::cout << "current quadrant is: " << quadrant_num(pixel_center, line_point) << std::endl;*/
		return abs(line_point.x - pixel_center.x) + abs(line_point.y - pixel_center.y) <= 0.5f;
	};

	auto end_point_exit = [&](Vec2 pixel_center, Vec2 line_point, int major_axis) -> bool {
		Vec2 top_point = Vec2(pixel_center.x, pixel_center.y + 0.5f); // top
		Vec2 right_point = Vec2(pixel_center.x + 0.5f, pixel_center.y); // right
		int quadrant = quadrant_num(pixel_center, line_point);
		//std::cout << "current point is: " << line_point << std::endl;
		//std::cout << "current quadrant is: " << quadrant << std::endl;
		if (!exit_from_diamond(pixel_center, line_point)) {
		
			if (major_axis == 0) {
				if (line_point == right_point || quadrant == 1 || quadrant == 4) {
					return true;
				}
			}
			else {
				if (line_point == top_point || quadrant == 1 || quadrant == 2) {
					return true;
				}
			}
		}
		return false;
	};

	auto start_point_exit = [=](Vec2 pixel_center, Vec2 line_point, int major_axis) -> bool {
		Vec2 bot_point = Vec2(pixel_center.x + 0.5f, pixel_center.y); // bot
		Vec2 left_point = Vec2(pixel_center.x, pixel_center.y + 0.5f); // left
		int quadrant = quadrant_num(pixel_center, line_point);
		//std::cout << "current start point is: " << line_point << std::endl;
		//std::cout << "current quadrant is: " << quadrant << std::endl;
		//std::cout << "major axis is: " << major_axis << std::endl;
		if (!exit_from_diamond(pixel_center, line_point)) {
		
			if (major_axis == 0) {
				if (((line_point != bot_point) && (line_point != left_point) && (quadrant != 2) && (quadrant != 3)) || (quadrant == 1)) {
					return false;
				}
			}
			else {
				if (((line_point != bot_point) && (line_point != left_point) && (quadrant != 4) && (quadrant != 3)) || (quadrant == 1)) {
					return false;
				}
			}
		}
		return true;
		};

	Vec2 a = Vec2(va.fb_position.x, va.fb_position.y);
	Vec2 b = Vec2(vb.fb_position.x, vb.fb_position.y);
	
	float delta_x = std::abs(b.x - a.x);
	float delta_y = std::abs(b.y - a.y);

	float line_length = sqrtf(delta_x * delta_x + delta_y * delta_y);

	int i;//, j; // determine major axis
	if (delta_x > delta_y) {
		i = 0;
		//j = 1;
	}
	else {
		i = 1;
		//j = 0;
	}

	// Vertical line edge case: delta_x == 0 (perfectly vertical)
	if (delta_x == 0) {
		i = 1; // Force Y to be the major axis
		//j = 0; // X is irrelevant in this case
	}

	// make sure that slope always positive
	if ((i == 0 && a.x > b.x) || (i == 1 && a.y > b.y)) {
		std::swap(a,b);
	}

	float a_i = (i == 0) ? a.x : a.y;
	float b_i = (i == 0) ? b.x : b.y;
	float a_j = (i == 1) ? a.x : a.y;
	float b_j = (i == 1) ? b.x : b.y;

	int t1 = static_cast<int> (std::floor(a_i));
	int t2 = static_cast<int> (std::floor(b_i));

	// along the major axis
	for (int u = t1; u <= t2; ++u) {
		float w = ((u + 0.5f) - a_i) / (b_i - a_i);
		float v = w * (b_j - a_j) + a_j;

		Vec2 line_point = Vec2((i == 0) ? (u + 0.5f) : v, (i == 0) ? v : (u + 0.5f));
		Vec2 pixel_center = Vec2((i == 0) ? (u + 0.5f) : (std::floor(v) + 0.5f),
			(i == 0) ? (std::floor(v) + 0.5f) : (u + 0.5f));

		if (u == t1) { // start point
			// also sample the second point to see if the distance between 1st and 2nd point is 
			// more than the total line distance
			int u_2 = t1 + 1;
			float w_2 = ((u_2 + 0.5f) - a_i) / (b_i - a_i);
			float v_2 = w_2 * (b_j - a_j) + a_j;
			Vec2 secPoint = Vec2((i == 0) ? (u_2 + 0.5f) : v_2, (i == 0) ? v_2 : (u_2 + 0.5f));
			Vec2 pixel_center_2 = Vec2((i == 0) ? (u_2 + 0.5f) : (std::floor(v_2) + 0.5f),
				(i == 0) ? (std::floor(v_2) + 0.5f) : (u_2 + 0.5f));

			// dist between 1st and 2nd point
			float dist = sqrtf((secPoint.x - line_point.x) * (secPoint.x - line_point.x) + 
				(secPoint.y - line_point.y) * (secPoint.y - line_point.y));

			if (start_point_exit(pixel_center, a, i)) {
				if (line_length < dist) {
					if (end_point_exit(pixel_center, b, i)) {
						fill_pixel(pixel_center, line_point, va, vb, w);
					}
				}
				else if (exit_from_diamond(pixel_center_2, secPoint)) {
					fill_pixel(pixel_center, line_point, va, vb, w);
				}
			}
		}
		else if (u == t2) { // end point
			if (end_point_exit(pixel_center, b, i)) {
				fill_pixel(pixel_center, line_point, va, vb, w);
			}
		}
		else {  // all other mid points
			if (exit_from_diamond(pixel_center, line_point)) {
				fill_pixel(pixel_center, line_point, va, vb, w);
			}
		}
	}
}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {

	// Use cross product to see if the point is in the triangle
	auto in_triangle = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 q) -> bool {
		Vec3 ac = c - a;
		Vec3 ab = b - a;
		Vec3 aq = q - a;
		Vec3 cb = b - c;
		Vec3 ca = a - c;
		Vec3 cq = q - c;
		Vec3 ba = a - b;
		Vec3 bc = c - b;
		Vec3 bq = q - b;

		return (dot(cross(ac, ab), cross(ac, aq)) >= 0) && (dot(cross(cb, ca), cross(cb, cq)) >= 0)
			&& (dot(cross(ba, bc), cross(ba, bq)) >= 0);

		};

	auto is_on_edge = [&](Vec3 a, Vec3 b, Vec3 q) {
		Vec3 ab = b - a;
		Vec3 aq = q - a;
		return cross(ab, aq).z == 0.0f;
		};

	auto calculate_abc = [&](Vec3 q, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc) {
		Vec3 params;
		Vec3 ab = vb.fb_position - va.fb_position;
		Vec3 ac = vc.fb_position - va.fb_position;
		Vec3 bq = q - vb.fb_position;
		Vec3 bc = vb.fb_position - vc.fb_position;
		Vec3 cq = q - vc.fb_position;
		Vec3 ca = vc.fb_position - va.fb_position;

		float area_abc = cross(ab, ac).z;
		float alpha = cross(bq, bc).z / area_abc;
		float beta = cross(cq, ca).z / area_abc;
		float gamma = 1.0f - alpha - beta;

		params.x = alpha;
		params.y = beta;
		params.z = gamma;

		return params;
		};

	// New Fragment
	auto new_frag_smooth = [&](Vec3 q, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc) {
		Vec3 params = calculate_abc(q, va, vb, vc);
		float alpha = params.x;
		float beta = params.y;
		float gamma = params.z;

		// interpolate the depth (z-value)
		float z = alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z;

		Fragment frag;
		frag.fb_position = Vec3(q.x + 0.5f, q.y + 0.5f, z);
		for (uint32_t i = 0; i < va.attributes.size(); ++i) { // interpolate the attributes
			frag.attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
		}
		return frag;
	};

	// New Fragment
	auto new_frag_correct = [&](Vec3 q, ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc) {
			//barycentric coordinates to interpolate depth
			Vec3 params = calculate_abc(q, va, vb, vc);
			float alpha = params.x;
			float beta = params.y;
			float gamma = params.z;

			// interpolate inv_w using barycentric coordinates
			float inv_w_interpolated = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;

			Fragment frag;
			// interpolate the attributes divided by w (inv_w) using barycentric coordinates
			for (uint32_t i = 0; i < va.attributes.size(); ++i) {
				// attribute at each vertex divided by w
				float Pa = va.attributes[i] * va.inv_w;
				float Pb = vb.attributes[i] * vb.inv_w;
				float Pc = vc.attributes[i] * vc.inv_w;

				// interpolate the attribute
				float P_interpolated = alpha * Pa + beta * Pb + gamma * Pc;

				frag.attributes[i] = P_interpolated / inv_w_interpolated;
			}
			return frag;
		};


	Vec3 ab = vb.fb_position - va.fb_position;
	Vec3 ac = vc.fb_position - va.fb_position;
	Vec3 crossed = cross(ab, ac);

	// Figure out orientation
	float area = 0.5f * crossed.z;
	//std::cout << "this is the area: " << area << std::endl;
	// Check if the triangle is clockwise or counterclockwise
	bool is_cw = (area < 0.0f);
	//std::cout << "it's currently cw: " << is_cw << std::endl;

	// bounding box of the triangle
	float minX = std::min({ va.fb_position.x, vb.fb_position.x, vc.fb_position.x });
	float minY = std::min({ va.fb_position.y, vb.fb_position.y, vc.fb_position.y });
	float maxX = std::max({ va.fb_position.x, vb.fb_position.x, vc.fb_position.x });
	float maxY = std::max({ va.fb_position.y, vb.fb_position.y, vc.fb_position.y });

	// iterate over the bounding box and check if each point is in the triangle
	for (int y = static_cast<int> (std::floor(minY)); y <= static_cast<int> (std::ceil(maxY)); ++y) {
		for (int x = static_cast<int> (std::floor(minX)); x <= static_cast<int> (std::ceil(maxX)); ++x) {

			// center of the pixel
			Vec3 q = Vec3(x + 0.5f, y + 0.5f, 0.0f);

			if (in_triangle(va.fb_position, vb.fb_position, vc.fb_position, q)) {
				// check with the top left rule
				bool on_left_edge = false, on_top_edge = false;
				bool in_mid = true;

				if (is_on_edge(va.fb_position, vb.fb_position, q)) {
					if (is_cw ? (va.fb_position.y < vb.fb_position.y) : (va.fb_position.y > vb.fb_position.y)) {
						on_left_edge = true;
					}
					else if ((va.fb_position.y == vb.fb_position.y) && (va.fb_position.y > vc.fb_position.y)) {
						on_top_edge = true;
					}
					in_mid = false;
				}
				else if (is_on_edge(vb.fb_position, vc.fb_position, q)) {
					if (is_cw ? (vb.fb_position.y < vc.fb_position.y) : (vb.fb_position.y > vc.fb_position.y)) {
						on_left_edge = true;
					}
					else if (vb.fb_position.y == vc.fb_position.y && (vb.fb_position.y > va.fb_position.y)) {
						on_top_edge = true;
					}
					in_mid = false;
				}

				else if (is_on_edge(vc.fb_position, va.fb_position, q)) {
					if (is_cw ? (vc.fb_position.y < va.fb_position.y) : (vc.fb_position.y > va.fb_position.y)) {
						on_left_edge = true;
					}
					else if (vc.fb_position.y == va.fb_position.y && (va.fb_position.y > vb.fb_position.y)) {
						on_top_edge = true;
					}
					in_mid = false;
				}

				if ((on_left_edge && !on_top_edge) || (on_top_edge && !on_left_edge) || in_mid) {

					if (q.y == std::min({ va.fb_position.y, vb.fb_position.y, vc.fb_position.y })) {
						continue; // Skip the fragment on the bottom edge
					}
					//barycentric coordinates to interpolate depth
					Vec3 aq = q - va.fb_position;
					Vec3 bq = q - vb.fb_position;
					Vec3 bc = vb.fb_position - vc.fb_position;
					Vec3 cq = q - vc.fb_position;
					Vec3 ca = vc.fb_position - va.fb_position;

					float area_abc = cross(ab, ac).z;
					float alpha = cross(bq, bc).z / area_abc;
					float beta = cross(cq, ca).z / area_abc;
					float gamma = 1.0f - alpha - beta;

					// interpolate the depth (z-value)
					float z = alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z;

					Fragment frag;
					frag.fb_position = Vec3(x + 0.5f, y + 0.5f, z);

					if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {

						// A1T3: flat triangles
						frag.attributes = va.attributes;
						emit_fragment(frag);
					}
					else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
						// A1T5: screen-space smooth triangles
						//if Interp_Smooth: interpolate as if (a, b, c) is a 2D triangle flat on the screen

						for (uint32_t i = 0; i < va.attributes.size(); ++i) { // interpolate the attributes
							frag.attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
						}
						// the derivate would be just computing the 2x2 block, top pixel and right pixel, change in screen space
						//calculate frag attribute at (x + 1, y)
						Fragment frag1 = new_frag_smooth(Vec3(q.x + 1, q.y, q.z),va,vb,vc);
						//calculate frag attribute at (x, y - 1)
						Fragment frag2 = new_frag_smooth(Vec3(q.x, q.y - 1, q.z),va,vb,vc);

						// calculate the derivatives
						for (uint32_t i = 0; i < frag1.attributes.size(); ++i) {
							frag.derivatives[i].x = (frag1.attributes[i] - frag.attributes[i]);
							frag.derivatives[i].y = (frag.attributes[i] - frag2.attributes[i]);
						}
						emit_fragment(frag);
					}
					else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
						// A1T5: perspective correct triangles
						//interpolate inv_w for all three points because they all have different z

						// interpolate inv_w using barycentric coordinates
						float inv_w_interpolated = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;

						// interpolate the attributes divided by w (inv_w) using barycentric coordinates
						for (uint32_t i = 0; i < va.attributes.size(); ++i) {
							// attribute at each vertex divided by w
							float Pa = va.attributes[i] * va.inv_w;
							float Pb = vb.attributes[i] * vb.inv_w;
							float Pc = vc.attributes[i] * vc.inv_w;

							// interpolate the attribute
							float P_interpolated = alpha * Pa + beta * Pb + gamma * Pc;

							frag.attributes[i] = P_interpolated / inv_w_interpolated;
						}

						//calculate frag attribute at (x + 1, y)
						Fragment frag1 = new_frag_correct(Vec3(q.x + 1, q.y, q.z),va, vb, vc);
						//calculate frag attribute at (x, y - 1)
						Fragment frag2 = new_frag_correct(Vec3(q.x, q.y - 1, q.z),va, vb, vc);

						// calculate the derivatives
						for (uint32_t i = 0; i < frag1.attributes.size(); ++i) {
							frag.derivatives[i].x = (frag1.attributes[i] - frag.attributes[i]);
							frag.derivatives[i].y = (frag.attributes[i] - frag2.attributes[i]);
						}

						emit_fragment(frag);

						}
					}

				}
			}
		}
	}


//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;