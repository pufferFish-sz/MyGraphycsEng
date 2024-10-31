
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>
#include <iostream>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh
	auto surface_area_heuristic = [&](float S_A, float S_B, float N_A, float N_B) {
			return S_A * N_A  + S_B  * N_B;
		};

	auto computeBucketIndex = [&](float centroid, float axis_min, float axis_max, int num_buckets) {
		float axis_range = axis_max - axis_min;
		float normalized_position = (centroid - axis_min) / axis_range;
		int bucket = static_cast<int>(normalized_position * num_buckets);
		return std::clamp(bucket, 0, num_buckets - 1); // Clamp to ensure the bucket is within bounds
		};

	std::function<size_t(size_t, size_t, size_t)> build_recursive = [&](size_t start, size_t end, size_t max_leaf_size) -> size_t {
		BBox box;
		for (size_t i = start; i < end; i++) { // bounding box for current node
			box.enclose(primitives[i].bbox());
		}

		size_t num_prims = end - start;
		if (num_prims <= max_leaf_size) {
			return new_node(box, start, num_prims, 0, 0); //l = r = 0 means leaf
		}
		int num_partitions = 12;
		float best_cost = FLT_MAX;
		int best_axis = -1;
		int best_split = -1;

		// loop through axis x,y,z
		for (int axis = 0; axis < 3; axis++) {
			std::vector<SAHBucketData> buckets(num_partitions); // initialize

			// bucket the primitives
			for (const auto& prim : primitives) {
				Vec3 centroid = prim.bbox().center();
				//std::cout << "the current axis is " << axis << std::endl;
				int bucket_index = computeBucketIndex(centroid[axis], box.min[axis], box.max[axis], num_partitions);
				//std::cout << "the primitive centroid is " << centroid << "and it gets placed in " << bucket_index << std::endl;
				buckets[bucket_index].bb.enclose(prim.bbox());
				buckets[bucket_index].num_prims++;
			}

			// evaluate the SAH for each partition
			for (int split = 1; split < num_partitions; split++) {
				BBox left_box, right_box;
				float left_count = 0, right_count = 0;

				for (int b = 0; b < split; b++) {
					left_box.enclose(buckets[b].bb);
					left_count += buckets[b].num_prims;
				}
				for (int b = split; b < num_partitions; b++) {
					right_box.enclose(buckets[b].bb);
					right_count += buckets[b].num_prims;
				}

				// calculate SAH
				float cost = surface_area_heuristic(left_box.surface_area(), right_box.surface_area(), left_count, right_count);
				if (cost < best_cost) {
					best_cost = cost;
					best_axis = axis;
					best_split = split;
				}
			}
		}

		// partition primitives using best split with std::partition
		if (best_axis != -1) {
			float axis_split_value = box.min[best_axis] + best_split * (box.max[best_axis] - box.min[best_axis]) / (float)num_partitions;

			auto it = std::partition(primitives.begin() + start, primitives.begin() + end, [&](const Primitive& prim) {
				return prim.bbox().center()[best_axis] < axis_split_value;
				});

			size_t mid = std::distance(primitives.begin(), it);

			//recurse on both children
			size_t left_index = build_recursive(start, mid, max_leaf_size);
			size_t right_index = build_recursive(mid, end, max_leaf_size);

			return new_node(box, start, num_prims, left_index, right_index);
		}
		return new_node(box, start, num_prims, 0, 0); // leaf node
		
	};
	// Keep these
    nodes.clear();
    primitives = std::move(prims);

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
	root_idx = build_recursive(0, primitives.size(), max_leaf_size);
	
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
