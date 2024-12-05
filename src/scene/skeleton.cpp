#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}
std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
	for (auto const &bone : bones) {
		(void)bone; //avoid complaints about unused bone
		//placeholder -- your code should actually compute the correct transform:
		
		Mat4 location;
		if (bone.parent == -1U) { // no parent
			location = Mat4::translate(base);
		}
		else { // with parent
			Mat4 local_transform = Mat4::translate(bones[bone.parent].extent);
			location = bind[bone.parent] * local_transform; // translate by extent of its parent
		}
		bind.emplace_back(location);
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;
}


std::vector< Mat4 > Skeleton::current_pose() const {
    //A4T2a: bone-to-skeleton transformations in the current pose

	//Similar to bind_pose(), but takes rotation from Bone::pose into account.
	// (and translation from Skeleton::base_offset!)

	//You'll probably want to write a loop similar to bind_pose().

	//Useful functions:
	//Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	//Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.

	std::vector< Mat4 > current;
	current.reserve(bones.size());

	for (const auto& bone : bones) {
		Mat4 location;
		Mat4 translation;
		Mat4 rotation;
		Vec3 x, y, z;
		bone.compute_rotation_axes(&x, &y, &z);

		if (bone.parent == -1U) {
			translation = Mat4::translate(base + base_offset);

			rotation = Mat4::angle_axis(bone.pose.z, z) *
					   Mat4::angle_axis(bone.pose.y, y) * 
					   Mat4::angle_axis(bone.pose.x, x);
			location = translation * rotation;
		}
		else {
			translation = Mat4::translate(bones[bone.parent].extent);

			rotation = Mat4::angle_axis(bone.pose.z, z) *
					   Mat4::angle_axis(bone.pose.y, y) *
				       Mat4::angle_axis(bone.pose.x, x);

			location = current[bone.parent] * translation * rotation;
		}

		current.emplace_back(location);
	}
	assert(current.size() == bones.size());
	return current;
}

std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
    //A4T2b: IK gradient

    // Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
	std::vector< Mat4 > poses = current_pose();
	Mat4 transform = Mat4::I; // Accumulated transformation matrix
	Mat4 prev_transform;
	//TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	//remember bone.compute_rotation_axes() -- should be useful here, too!
	for (auto& handle : handles) {
		if (!handle.enabled) continue; // skip disabled handles

		Vec3 h = handle.target; // target position
		Vec3 p;                 // tip position of the handle's associated bone

		// walk up the kinematic chain to compute gradients
		for (BoneIndex b = handle.bone; b != -1U; b = bones[b].parent) {
			const Bone& bone = bones[b];

			// compute local axes
			Vec3 x, y, z;
			bone.compute_rotation_axes(&x, &y, &z);
			
			if (bone.parent == -1U) {
				transform = Mat4::translate(base + base_offset);
			}
			else {
				transform = poses[bone.parent] * Mat4::translate(bones[bone.parent].extent);
			}

			// save transform up to, but not including current bone
			prev_transform = transform;

			Mat4 rotation =
				Mat4::angle_axis(bone.pose.z, z) *
				Mat4::angle_axis(bone.pose.y, y) *
				Mat4::angle_axis(bone.pose.x, x);

			transform = transform * rotation;

			Vec3 r = prev_transform * Vec3(0,0,0);

			// compute transformed axes in skeleton-local space
			Vec3 y_axis = (prev_transform * Mat4::angle_axis(bone.pose.z, z)).rotate(y);
			Vec3 x_axis = (prev_transform * Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y)).rotate(x);
			Vec3 z_axis = prev_transform.rotate(z);

			if (b == handle.bone) {
				p = transform * bone.extent;
			}

			// gradient contributions using cross product
			Vec3 diff = p - h; // vector to target
			Vec3 relative_tip = p - r; // tip relative to bone origin

			gradient[b].x += dot(cross(x_axis, relative_tip), diff);
			gradient[b].y += dot(cross(y_axis, relative_tip), diff);
			gradient[b].z += dot(cross(z_axis, relative_tip), diff);
		}
	}
	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//check which handles are enabled
	//run `steps` iterations
	
	//call gradient_in_current_pose() to compute d loss / d pose
	//add ...
	auto compute_loss = [&]() -> float {
		float loss = 0.0f;
		std::vector<Mat4> poses = current_pose();
		for (const auto& handle : handles) {
			if (!handle.enabled) continue;

			Mat4 transform = Mat4::I;
			for (BoneIndex b = handle.bone; b != -1U; b = bones[b].parent) {
				const Bone& bone = bones[b];
				Vec3 x, y, z;
				bone.compute_rotation_axes(&x, &y, &z);

				if (bone.parent == -1U) {
					transform = Mat4::translate(base + base_offset);
				}
				else {
					transform = poses[bone.parent] * Mat4::translate(bones[bone.parent].extent);
				}

				Mat4 rotation =
					Mat4::angle_axis(bone.pose.z, z) *
					Mat4::angle_axis(bone.pose.y, y) *
					Mat4::angle_axis(bone.pose.x, x);

				transform = transform * rotation;
			}
			Vec3 tip = transform * bones[handle.bone].extent;
			Vec3 diff = tip - handle.target;
			loss += 0.5f * dot(diff, diff); // Use squared distance
		}
		return loss;
		};

	// gradient descent loop
	std::vector<Vec3> pose_prev(bones.size()); // Store the previous pose
	std::vector<Vec3> pose_curr(bones.size()); // Current pose

	// Initialize previous and current poses from bones
	for (size_t i = 0; i < bones.size(); i++) {
		pose_prev[i] = bones[i].pose;
		pose_curr[i] = pose_prev[i];
	}

	for (uint32_t step = 0; step < steps; step++) {
		float loss_prev = compute_loss();
		/*std::cout << "loss_prev " << loss_prev << std::endl;
		std::cout << "at step " << step << std::endl;*/
		std::vector<Vec3> gradient = gradient_in_current_pose(); // Compute gradient
		float step_size = 1.0f; // Initial step size
		float loss_trial = 0.0f;

		// gradient descent with backtracking line search
		do {
			// update poses using gradient descent
			for (size_t i = 0; i < bones.size(); i++) {
				pose_curr[i] = pose_prev[i] - step_size * gradient[i];
				bones[i].pose = pose_curr[i];
			}

			loss_trial = compute_loss(); // Recompute loss with updated pose
			step_size /= 2.0f; // Reduce step size if loss increased
			//std::cout << "loss trial: " << loss_trial << " loss_prev" << loss_prev << std::endl;
		} while (loss_trial > loss_prev);

		// check for convergence
		if (step > 10 && std::abs(loss_prev - loss_trial) < 1e-5f) {
			return true; 
		}

		// update pose_prev for the next iteration
		pose_prev = pose_curr;
	}

	//if at a local minimum (e.g., gradient is near-zero), return 'true'.
	//if run through all steps, return `false`.
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)

    // Return the closest point to 'p' on the line segment from a to b

	//Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)
	Vec3 ap = p - a;
	Vec3 ab = b - a;
	Vec3 ret;

	float t = dot(ap, ab) / dot(ab, ab);

	// edge cases
	if (t <= 0) { // if point is before start
		ret = a;
	}
	else if (t >= 1) { // if point is after end
		ret = b;
	}
	else {
		ret = a + ab * t; // in between
	}

    return ret;
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; //avoid complaints about unused mesh

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)
	//be sure to use bone positions in the bind pose (not the current pose!)
	//you should fill in the helper closest_point_on_line_segment() before working on this function
	
	std::vector< Mat4 > poses = bind_pose();

	for (auto& vertex : mesh.vertices) {

		float weight_sum = 0.0f;

		for (auto& bone_weight : vertex.bone_weights) {
			const Bone& bone = bones[bone_weight.bone];

			// start and end points of the bone in the bind pose
			Vec3 bone_start = poses[bone_weight.bone] * Vec3(0.0f);
			Vec3 bone_end = poses[bone_weight.bone] * bone.extent;
			Vec3 closest_point = closest_point_on_line_segment(bone_start, bone_end, vertex.position);

			float dist = (vertex.position - closest_point).norm();
			float new_weight = std::max(0.0f, bone.radius - dist) / bone.radius;

			// assign the weight if it is non-zero
			if (new_weight > 0.0f) {
				bone_weight.weight = new_weight;
				weight_sum += new_weight;
			}
		}

		// normalize weights
		if (weight_sum > 0.0f) {
			for (auto& weights : vertex.bone_weights) {
				if (weights.weight > 0.0f) {
					weights.weight /= weight_sum;
				}
			}
		}
	}

}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());


	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector<Mat4> bind_to_current(bind.size());
	for (size_t i = 0; i < bind.size(); ++i) {
		bind_to_current[i] = current[i] * bind[i].inverse();
	}

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) {
		//skinned_positions.emplace(vi, vi->position); //PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		//NOTE: vertices with empty bone_weights should remain in place.
		if (vi->bone_weights.empty()) {
			Mat4 trans = bind_to_current[0];
			Vec3 skinned_pos = trans * vi->position;
			skinned_positions.emplace(vi, skinned_pos);
			//std::cout << "for vertex vi : " << skinned_pos << "this is bone weight empty case" << std::endl;
		}
		else {
			Vec3 skinned_position = Vec3(0.0f);
			for (const auto& bone_weight : vi->bone_weights) {
				if (bone_weight.weight > 0.0f) {
					Mat4 transform = bind_to_current[bone_weight.bone];
					skinned_position += bone_weight.weight * (transform * vi->position);
				}
			}
			skinned_positions.emplace(vi, skinned_position);
			//std::cout << "for vertex vi : " << vi->id << "this is bone weight non-empty case" << std::endl;
		}

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted
			if (!h->face->boundary) {
				//std::cout << "the corner normal is : " << h->corner_normal << std::endl;
				if (vi->bone_weights.empty()) {
					// Use the root bone's transformation matrix to transform the normal
					Mat4 inv_transpose = Mat4::transpose(bind_to_current[0].inverse());
					Vec3 transformed_normal = (inv_transpose * h->corner_normal).unit();
					skinned_normals.emplace(h, transformed_normal);
					//std::cout << " empty case, emplaced : " << transformed_normal << std::endl;
				}
				else {
					Vec3 norm_transform = Vec3(0.0f);
					for (const auto& bone_weight : vi->bone_weights) {
						if (bone_weight.weight > 0.0f) {
							Mat4 inv_transpose = Mat4::transpose(bind_to_current[bone_weight.bone].inverse());
							norm_transform += bone_weight.weight * (inv_transpose * h->corner_normal);
						}
					}
					Vec3 normal = norm_transform.unit();
					skinned_normals.emplace(h, normal);
					//std::cout << " non empty case, emplaced : " << normal << std::endl;
				}
			}

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	Indexed_Mesh result; //PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.
	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> indexes;

	for (Halfedge_Mesh::FaceCRef f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		if (f->boundary) continue;

		uint32_t corners_begin = static_cast<uint32_t>(verts.size());
		Halfedge_Mesh::HalfedgeCRef h = f->halfedge;
		do {
			Indexed_Mesh::Vert vert;
			vert.pos = skinned_positions.at(h->vertex);
			vert.norm = skinned_normals.at(h);
			vert.uv = h->corner_uv;
			vert.id = f->id;
			verts.emplace_back(vert);
			h = h->next;
		} while (h != f->halfedge);
		uint32_t corners_end = static_cast<uint32_t>(verts.size());

		for (size_t i = corners_begin + 1; i + 1 < corners_end; i++) {
			indexes.emplace_back(corners_begin);
			indexes.emplace_back(static_cast<uint32_t>(i));
			indexes.emplace_back(static_cast<uint32_t>(i + 1));
		}
	}
	result = Indexed_Mesh(std::move(verts), std::move(indexes));
	return result;
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
