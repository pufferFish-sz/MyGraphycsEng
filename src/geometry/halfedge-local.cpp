
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
	   v4--v2\
		|  |  \
		|  |   v3
		|  |  /
	    x--v1/
 
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	auto update_face = [&](HalfedgeRef curr_edge, FaceRef target_face) {
		HalfedgeRef curr = curr_edge;
		do {
			curr->face = target_face;
			curr = curr->next;
		} while (curr != curr_edge);
		};

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	VertexRef v3 = t->next->next->vertex;
	VertexRef v4 = h->next->next->vertex;
	HalfedgeRef h_one_after = h->next;
	HalfedgeRef h_two_after = h->next->next;

	uint32_t steps = 0;
	HalfedgeRef t_one_before = t;
	while (t_one_before->next != t) {
		steps++;
		t_one_before = t_one_before->next;
	}
	
	HalfedgeRef t_two_before = t;
	for (uint32_t i = 0; i < steps - 1; ++i) {
		t_two_before = t_two_before->next;
	}

	// Phase 2: Allocate new elements, set data
	// First bisect the edge
	bisect_edge(e);
	//get the components after bisecting
	HalfedgeRef h_new = e->halfedge;
	HalfedgeRef h2 = h_new->next;
	VertexRef vm = h2->vertex;
	HalfedgeRef t2 = h_new->twin;
	HalfedgeRef t_new = h2->twin;
	
	EdgeRef e3 = emplace_edge();
	HalfedgeRef h3 = emplace_halfedge();
	interpolate_data({ h, h_two_after }, h3); //set corner_uv, corner_normal

	HalfedgeRef t3 = emplace_halfedge();
	interpolate_data({ h_one_after, h2 }, t3); //set corner_uv, corner_normal

	FaceRef f1_prime = emplace_face();

	// boundary case: doesn't subdivide faces
	if (f1->boundary || f2->boundary) {
		HalfedgeRef inner_halfedge;
		
		if (f1->boundary) {
			inner_halfedge = t2;
		}
		else {
			inner_halfedge = h2;
		}

		// e3
		e3->halfedge = h3;

		// h3
		h3->vertex = inner_halfedge->next->next->vertex; 
		h3->next = h2;
		h3->edge = e3;
		h3->twin = t3;

		// t3 
		t3->vertex = vm; 
		t3->next = h_two_after;
		t3->edge = e3;
		t3->twin = h3;

		h_one_after->next = h3;  
		h_new->next = t3; 

		// Update the face pointers
		update_face(h3, f1_prime);
		update_face(t3, f1);
		f1_prime->halfedge = h3;
		f1->halfedge = h_new;

		// Return the new midpoint vertex
		return vm;
	}

	// non boundary case: subdivide face
	HalfedgeRef h4 = emplace_halfedge();
	interpolate_data({ t_new, t_one_before }, h4); //set corner_uv, corner_normal

	EdgeRef e4 = emplace_edge();
	HalfedgeRef t4 = emplace_halfedge();
	interpolate_data({ t_two_before, t2 }, t4); //set corner_uv, corner_normal

	FaceRef f2_prime = emplace_face();

	// Phase 3: Reassign connectivity
	// e3
	e3->halfedge = h3;
	// e4
	e4->halfedge = t4;
	// h3
	h3->vertex = vm;
	h3->twin = t3;
	h3->next = h_two_after;
	h3->edge = e3;
	h3->face = f1;
	// t3
	t3->vertex = v4;
	t3->twin = h3;
	t3->next = h2;
	t3->edge = e3;
	t3->face = f1_prime;
	// t4
	t4->vertex = v3;
	t4->twin = h4;
	t4->next = t2;
	t4->edge = e4;
	t4->face = f2_prime;
	// h4
	h4->vertex = vm;
	h4->twin = t4;
	h4->next = t_one_before;
	h4->edge = e4;
	h4->face = f2;
	// f1
	f1->halfedge = h_new;
	// f2
	f2->halfedge = t_new;
	// f1_prime
	f1_prime->halfedge = h2;
	// f2_prime
	f2_prime->halfedge = t2;
	// h_new
	h_new->next = h3;
	// t_2_before
	t_two_before->next = t4;
	// h_1_after
	h_one_after->next = t3;
	// t_new
	t_new->next = h4;

	// Update the `face` pointers for the halfedges involved in f1 and f2
	update_face(h2, f1_prime);
	update_face(t2, f2_prime);
	
    return vm;
}

/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	auto update_face = [&](HalfedgeRef curr_edge, FaceRef target_face) {
		HalfedgeRef curr = curr_edge;
		do {
			curr->face = target_face;
			curr = curr->next;
		} while (curr != curr_edge);
		};

	auto figure_one_before = [](HalfedgeRef curr_edge) {
		HalfedgeRef curr = curr_edge;
		do {
			curr = curr->next;
		} while (curr->next != curr_edge);
		return curr;
		};

	// Phase 1: collect existing elements
	HalfedgeRef h = f->halfedge;
	HalfedgeRef t = h->twin;
	FaceRef outsideFace = t->face;
	EdgeRef e = h->edge;
	int iteration = 0;
	HalfedgeRef last_half_edge = t->next->twin;

	// refs to be stored and updated for later iteration use
	HalfedgeRef prev_t_new;
	HalfedgeRef prev_h_new;
	HalfedgeRef prev_ch2;
	HalfedgeRef prev_t;
	HalfedgeRef prev_h;
	HalfedgeRef first_t_new;
	HalfedgeRef first_t;
	HalfedgeRef first_ch2;
	VertexRef first_v_new;
	uint32_t first_v1_degree = 0;

	HalfedgeRef curr = h;
	do {
		//Step 1: make new edges, half edges, and vertices
		//get vertices on the edge
		VertexRef v1 = curr->vertex;
		t = curr->twin;
		VertexRef v2 = t->vertex;
		uint32_t v1_degree = v1->degree();

		//make a new vertex
		VertexRef v_new = emplace_vertex();
		v_new->position = v1->position;
		
		//make 2 new edges
		EdgeRef e_new = emplace_edge();
		e_new->sharp = e->sharp; //copy sharpness flag
		EdgeRef ce_new = emplace_edge();
		ce_new->sharp = e->sharp; //copy sharpness flag

		//make the 4 half edges
		HalfedgeRef h_new = emplace_halfedge();
		HalfedgeRef t_new = emplace_halfedge();
		HalfedgeRef ch1 = emplace_halfedge();
		HalfedgeRef ch2 = emplace_halfedge();

		//Step 2: connect them together
		v_new->halfedge = h_new; //v_new
		e_new->halfedge = h_new; //e_new
		ce_new->halfedge = ch1;
		//h_new
		h_new->vertex = v_new;
		//std::cout << "the h_new->vertex in current h of " << curr->id << "is " << h_new->vertex->id << std::endl;
		h_new->twin = t_new;
		h_new->edge = e_new; // h_new->next yet to be defined
		//t_new
		t_new->twin = h_new;
		t_new->edge = e_new;// t_new->vertex, t_new->next yet to be defined
		//ch1
		ch1->vertex = v1;
		ch1->twin = ch2;
		ch1->edge = ce_new;
		ch1->next = h_new;
		//ch2
		ch2->vertex = v_new;
		ch2->twin = ch1;
		ch2->edge = ce_new; //ch2->next yet to be defined
		
		HalfedgeRef prev_working_edge;
		HalfedgeRef rightmost_edge;
		HalfedgeRef top_edge;
		if (v1_degree > 2) {
			//search for rightmost boundary edge
			
			 //reconnect the surrounding edges to the new edges
			uint32_t curr_num_degree = v1_degree;
			HalfedgeRef working_edge = t->next;

			//loop thru the fan to find the one that is boundary
			rightmost_edge = working_edge;
			do {
				rightmost_edge = rightmost_edge->twin->next;
			} while (!rightmost_edge->face->boundary);

			do {
				//std::cout << "current working edge is" << working_edge->id << std::endl;
				if (working_edge->twin->next != curr && working_edge!=curr) {// not on the boundary of the extruded mesh
					//then reconnect it
					working_edge->vertex = v_new;
					// need to define working_edge->twin->next later
					//figure out if the connection is on the right or top side
					if (working_edge->twin->next->twin->next == curr) {// dis is right side
						if (iteration == 0) {
							prev_working_edge = working_edge;
						}
						else {
							working_edge->twin->next = prev_t_new;
							/*if (curr == last_half_edge) {
								std::cout << "it did come here right??? with curr id" << curr->id << std::endl;
								prev_working_edge->twin->next = first_t_new;
							}*/
						}
					}
					if (curr->twin->next == working_edge) { // dis is the top side
						//save this value
						top_edge = working_edge;
					}
				}
				curr_num_degree--;
				working_edge = working_edge->twin->next;
			} while (curr_num_degree > 2);
		}
		//t
		t->next = ch1;
		if (iteration == 0) {
			first_t_new = t_new;
			first_ch2 = ch2;
			first_v_new = v_new;
			first_v1_degree = v1_degree;
			first_t = t;
		}
		else {
			prev_h_new->next = ch2;
			prev_t_new->vertex = v_new;
			ch2->next = prev_t;
			//std::cout << "what is prev t ;-;" << prev_t->id << std::endl;
			if (prev_h->twin->face->boundary && v1_degree <=2) {
				t_new->next = prev_t_new; // this is dependent on whether prev_t is a boundary edge
			}
			else if (!curr->twin->face->boundary && v1_degree > 2) {
				t_new->next = top_edge; // need to find the working edge
			}
			else {
				t_new->next = rightmost_edge;
			}
			/*std::cout << "t_new is" << t_new->id << std::endl;
			std::cout << "and its next gets assigned as" << prev_t_new->id << std::endl;*/
			if (curr == last_half_edge) {
				/*std::cout << "first v1 degree is " << first_v1_degree << std::endl;
				std::cout << "prev_t is " << prev_t->id << std::endl;*/
				if (prev_h->twin->face->boundary && (first_v1_degree <= 2)) {
				/*	std::cout << "first_t_is " << first_t->id << std::endl;
					std::cout << "first_ch2 is " << t_new->id << std::endl;*/
					first_ch2->next = t;
					first_t_new->next = t_new; // this is dependent on whether prev_t is a boundary edge
				}
				h_new->next = first_ch2;

				t_new->vertex = first_v_new;
				//ch2->next = prev_t_new;
			}
		}
		//update the face of the new t
		t_new->face = t->face;
		t_new->face->halfedge = t_new;
		//update the stuff to save for next iteration
		prev_t_new = t_new;
		prev_h_new = h_new;
		prev_ch2 = ch2;
		prev_t = t;
		prev_h = h;
		curr = curr->next;
		iteration++;
		//std::cout << "this is iteration " << iteration << std::endl;
	} while (curr != h);

	//set the faces for all of them
	for (int i = 0; i < iteration; i++) {
		//std::cout << "this is making the " << i << "th face" << std::endl;
		FaceRef f_new = emplace_face();
		f_new->halfedge = h->twin;
		//std::cout << "It still prints stuff here right" << std::endl;
		update_face(h->twin, f_new);
		//update_face(h->twin->next->next->twin, outsideFace);
		h = h->next;
		//std::cout << "this is making the " << i << "th face" << std::endl;
	}

    return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	auto update_face = [&](HalfedgeRef curr_edge, FaceRef target_face) {
		HalfedgeRef curr = curr_edge;
		do {
			curr->face = target_face;
			curr = curr->next;
		} while (curr != curr_edge);
		};

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	uint32_t v1_degree = v1->degree();
	uint32_t v2_degree = v2->degree();
	// edge case 1: ensure the edge is not a boundary edge
	if (f1->boundary || f2->boundary) {
		return std::nullopt;
	}

	 //edge case 2: ensure edge flip doesn' distort faces
	 //check if the edge is flippable
	if ((v1_degree == 2) || (v2_degree == 2) ) {
		return std::nullopt; // Invalid flip
	}
	
	// Phase 3: Reassign connectivity
	//store edges refs for later use
	HalfedgeRef h_next = h->next;
	HalfedgeRef t_next = t->next;

	HalfedgeRef h_next_next = h->next->next;
	HalfedgeRef t_next_next = t->next->next;

	HalfedgeRef last_edgef1 = h;
	while (last_edgef1->next != h) {
		last_edgef1 = last_edgef1->next;  // Find the previous halfedge around face 1
	}

	HalfedgeRef last_edgef2 = t;
	while (last_edgef2->next != t) {
		last_edgef2 = last_edgef2->next;  // Find the previous halfedge around face 2
	}

    //reassign the vertices
	VertexRef v2_new = h->next->next->vertex;
	VertexRef v1_new = t->next->next->vertex;

	h->vertex = v1_new;
	t->vertex = v2_new;
	v1_new->halfedge = h;

	//reassign next pointers
	h->next = h_next_next;
	t->next = t_next_next;

	last_edgef1->next = t_next;
	last_edgef2->next = h_next;

	t_next->next = h;
	h_next->next = t;

	last_edgef1->next->vertex->halfedge = t_next;

	// Phase 3: Reassign the face's starting halfedges
	f1->halfedge = h;
	f2->halfedge = t;

	// Update the `face` pointers for the halfedges involved in f1 and f2
	update_face(h, f1);
	update_face(t, f2);

	return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge
	auto update_face = [&](HalfedgeRef curr_edge, FaceRef target_face) {
		HalfedgeRef curr = curr_edge;
		do {
			curr->face = target_face;
			curr = curr->next;
		} while (curr != curr_edge);
		};

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	auto face_is_triangle = [&](HalfedgeRef e_prime) -> bool {
		if (e_prime->next->next->next == e_prime) {
			return true;
		}
		else return false;
	};

	auto find_rightmost_edge = [&](HalfedgeRef working_edge) {
		HalfedgeRef rightmost_edge = working_edge;
		do {
			rightmost_edge = rightmost_edge->twin->next;
		} while (!rightmost_edge->face->boundary);
		return rightmost_edge;
		};

	auto find_prev_edge = [&](HalfedgeRef working_edge) {
		HalfedgeRef prev_edge = working_edge;
		do {
			prev_edge = prev_edge->next;
		} while (prev_edge->next != working_edge);
		return prev_edge;
		};

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	HalfedgeRef h_one_before = find_prev_edge(h);
	/*do {
		h_one_before = h_one_before->next;
	}while (h_one_before->next != h);*/
	HalfedgeRef t_next = t->next;
	HalfedgeRef h_one_after = h->next;
	HalfedgeRef t_of_h_one_after = h_one_after->twin;
	EdgeRef e_one_after = h_one_after->edge;

	Vec3 v1_new_position = (v1->position + v2->position) / 2.0f;

	uint32_t v2_degree = v2->degree();
	uint32_t v1_degree = v1->degree();

	//case 0: it is a single triangle: do nothing
	//every vertex has degree 2
	if (face_is_triangle(h)) {
		HalfedgeRef curr = h;
		int count = 0;
		do {
				uint32_t curr_degree = curr->vertex->degree();
				if (curr_degree == 2) {
					count++;
				}
				curr = curr->next;
		} while (curr != h);
		if (count == 3) {
			return std::nullopt;
		}
	}
	
	FaceRef newFace;
	FaceRef face_underneath;
	//case 1: one of the faces is a triangle and on the edge: need to delete more stuff
	if ((f1->boundary || f2->boundary) && (face_is_triangle(h) || face_is_triangle(t))) {
		
		HalfedgeRef curr = v2->halfedge;
		//std::cout << "v2 degree is " <<v2_degree<< std::endl;
		HalfedgeRef rightmost_edge = t->next;
		//check if there is a face beneath it
		if (v2_degree > 2) {
			HalfedgeRef last_h_in_neighbor = h_one_after->twin;
			face_underneath = h_one_after->twin->face;
			do {
				last_h_in_neighbor = last_h_in_neighbor->next;
			} while (last_h_in_neighbor->next != h_one_after->twin);
			for (uint32_t i = 0; i < v2_degree - 1; ++i) {
				//whatever is connected with v2 gets reconnected with v1
				if (curr != t && curr != h_one_after) {
					/*std::cout << "curr id is " << curr->id << std::endl;
					std::cout << "the curr twin vertex id is " << curr->twin->vertex->id << std::endl;*/
					curr->vertex = v1;
					
					//std::cout << "v1 halfedge now set to " << curr->id << std::endl;
					
					h_one_before->next = curr;
					
					curr->twin->next = t_next;
					
				}
				curr = curr->twin->next;
			}
			//std::cout << "t->next id" << t_next->id << std::endl;
			last_h_in_neighbor->next = h_one_before;
			//std::cout << "last h in neighbor id " << last_h_in_neighbor->id << std::endl;
			//std::cout << "h_one_before " << h_one_before->id << std::endl;
		}
		// if there isn't a face underneath, still delete the two edges of the triangle
		//erase the edge
		erase_edge(e);
		erase_halfedge(h);
		erase_halfedge(t);
		erase_vertex(v2);
		//erase the following one as well
		erase_edge(e_one_after);
		erase_halfedge(h_one_after);
		erase_halfedge(t_of_h_one_after);
		//erase out the face as well
		erase_face(f1);
		//new position
		v1->position = v1_new_position;
		//other linkages
		h_one_after->next->vertex->halfedge = h_one_before;
		f2->halfedge = rightmost_edge;
		//update face
		if (v2_degree > 2) {
			h_one_before->face = face_underneath;
		}
		else {
			update_face(h_one_before, f2);
		}
		v1->halfedge = h_one_before->twin;
		
		return v1;
	}

	//case 2: both faces of the given edge not a triangle 
	if (!face_is_triangle(h) && !face_is_triangle(t)) {
		if ((v2_degree <= 3) && (v1_degree <= 3)) { // case 3: hourglass shape, non manifold
		/*	unint32_t check1 = h->next->next->twin->face->id;
			unint32_t check2 = t->next->next->twin->face->id;*/
			if ((h->next->next->twin->face->boundary) && (t->next->next->twin->face->boundary)) {
				return std::nullopt;
			}
			/*if (check1 == check2) {
				return std::nullopt;
			}*/
		}
		//uint32_t v2_degree = v2->degree();
		HalfedgeRef curr = v2->halfedge;
		for (uint32_t i = 0; i < v2_degree - 1; ++i) {
			//whatever is connected with v2 gets reconnected with v1
			if (curr != t) {
				curr->vertex = v1;
				v1->halfedge = curr;
				if (i == 0) { // check the first one
					h_one_before->next = curr;
				}
				else if (i == v2_degree - 2) { // check the last one
					curr->twin->next = t_next;
				}
			}
			curr = curr->twin->next;
		}
		erase_edge(e);
		erase_halfedge(h);
		erase_halfedge(t);
		erase_vertex(v2);
		v1->position = v1_new_position;
		return v1;
	}
	//std::cout << "it came to reject! " << std::endl;
    return std::nullopt;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move

	 // calculate the centroid of the face
	Vec3 centroid(0.0f, 0.0f, 0.0f);
	HalfedgeRef h = face->halfedge;
	HalfedgeRef curr = h;
	int num_vertices = 0;
	HalfedgeRef revolving_edge = h;

	do {
		centroid += curr->vertex->position;
		num_vertices++;
		curr = curr->next;
	} while (curr != h);

	centroid /= static_cast<float>(num_vertices);

	//move each vertex based on shrink factor and move offset
	if (shrink >= 0) {
		curr = h;
		do {
			Vec3& pos = curr->vertex->position;

			if (shrink != 0) {
				// shrink towards or away from the centroid
				pos = centroid + shrink * (pos - centroid);
			}
			//std::cout << "the current shrink is: " << shrink << std::endl;
			// apply move offset
			pos += move;

			curr = curr->next;
		} while (curr != revolving_edge);

	}
	else {
		curr = h; 
		do {
			//std::cout << "ever came here?? the shrink is negative: "<< shrink << std::endl;
			//Vec3& pos = curr->twin->next->next->vertex->position;
			Vec3& pos = curr->vertex->position;

			// negative case
			pos = centroid + (1 - shrink) * (pos - centroid);

			pos += move;

			curr = curr->next;
		} while (curr != h);
	}


}

