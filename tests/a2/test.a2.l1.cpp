#include "test.h"
#include "geometry/halfedge.h"
//#include <iostream>

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	//std::cout << "Initial mesh state:\n" << mesh.describe() << "\n";
	if (auto ret = mesh.flip_edge(edge)) {

		//std::cout << "Mesh state after flip_edge:\n" << mesh.describe() << "\n";
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_basic_simple("a2.l1.flip_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 2}, 
		{0, 2, 1}
	});

	expect_flip(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_edge_boundary("a2.l1.flip_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

/*
Edge CASE

Initial mesh:
0--1--2
|  |  |
|  |  |
|  6  |
|  |  |
|  |  |
3--4--5

Flip Edge on Edge: 1-5

After mesh:
0--1--2
|  |  |
|  |  |
|  6  |
|  |  |
|  |  |
3--4--5
*/
Test test_a2_l1_flip_edge_edge_case("a2.l1.flip_edge.case", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	 Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 0.0f, 0.0f),Vec3(4.0f, 0.0f, 0.0f),
	 Vec3(0.0f, 4.0f, 0.0f), Vec3(2.0f, 4.0f, 0.0f), Vec3(4.0f, 4.0f, 0.0f),
	 Vec3(2.0f,2.0f, 0.0f)
		}, {
		 {0, 1, 6, 4, 3},
		 {1, 2, 5, 4,6}
		});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work creates wrong mesh - square case.");
	}
	});

/*
Edge CASE

Initial mesh:
0--1--2
|  \  |
|    7|
|  /  |
| 6   |
|  \  |
3--4--5

Flip Edge on Edge: 6-7

After mesh:
0--1--2
|  \  |
|    7|
|  /  |
| 6   |
|  \  |
3--4--5
*/
Test test_a2_l1_flip_edge_zigzag_edge_case("a2.l1.flip_edge.case.zigzag", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	 Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 0.0f, 0.0f),Vec3(4.0f, 0.0f, 0.0f),
	 Vec3(0.0f, 4.0f, 0.0f), Vec3(2.0f, 4.0f, 0.0f), Vec3(4.0f, 4.0f, 0.0f),
	 Vec3(1.0f,1.0f, 0.0f),Vec3(3.0f,3.0f, 0.0f),
		}, {
		 {0, 1, 7, 6, 4, 3},
		 {1, 2, 5, 4,6,7}
		});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work creates wrong mesh - zigzag case.");
	}
	});

