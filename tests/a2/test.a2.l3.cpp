#include "test.h"
#include "geometry/halfedge.h"
#include <iostream>

static void expect_collapse(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	//std::cout << "Initial mesh state:\n" << mesh.describe() << "\n";
	if (auto ret = mesh.collapse_edge(edge)) {
		//std::cout << "Mesh state after split_edge:\n" << mesh.describe() << "\n";
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("collapse_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse Edge on Edge: 2-3

After mesh:
0-----1\
 \   /  \
  \ /    \
   2------3
  / \    /
 /   \  /
4-----5/
*/
Test test_a2_l3_collapse_edge_basic_simple("a2.l3.collapse_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 Vec3(-1.2f, 0.0f, 0.0f),   	 Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 3, 1}, 
		{2, 5, 6, 3}, 
		{1, 3, 4}, 
		{3, 6, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 			Vec3(0.0f, 0.0f, 0.0f),  			Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 5}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_collapse(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 0-1

After mesh:
    0--\
   / \  \
  /   \  \
 /     1--2
/      | /
3------4/
*/
Test test_a2_l3_collapse_edge_edge_boundary("a2.l3.collapse_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		       Vec3(0.05f, 1.05f, 0.0f), 
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 1, 2}, 
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});

	expect_collapse(mesh, edge, after);
});

/*
EDGE CASE single triangle

Initial mesh:
0
|\
| \
|  \
1---2

Collapse Edge on Edge: 0-2

After mesh:
0
|\
| \
|  \
1---2

(reject)
*/
Test test_a2_l3_collapse_edge_triangle("a2.l3.collapse_edge.triangle", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	 Vec3(0.0f, 2.0f, 0.0f),
	 Vec3(0.0f, 0.0f, 0.0f),  Vec3(2.0f, 0.0f, 0.0f)
		}, {
		 {0, 2, 1}
		});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

	if (mesh.collapse_edge(edge)) {
		throw Test::error("collapse_edge should not work for single triangles.");
	}
	});


/*
Edge CASE

Initial mesh:
0--1--2
|  |  |
|  |  |
3--4--5

Edge Collapse on Edge: 1-4

After mesh:
0--1--2
|  |  |
|  |  |
3--4--5

(reject)
*/
Test test_a2_l3_collapse_edge_square("a2.l3.collapse_edge.square", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 0.0f, 0.0f),Vec3(4.0f, 0.0f, 0.0f),
	Vec3(0.0f, 4.0f, 0.0f), Vec3(2.0f, 4.0f, 0.0f), Vec3(4.0f, 4.0f, 0.0f)
		}, {
		{0, 1, 4, 3},
		{1, 2, 5, 4}
		});
	// std::cout
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.collapse_edge(edge)) {
		throw Test::error("collapse_edge should not work creates non manifold mesh - square case.");
	}
	});
