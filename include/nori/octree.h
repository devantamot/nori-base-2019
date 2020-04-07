#pragma once
#include <vector> 
#include <nori/mesh.h>
#include <nori/bbox.h>

using std::vector;

#define MIN_VOLUME 0.05f

NORI_NAMESPACE_BEGIN
/*-------------------------------------------------------------------------------------*/

class Octree {

public:
	Octree(Mesh *mesh);

	TBoundingBox<Point3f> m_globalBbox; // The global bound of the octtree
	Mesh *m_objMesh;
	int d_leafCount, d_innerCount;
	void addTriangle(uint32_t t);
	int rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay);

protected:
	class Node;
	class InnerNode;
	class LeafNode;
	Node *m_root; // reference to the root node of this octTree

	// Abstract class for the InnerNode and LeafNode class
	class Node {
	public:
		TBoundingBox<Point3f> m_bbox;

		Node(TBoundingBox<Point3f> b, Octree *oct);


		virtual Node* addTriangle(uint32_t t, Octree* m_octRef) = 0;

		virtual int rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay, Octree* m_octRef) = 0;
	};

	/*-------------------------------------------------------------------------------------*/
// Class for nodes that have children
	class InnerNode : public Node {
	public:
		// Sets its own bounding box to the one passed in
		// Creates 8 separate children bounding boxes as well 
		InnerNode(TBoundingBox<Point3f> box, Octree* oct);

		// Checks each bounding box to see if the triangle belongs in there.
		Node* addTriangle(uint32_t t, Octree* m_octRef);
		int rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay, Octree* m_octRef);

		struct BBoxSort {
			BBoxSort(Point3f ref, TBoundingBox<Point3f>** childs) { this->ref = ref; this->childs = childs; }
			bool operator () (int a, int b) {
				return childs[a]->distanceTo(ref) < childs[b]->distanceTo(ref);
			}

			Point3f ref;
			TBoundingBox<Point3f>** childs;
		};

	protected:
		Node* m_children[8]; // All the children of the inner node
		TBoundingBox<Point3f>* m_childBoxes[8]; // The bounding boxes for the children
	};

	/*-------------------------------------------------------------------------------------*/
	// Has no children but has up to 10 trianlges
	class LeafNode : public Node {
	public:

		vector<uint32_t> m_triangles;

		LeafNode(TBoundingBox<Point3f> box, Octree* oct);
		LeafNode(TBoundingBox<Point3f> box, Octree* oct, uint32_t t);
		

		Node* addTriangle(uint32_t t, Octree* m_octRef);
		int rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay, Octree* m_octRef);

	};
};



NORI_NAMESPACE_END