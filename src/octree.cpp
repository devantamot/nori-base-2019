#include <vector> 
#include <nori/octree.h>
#include <typeinfo>
#include <algorithm>
#include <iterator>

using std::string;
//using namespace std;

NORI_NAMESPACE_BEGIN

/*-------------------------------------------------------------------------------------*/
// OCTTREE METHODS
Octree::Octree(Mesh *mesh) {
	m_globalBbox = mesh->getBoundingBox();
	m_root = nullptr;
	m_objMesh = mesh;
	d_leafCount = 0;
	d_innerCount = 0;
	cout << "Creating Octtree:\n\tBBox: " 
		<< m_objMesh->getBoundingBox().toString() << "\n\tVol : " 
		<< m_objMesh->getBoundingBox().getVolume() <<"\n";
}

void Octree::addTriangle(uint32_t t) {

	// If there is no node added, then this will add the first node
	if (m_root == nullptr) {
		m_root = new Octree::LeafNode(m_globalBbox, this, t);
	}
	else {
		m_root = m_root->addTriangle(t, this);
	}
}

int Octree::rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay) {
	//cout << "Starting ray search\n";
	return m_root->rayIntersect(ray, its, shadowRay, this);
}
/*-------------------------------------------------------------------------------------*/
// Abstract class for the InnerNode and LeafNode class

Octree::Node::Node(TBoundingBox<Point3f> b, Octree* m_octRef) { m_bbox = b; }

/*-------------------------------------------------------------------------------------*/
// INNER NODE METHODS
// Sets its own bounding box to the one passed in
// Creates 8 separate children bounding boxes as well 
Octree::InnerNode::InnerNode(TBoundingBox<Point3f> box, Octree* m_octRef) : Node(box, m_octRef) {

	// Initializes the children boxes from the corners to the center
	m_octRef->d_innerCount++;
	Point3f min = box.min, max = box.max, center = box.getCenter();

	// Bottom Front Left
	m_childBoxes[0] = new TBoundingBox<Point3f>(min, center);
	// Upper Front Left
	m_childBoxes[1] = new TBoundingBox<Point3f>(Point3f(min.x(),center.y(),min.z()), Point3f(center.x(), max.y(), center.z()));
	// Bottom Front Right
	m_childBoxes[2] = new TBoundingBox<Point3f>(Point3f(center.x(), min.y(), min.z()), Point3f(max.x(), center.y(), center.z()));
	// Upper Front Right
	m_childBoxes[3] = new TBoundingBox<Point3f>(Point3f(center.x(), center.y(), min.z()), Point3f(max.x(), max.y(), center.z()));

	// Bottom Back Left
	m_childBoxes[4] = new TBoundingBox<Point3f>(Point3f(min.x(), min.y(), center.z()), Point3f(center.x(), center.y(), max.z()));
	// Upper Back Left
	m_childBoxes[5] = new TBoundingBox<Point3f>(Point3f(min.x(), center.y(), center.z()), Point3f(center.x(), max.y(), max.z()));
	// Bottom Back Right
	m_childBoxes[6] = new TBoundingBox<Point3f>(Point3f(center.x(), min.y(), center.z()), Point3f(max.x(), center.y(), max.z()));
	// Upper Back Right
	m_childBoxes[7] = new TBoundingBox<Point3f>(center, max);

	// Sets all child pointers to null
	for (int i = 0; i < 8; i++) {
		m_children[i] = nullptr;
	}
}

// Checks each bounding box to see if the triangle belongs in there.
Octree::Node* Octree::InnerNode::addTriangle(uint32_t t, Octree* m_octRef) {

	for (int i = 0; i < 8; i++) {

		if (m_childBoxes[i]->overlaps(m_octRef->m_objMesh->getBoundingBox(t))) {

			if (m_children[i] == nullptr) { 
				m_children[i] = new Octree::LeafNode(*m_childBoxes[i], m_octRef, t);
			}	
			else { m_children[i] = m_children[i]->addTriangle(t, m_octRef); }
		}
	}

	return this;
}

int Octree::InnerNode::rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay, Octree* m_octRef) {

	int triIntersect[8], closest = -1, oddIndexing[8] = { 0,1,2,3,4,5,6,7 };

	// now sort them based on which ones are closest to the ray origin
	std::sort(std::begin(oddIndexing), std::end(oddIndexing), InnerNode::BBoxSort(ray.o, m_childBoxes));

	for (int i = 0; i < 8; i++) {
		if (m_childBoxes[oddIndexing[i]]->rayIntersect(ray) && (m_children[oddIndexing[i]] != nullptr)) {

			triIntersect[oddIndexing[i]] = m_children[oddIndexing[i]]->rayIntersect(ray, its, shadowRay, m_octRef);
			if (triIntersect[oddIndexing[i]] > -1) return triIntersect[oddIndexing[i]];
			else if (triIntersect[oddIndexing[i]] == -2) return -2;
		}
		else {
			triIntersect[oddIndexing[i]] = -1;
		}
	}

	return closest;
}

/*-------------------------------------------------------------------------------------*/
// LEAF NODE METHODS
Octree::LeafNode::LeafNode(TBoundingBox<Point3f> box, Octree* m_octRef, uint32_t t) : Node(box, m_octRef) {

	m_octRef->d_leafCount++;
	m_triangles.push_back(t);
}

Octree::Node* Octree::LeafNode::addTriangle(uint32_t t, Octree* m_octRef) {

	m_triangles.push_back(t);

	// If there are already 10 triangles in here and the bounding box isn't too small, then just make a new
	// Inner node and return it
	if (m_triangles.size() > 10 && m_bbox.getVolume() > MIN_VOLUME) {

		Node *inner = new InnerNode(Node::m_bbox, m_octRef); // create a new inner node with this nodes bounding box
		m_octRef->d_leafCount--;
		for (int i = 0; i < m_triangles.size(); i++) {
			inner = inner->addTriangle(m_triangles.at(i), m_octRef);
		}
		m_triangles.clear();
		return inner;
	}

	return this;
}

int Octree::LeafNode::rayIntersect(const Ray3f& ray_, Intersection& its, bool shadowRay, Octree* m_octRef) {

	int closest = -1;
	Ray3f ray(ray_);

	for (int i = 0; i < m_triangles.size(); i++) {
		float u, v, t;

		if (m_octRef->m_objMesh->rayIntersect(m_triangles.at(i), ray, u, v, t)) {

			if (shadowRay) return -2;

			ray.maxt = its.t = t;
			its.uv = Point2f(u, v);
			its.mesh = m_octRef->m_objMesh;

			closest = m_triangles.at(i);
		}
	}

	return closest;
}

NORI_NAMESPACE_END