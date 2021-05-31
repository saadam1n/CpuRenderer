#pragma once

#include "Triangle.h"
#include "BVH.h"
#include <vector>


// A mesh is simply a manifold created by a union of multiple triangles
class Mesh : public Hittable {
public:
	// Sets a union (the math union) of triangles to the mesh's geometry
	void SetTriangles(const std::vector<Triangle>& TriangleUnion);
	// Intersect with all the triangles contained within the mesh
	bool Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const;
private:
	bool IntersectLeaf(const NodeSerialized& N, const Ray& R, TriangleIntersection& IntersectionInfo) const;
	// I don't really need the versatility of a vector here but it's much easier than keeping track of everything like size yourself 
	// REMINDER: DO NOT CAUSE ALLOCATIONS SINCE IntersectionInfo RELIES ON POINTERS TO TRIANGLES
	std::vector<Triangle> Triangles;
	// BVH for the mesh
	BoundingVolumeHierarchy BVH;
};