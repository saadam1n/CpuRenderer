#pragma once

#include "Hittable.h"
#include "Vertex.h"
#include "Material.h"

struct Triangle;
struct MaterialProperties;

struct TriangleIntersection : public Intersection {
	const Triangle* Surface; // Pointer to triangle
	float U, V;              // Barycentric interpol coeffs
	const MaterialProperties* Material; // Intersected material
	bool Backface;  // Did the ray hit the triangle on the front or back side?
	// Deferred interpolation to prevent computations of occluded triangles
	Vertex GetInterpolatedVertex(void) const;
};

struct Triangle : public Hittable {
	// Currently indexing is disabled since normal idxing leads to energy loss in path tracing 
	// See the paper "Microfacet Normal Mapping" for an explaination and possible solution
	Vertex Vertices[3];
	bool Intersect(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const;
private:
	// Cramer's rule
	bool IntersectMollerTrumbore(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const;
	// Intersect plane
	bool IntersectPlaneBarycentric(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const;
};