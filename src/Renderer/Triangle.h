#pragma once

#include "Hittable.h"
#include "Vertex.h"

struct Triangle;

struct TriangleIntersection : public Intersection {
	const Triangle* Surface; // Pointer to triangle
	float U, V;              // Barycentric interpol coeffs
	// Deferred interpolation to prevent computations of occluded triangles
	Vertex GetInterpolatedVertex(void) const;
};

struct Triangle : public Hittable {
	// Currently indexing is disabled since normal idxing leads to energy loss in path tracing 
	// See the paper "Microfacet Normal Mapping" for an explaination and possible solution
	Vertex Vertices[3];
	bool Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const;
};