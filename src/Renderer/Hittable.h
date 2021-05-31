#pragma once

#include "Intersection.h"
#include "Ray.h"

// Base structure for hittable objects (not virtual to save memory space)

struct Hittable {
	bool Intersect(const Ray& R, Intersection& IntersectInfo) const = delete;
};