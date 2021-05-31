#pragma once

#include <glm/glm.hpp>
#include "Hittable.h"

struct AABBIntersection : public Intersection{
	float Min, Max;
};

struct AABB : public Hittable {
	AABB(void);
	AABB(const glm::vec3& Mi, const glm::vec3& Ma);

	void ExtendMax(const glm::vec3& Val);
	void ExtendMin(const glm::vec3& Val);

	void Extend(const glm::vec3& Pos);
	void Extend(const AABB& BBox);

	float SurfaceArea(void) const;
	float SurfaceAreaHalf(void) const;

	bool Intersect(const Ray& R, AABBIntersection& IntersectionInfo) const;

	glm::vec3 Min;
	glm::vec3 Max;
};
