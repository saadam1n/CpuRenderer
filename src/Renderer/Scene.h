#pragma once

#include "Mesh.h"

class Scene {
public:
	bool Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const;
	void LoadFile(const char* Path);
	// Get background radiance, typically from the sky. Samples for anything, such as a cubemap, or a sky radiance function 
	glm::vec3 GetSkyIrradiance(const Ray& R) const;
private:
	std::vector<Mesh> Meshes;
};