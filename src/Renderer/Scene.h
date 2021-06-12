#pragma once

#include "Mesh.h"
#include <random>

struct AreaTriangle {
	const Triangle* T;
	float A;
};

struct PolygonalLight {
public:
	void SetPolygons(const std::vector<Triangle>& Geometry);

	std::pair<Vertex, const Triangle*> ChooseRandomPoint(float X, float R0, float R1) const ;
	float GetArea(void) const;

	const MaterialProperties* Material;
private:
	std::pair<Vertex, const Triangle*> GetRandomPoint(const AreaTriangle& AT, float R0, float R1) const;

	std::vector<AreaTriangle> SelectionTriangles;
	float Area;
};

// TODOL fix memory leak on program end
class Scene {
public:
	bool Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const;
	void LoadFile(const char* Path);
	// Get background radiance, typically from the sky. Samples for anything, such as a cubemap, or a sky radiance function 
	glm::vec3 GetSkyIrradiance(const Ray& R) const;

	const PolygonalLight* GetRandomLight(const float X) const;
	float GetTotalLightArea(void) const;

	bool ComputeVisibility(const Ray& R, const Triangle* T, const float L) const;
private:
	std::vector<Mesh*> Meshes;
	std::vector<PolygonalLight> Lights;

	float TotalLightArea;
};