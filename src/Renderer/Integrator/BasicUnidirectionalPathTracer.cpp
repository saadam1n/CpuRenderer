#include "BasicUnidirectionalPathTracer.h"
#include <glm/gtc/random.hpp>
#include <thread>
#include <iostream>
#include <algorithm>
#include <iterator>

void NaiveUnidirectionalIntegrator::RenderImage(Texture<ColorRGB32F>& Image, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality) {
	for (size_t Y = 0; Y < Image.GetHeight(); Y++) {
		for (size_t X = 0; X < Image.GetWidth(); X++) {
			Image.WriteFlipped(X, Y, ComputeColor(Eye, World, Quality, (float)X / Image.GetWidth(), (float)Y / Image.GetHeight()));
		}
	}
}


// TODO: Stochastic anti aliasing
ColorRGB32F NaiveUnidirectionalIntegrator::ComputeColor(const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, const float X, const float Y) {
	Ray ViewRay = Eye.GenerateRay(X, Y);
	// Intersect with scene
	TriangleIntersection IntersectionInfo;
	if (!World.Intersect(ViewRay, IntersectionInfo)) {
		glm::vec3 SkyRadiance = World.GetSkyIrradiance(ViewRay);
		return { SkyRadiance.r, SkyRadiance.g, SkyRadiance.b };
	}
	// Get primary vertex on path and build tangent space to world space transform for hemisphere sampling
	Vertex PathVertex = IntersectionInfo.GetInterpolatedVertex();
	glm::vec3 Tangent = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), PathVertex.Normal);
	if (glm::dot(Tangent, Tangent) < 1e-3f) {
		Tangent = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), PathVertex.Normal);
	}
	Tangent = glm::normalize(Tangent);
	glm::mat3 TBN = glm::mat3(Tangent, glm::cross(Tangent, PathVertex.Normal), PathVertex.Normal);
	// Basic sky irradiance integration, no scene occlusion/interreflection taken into account for simplicity 
	glm::vec3 Radiance (0.0f);
	// Start monte carlo integration
	for (size_t Counter = 0; Counter < Quality.SamplesPerPixel; Counter++) {
		// Choose random pair of spherical coordiantes (theta, phi)
		glm::vec2 SphereCoords = glm::linearRand(glm::vec2(0.0f), glm::vec2(3.141529f));
		SphereCoords.y *= 2.0f;
		// Construct vector from it
		glm::vec3 Direction;
		Direction.y = glm::sin(SphereCoords.x);
		Direction.x = glm::cos(SphereCoords.x) * glm::cos(SphereCoords.y);
		Direction.x = glm::cos(SphereCoords.x) * glm::sin(SphereCoords.y);
		float Foreshortening = Direction.y;
		Direction = TBN * Direction;
		// Get radiance
		Ray SkyRay;
		SkyRay.Direction =  Direction;
		Radiance += Foreshortening * World.GetSkyIrradiance(SkyRay);
	}
	Radiance /= Quality.SamplesPerPixel;
	Radiance = PathVertex.Position;
	return { Radiance.r, Radiance.g, Radiance.b };
}
