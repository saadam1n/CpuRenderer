#pragma once

#include "IntegratorBase.h"
/*
Type: Path tracer
Convergence: *cricket noises*
Bias status: Unbiased
Description: Basic unidirectional path tracer with hemisphere sampling
Notes: Does not have any importance sampling of any kind (NEE or BRDF). Only pure diffuse surfaces are supported, although pure specular support should come soon
*/
class NaiveUnidirectionalIntegrator : public Integrator {
public:
	virtual void RenderImage(Texture<ColorRGB32F>& Image, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality);
private:
	ColorRGB32F ComputeColor(const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, const float X, const float Y);
};