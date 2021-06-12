#pragma once

#include "IntegratorBase.h"
#include "../Sampler.h"

/*
Type: Path tracer
Convergence: Not good, not bad
Bias status: Unbiased
Description: Basic unidirectional path tracer with cosine-weighted hemisphere sampling and NEE+MIS
Notes: Currently only supports pure diffuse, pure specular, and refractive surfaces. Physically based BRDFs are currently not avaible
*/
class NaiveUnidirectionalIntegrator : public Integrator {
public:
	NaiveUnidirectionalIntegrator(void);
	virtual void RenderImage(Texture<ColorRGB32F>& Image, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality);
private:
	ColorRGB32F ComputeColor(const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, const uint32_t Seed, const float SensorX, const float SensorY, const float X, const float Y);
	ColorRGB32F EvaluatePath(Ray& IrradianceRay, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, SamplerDefault& CurrentSampler);

	SamplerDefault Sampler;
};