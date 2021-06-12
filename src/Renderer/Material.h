#pragma once

#include <glm/glm.hpp>
#include "Texture.h"
#include "Color.h"

struct MaterialProperties {
	enum class SurfaceType {
		Emissive,   // Light source
		Diffuse,    // Pure diffuse
		Specular,   // Pure specular
		Refractive, // Varying refraction and reflection 
	};


	MaterialProperties(void);

	ColorRGB32F Diffuse;
	ColorRGB32F Emission;

	float RefractiveIndex;

	SurfaceType Type;
};


/*

	// Basic information of the BRDF
	enum class ReflectanceType {
		CUSTOM_COOK_TORRANCE,        // Use material textures for cook torrance BRDF (not yet implemented)
		PERFECT_DIFFUSE,             // Perfect diffuse (fr = 1 / PI)
		PERFECT_OPAQUE_SPECULAR,     // Perfect specular (ideal reflection using schlick's approximiation, no refraction)
		PERFECT_TRANSPARENT_SPECULAR // Perfect specular (ideal reflection and refraction using schlick's approximiation)
	} Type;

	bool EmitsLight;

	struct {
		// Base color. Used as diffuse color if albedo is not present
		ColorRGB32F Color;
		// Base emissive
		ColorRGB32F Emissive;
	} Base;

	struct {
		Texture<ColorRGB32F> Diffuse;
	} Textures;

	struct {
		ColorRGB32F RefractiveIndex;
		ColorRGB32F Extinction;
	} OpticalProperties;

	// Foreshortening sold separately 
	ColorRGB32F ComputeBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV);
private:
	ColorRGB32F SampleDiffuse(glm::vec2 UV);
	ColorRGB32F ComputeFresnel(const glm::vec3& N, const glm::vec3& H);

	ColorRGB32F ComputePerfectDiffuseBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV);
	ColorRGB32F ComputePerfectOpaqueSpecularBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV);
	ColorRGB32F ComputePerfectTransparentSpecularBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV);
};
*/