#include "Material.h"
#include "Constants.h"
#include <stdexcept>

MaterialProperties::MaterialProperties(void) : Type(SurfaceType::Diffuse), Emission(0.0f) {}

/*
ColorRGB32F Material::ComputeBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV) {
	switch (Type) {
	case ReflectanceType::PERFECT_DIFFUSE: {
		return ComputePerfectDiffuseBRDF(Wi, Wo, N, UV);
	} break;
	case ReflectanceType::PERFECT_OPAQUE_SPECULAR: {
		return ComputePerfectOpaqueSpecularBRDF(Wi, Wo, N, UV);
	} break;
	case ReflectanceType::PERFECT_TRANSPARENT_SPECULAR: {
		return ComputePerfectTransparentSpecularBRDF(Wi, Wo, N, UV);
	} break;
	default: {
		throw std::runtime_error("Unsupported BRDF type");
	}break;
	}
}

ColorRGB32F Material::SampleDiffuse(glm::vec2 UV) {
	if (Textures.Diffuse.GetRawData()) {
		return Textures.Diffuse.Sample(UV);
	} else {
		return Base.Color;
	}
}

ColorRGB32F Material::ComputeFresnel(const glm::vec3& N, const glm::vec3& H) {
	constexpr ColorRGB32F AirRefractiveIndex(1.0f);

	ColorRGB32F R0 = (OpticalProperties.RefractiveIndex - AirRefractiveIndex) / (OpticalProperties.RefractiveIndex + AirRefractiveIndex);
	R0 *= R0;

	float Angular = 1.0f - glm::dot(N, H);
	float Angular2 = Angular * Angular;
	float Angular3 = Angular2 * Angular;

	return R0 + (1.0f - R0) * (Angular2 * Angular3);
}

ColorRGB32F Material::ComputePerfectDiffuseBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV) {
	return SampleDiffuse(UV) / Constants::kPi;
}

ColorRGB32F Material::ComputePerfectOpaqueSpecularBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV) {
	return glm::vec3(0.0f);
}

ColorRGB32F Material::ComputePerfectTransparentSpecularBRDF(const glm::vec3& Wi, const glm::vec3& Wo, const glm::vec3& N, const glm::vec2& UV) {
	return glm::vec3(0.0f);
}

*/
