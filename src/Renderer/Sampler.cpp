#include "Sampler.h"

SamplerDefault::SamplerDefault(void) : UniformUnitF32(0.0f, 1.0f), Generator(2006) {}

void SamplerDefault::SetSeed(const uint32_t S) {
	Generator.seed(S);
}

float SamplerDefault::NextFloat(void) {
	return UniformUnitF32(Generator);
}