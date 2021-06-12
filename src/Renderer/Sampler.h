#pragma once

#include <random>

class SamplerDefault {
public:
	SamplerDefault(void);
	void SetSeed(const uint32_t S); // Reseed the generator

	float NextFloat(void);
private:
	std::mt19937 Generator;
	std::uniform_real_distribution<float> UniformUnitF32;
};