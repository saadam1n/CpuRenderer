#pragma once

#include <glm/glm.hpp>

struct Vertex {
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec2 TextureCoordinates;

	Vertex(const glm::vec3& P, const glm::vec3& N, const glm::vec2& TC);
	Vertex(void);

	Vertex operator*(float Value)         const;
	Vertex operator+(const Vertex& Value) const;
};