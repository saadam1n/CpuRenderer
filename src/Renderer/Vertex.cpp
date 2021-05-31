#include "Vertex.h"

Vertex::Vertex(const glm::vec3& P, const glm::vec3& N, const glm::vec2& TC) : Position(P), Normal(N), TextureCoordinates(TC) {}
Vertex::Vertex(void) : Vertex(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec2(0.0f)) {}

Vertex Vertex::operator*(float Value) const {
	Vertex NewVertex;

	NewVertex.Position = Position * Value;
	NewVertex.Normal = Normal * Value;
	NewVertex.TextureCoordinates = TextureCoordinates * Value;

	return NewVertex;
}

Vertex Vertex::operator+(const Vertex& Value) const {
	Vertex NewVertex;

	NewVertex.Position = Position + Value.Position;
	NewVertex.Normal = Normal + Value.Normal;
	NewVertex.TextureCoordinates = TextureCoordinates + Value.TextureCoordinates;

	return NewVertex;
}