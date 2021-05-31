#include "Triangle.h"

Vertex TriangleIntersection::GetInterpolatedVertex(void) const {
	Vertex InterpolatedVertex =

		Surface->Vertices[0] * (1.0f - U - V) +
		Surface->Vertices[1] * U +
		Surface->Vertices[2] * V

		;

	InterpolatedVertex.Normal = glm::normalize(InterpolatedVertex.Normal);

	return InterpolatedVertex;
}

bool Triangle::Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const {
	glm::vec3 V01 = Vertices[1].Position - Vertices[0].Position;
	glm::vec3 V02 = Vertices[2].Position - Vertices[0].Position;

	glm::vec3 Pvec = glm::cross(R.Direction, V02);
	float Determinant = glm::dot(V01, Pvec);

	constexpr float kEpsilon = 1e-6f;
	if (fabsf(Determinant) < kEpsilon) {
		return false;
	}

	float DeterminantRCP = 1.0f / Determinant;

	glm::vec3 Tvec = R.Origin - Vertices[0].Position;
	float U = glm::dot(Tvec, Pvec) * DeterminantRCP;

	if (U < 0.0f || U > 1.0f) {
		return false;
	}

	glm::vec3 Qvec = glm::cross(Tvec, V01);
	float V = glm::dot(R.Direction, Qvec) * DeterminantRCP;

	if (V < 0.0f || U + V  > 1.0f) {
		return false;
	}

	float T = glm::dot(V02, Qvec) * DeterminantRCP;

	if (T < IntersectionInfo.Depth && T > 0.0f) {
		IntersectionInfo.Depth = T;

		IntersectionInfo.Surface = this;
		IntersectionInfo.U = U;
		IntersectionInfo.V = V;

		return true;
	} else {
		return false;
	}
}

/*
This didn't seem to work and instead seemed to slow things down
I'm still sure it benefits a parallel processor like the GPU

	// Minimize branch prediction but putting all if statements here
	if (
		(fabsf(Determinant) > kEpsilon) &&        // Numerical stability of inv det
		(U > 0.0f || U < 1.0f) &&                // Valid range  for U
		(V > 0.0f || U + V  < 1.0f) &&           // Valid ranges for V
		(T < IntersectionInfo.Depth && T > 0.0f)  // Unoccluded triangle in front of the rayu
		) {
		IntersectionInfo.Depth = T;

		IntersectionInfo.Surface = this;
		IntersectionInfo.U = U;
		IntersectionInfo.V = V;

		return true;
	}
*/