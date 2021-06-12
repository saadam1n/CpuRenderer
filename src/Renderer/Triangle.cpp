#include "Triangle.h"
#include <limits>
#include <iostream>

Vertex TriangleIntersection::GetInterpolatedVertex(void) const {
	Vertex InterpolatedVertex =

		Surface->Vertices[0] * (1.0f - U - V) +
		Surface->Vertices[1] * U +
		Surface->Vertices[2] * V

		;

	InterpolatedVertex.Normal = glm::normalize(InterpolatedVertex.Normal);

	//if (Backface) {
	//	InterpolatedVertex.Normal = -InterpolatedVertex.Normal;
	//}

	return InterpolatedVertex;
}

bool Triangle::Intersect(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const {
#if 1
	return IntersectMollerTrumbore(R, IntersectionInfo, Material);
#else
	return IntersectPlaneBarycentric(R, IntersectionInfo, Material);
#endif
}

bool Triangle::IntersectMollerTrumbore(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const {
	glm::vec3 Edge1 = Vertices[1].Position - Vertices[0].Position;
	glm::vec3 Edge2 = Vertices[2].Position - Vertices[0].Position;

	glm::vec3 Tv = R.Origin - Vertices[0].Position;
	glm::vec3 Pv = glm::cross(R.Direction, Edge2);
	glm::vec3 Qv = glm::cross(Tv, Edge1);

	float Determinant = glm::dot(Pv, Edge1);
	float DeterminantRCP = 1.0f / Determinant;

	float T = DeterminantRCP * glm::dot(Qv, Edge2);
	float U = DeterminantRCP * glm::dot(Tv, Pv);
	float V = DeterminantRCP * glm::dot(Qv, R.Direction);

	bool Backface = false;

	constexpr float kEpsilon = 1e-3f;
	if (
		(T < IntersectionInfo.Depth && T > kEpsilon) &&
		!(U < 0.0f || U > 1.0f) &&
		!(V < 0.0f || U + V  > 1.0f) &&
		!(Determinant < 0.0f ? [](float& Det, bool& Back) -> float {Back = true; return -Det; }(Determinant, Backface) : Determinant < std::numeric_limits<float>::epsilon())
		) {
		IntersectionInfo.Depth = T;

		IntersectionInfo.Surface = this;
		IntersectionInfo.U = 1.0f;
		IntersectionInfo.V = 0.0f;

		IntersectionInfo.Backface = Backface;

		IntersectionInfo.Material = Material;

		return true;
	}
	else {
		return false;
	}
}

/*
Intersect the plane represented by the triangle
Then compute the barycentric coordinates to test whether it is inside or outside of it
*/
bool Triangle::IntersectPlaneBarycentric(const Ray& R, TriangleIntersection& IntersectionInfo, const MaterialProperties* Material) const {
	glm::vec3 Edge1 = Vertices[1].Position - Vertices[0].Position;
	glm::vec3 Edge2 = Vertices[2].Position - Vertices[0].Position;

	glm::vec3 Normal = glm::cross(Edge1, Edge2);
	Normal = glm::normalize(Normal);

	float PlaneDistance = glm::dot(Normal, Vertices[0].Position - R.Origin);
	float CosineTheta   = glm::dot(Normal, R.Direction);

	float T = PlaneDistance / CosineTheta;
	glm::vec3 P = R.Extend(T);

	// Current solve the system by hand instead of using Cramer's rule

	return false;
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

/*


*/

/*
	glm::vec3 V01 = Vertices[1].Position - Vertices[0].Position;
	glm::vec3 V02 = Vertices[2].Position - Vertices[0].Position;

	glm::vec3 Tv = R.Origin - Vertices[0].Position;
	glm::vec3 Qv = glm::cross(Tv, V01);
	glm::vec3 Pv = glm::cross(V02, R.Direction);

	float Determinant    = glm::dot(V01, Pv);
	float DeterminantRCP = 1.0f / Determinant;

	float T = DeterminantRCP * glm::dot(Qv, V02);
	float U = DeterminantRCP * glm::dot(Tv, Pv);
	float V = DeterminantRCP * glm::dot(Qv, R.Direction);

	if (
		(T < IntersectionInfo.Depth && T > 0.0f) &&
		!(U < 0.0f || U > 1.0f) &&
		!(V < 0.0f || U + V  > 1.0f) &&
		!(fabsf(Determinant) < std::numeric_limits<float>::epsilon())
		) {
		IntersectionInfo.Depth = T;

		IntersectionInfo.Surface = this;
		IntersectionInfo.U = 1.0f;
		IntersectionInfo.V = 0.0f;

		IntersectionInfo.Material = Material;

		return true;
	} else {
		return false;
	}
*/