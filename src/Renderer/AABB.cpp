#include "AABB.h"
#include <stdint.h>
#include <algorithm>
#include <iostream>

AABB::AABB(void) : Min(FLT_MAX), Max(-FLT_MAX) {}

void AABB::ExtendMax(const glm::vec3& Val) {
	Max = glm::max(Max, Val);
}

void AABB::ExtendMin(const glm::vec3& Val) {
	Min = glm::min(Min, Val);
}

void AABB::Extend(const glm::vec3& Pos) {
	ExtendMax(Pos);
	ExtendMin(Pos);
}

float AABB::SurfaceArea(void) const {
	/*
	An AABB is really just a fancy way to say "rectangular prism" (although with axis alignment restrictions, unless if it was an OBB)
	First let's define side lengths of a rectangular prism

	Equation 1:
	X = Max_X - Min_X
	Y = Max_Y - Min_Y
	Z = Max_Z - Min_Z

	Equation 1 is true as long as Max is always greater on any axis than Min, and in our case that's always true
	However in a case where the bounding box uses its default values (see constructor) then everything is broken.
	This can be fixed by the good old assert. Also, with GLM's vector math, we can just do XYZ = Max - Min

	Next we need to compute the surface area of the AABB. As we all know, area is width * length.
	Surface area is the summation of the area of all polygons on an object (at least in an object that isn't
	"continous" like a perfect sphere defined by a radius and position, but rather something like a discretized 
	set of triangles). A rectangular prism contains 6 rectangles, and the surface area is the summation of all of
	them. However, we can optimize this.

	Now there's probably a some sort of theorem for this, but I will just call it the "rectanglular prism oppisite 
	side rectangle theorm" or OSR for sort. In OSR, suppose we look at the rectangular prisim from one of the axes. 
	We will see a rectangle with a surface area which we will call A. Now we flip our directions and view the rectangle
	we now see, it will also be having a surface area of A. We can combine these two into a pair when computing surface 
	area. Instead of computing and adding the result of A twice, we could compute A once and add 2A. Another thing to note
	is that for each pair we multiply it by 2 before adding it. We can factor 2 out to reduce 2 multiplications. See equation
	2 for an optimized verion of surface area calculations and their derivations.

	Equation 2:
	SA = XY + XY + XZ + XZ + YZ + YZ
	SA = 2XY + 2XZ + 2YZ
	SA = 2(XY + XZ + YX)

	And there we have it. Fast surface area calculations. 
	*/

	return SurfaceAreaHalf() * 2.0f;
}

// Taken from madman's blog. Seriously, that guy has some really good stuff on BVHs
float AABB::SurfaceAreaHalf(void) const {
	glm::vec3 SideLengths = Max - Min;

	return
		SideLengths.x * (SideLengths.y + SideLengths.z) +
		SideLengths.y *  SideLengths.z;
}

void AABB::Extend(const AABB& BBox) {
	ExtendMax(BBox.Max);
	ExtendMin(BBox.Min);
}

AABB::AABB(const glm::vec3& Mi, const glm::vec3& Ma) : Min(Mi), Max(Ma) {}

float BoolToFloat(bool X) {
#if 1
	constexpr float Table[2] = { 0.0f, 1.0f };
	size_t Idx = (size_t)X;
	return Table[Idx];
#else
	return (float)X;
#endif
}

float MaxF(float A, float B) {
	float X = BoolToFloat(A > B);
	return glm::mix(A, B, X);
}

float MinF(float A, float B) {
	float X = BoolToFloat(A < B);
	return glm::mix(A, B, X);
}

glm::vec3 MaxF(const glm::vec3& A, const glm::vec3& B) {
	glm::vec3 C;

	C.x = MaxF(A.x, B.x);
	C.y = MaxF(A.y, B.y);
	C.z = MaxF(A.z, B.z);

	return C;
}

glm::vec3 MinF(const glm::vec3& A, const glm::vec3& B) {
	glm::vec3 C;

	C.x = MinF(A.x, B.x);
	C.y = MinF(A.y, B.y);
	C.z = MinF(A.z, B.z);

	return C;
}

#define USE_GLM_MIN_MAX

#ifdef USE_GLM_MIN_MAX

#define MIN_OF(X, Y) glm::min(X, Y)
#define MAX_OF(X, Y) glm::max(X, Y)

#else

#define MIN_OF(X, Y) MinF(X, Y)
#define MAX_OF(X, Y) MaxF(X, Y)

#endif

// 9 branches, really bad 
// (might take a look at "Fast Ray-Axis Aligned Bounding Box Overlap Tests with Plucker Coordinates" (https://pages.cpsc.ucalgary.ca/~blob/ps/jgt04.pdf) for faster intersection)
bool AABB::IntersectSlab(const Ray& R, AABBIntersection& IntersectionInfo) const {
	glm::vec3 MinPlaneT = Min * R.Direction + R.Origin; // 3 mul 3 add
	glm::vec3 MaxPlaneT = Max * R.Direction + R.Origin; // 3 mul 3 add

	glm::vec3 MinT = MIN_OF(MinPlaneT, MaxPlaneT);
	glm::vec3 MaxT = MAX_OF(MinPlaneT, MaxPlaneT);

	float Entry = MAX_OF(MinT.x, MAX_OF(MinT.y, MinT.z));
	float Exit  = MIN_OF(MaxT.x, MIN_OF(MaxT.y, MIN_OF(MaxT.z, IntersectionInfo.Depth)));

	Exit += 1e-3f; 

	if (Entry < Exit && Exit > 0.0f) { // 1 branch
		IntersectionInfo.Min = Entry;
		IntersectionInfo.Max = Exit;
		return true;
	}
	else {
		return false;
	}
}

/*
We compute the point that the ray intersects with each plane of the AABB
Then we test whether the point is on the AABB
*/
bool AABB::IntersectPoint(const Ray& R, AABBIntersection& IntersectionInfo) const {
	float Bounds[] = {
		Min[0],
		Min[1],
		Min[2],
		Max[0],
		Max[1],
		Max[2],
	};
	constexpr size_t BoundCount = sizeof(Bounds) / sizeof(float); // 6
	constexpr size_t VectorCount = BoundCount / 2;                // 3

	float Distances[6] = { 0.0f, 0.0f, 0.0f }; // 0 - Entry, 1 - Exit, 2 to 5 - Just in case a point passes through (a) corner(s)
	size_t DistanceIndex = 0;                  // Increment the index of distance we are writing to after every intersection
	bool ValidIntersection = false;            // Check if an intersection occured 

	// Avoid compliler warnings when casting from a larger to smaller type
	using VectorIndexT = glm::vec3::length_type;

	constexpr VectorIndexT BoundPairs[] = {
		1, 2, // X - Y & Z
		0, 2, // Y - X & Z
		0, 1, // Z - X & Y
	};

	// 0 branches with loop unrolling, 6 without
	for (size_t BoundIndex = 0; BoundIndex < BoundCount; BoundIndex++) {
		VectorIndexT ElementIndex = (VectorIndexT)BoundIndex % (VectorIndexT)VectorCount;

		// Find the point on the slab
		float CurrentDistance = Bounds[BoundIndex] * R.Direction[ElementIndex] + R.Origin[ElementIndex];
		glm::vec3 Point = R.Extend(CurrentDistance);

		// Obtain the bounds we have to check
		size_t PairIndex = 2 * (size_t)ElementIndex;
		VectorIndexT BoundCheckIndex[2];

		BoundCheckIndex[0] = BoundPairs[PairIndex    ];
		BoundCheckIndex[1] = BoundPairs[PairIndex + 1];

		// Check whether the point is on the AABB by checking bounds
		bool BoundPass[2];

		constexpr float kPlanePadding = 1e-3f;
		BoundPass[0] = Min[BoundCheckIndex[0]] < Point[BoundCheckIndex[0]] + kPlanePadding && Point[BoundCheckIndex[0]] - kPlanePadding < Max[BoundCheckIndex[0]];
		BoundPass[1] = Min[BoundCheckIndex[1]] < Point[BoundCheckIndex[1]] + kPlanePadding && Point[BoundCheckIndex[1]] - kPlanePadding < Max[BoundCheckIndex[1]];

		bool CurrentIntersection = BoundPass[0] && BoundPass[1];

		Distances[DistanceIndex] += CurrentDistance * (float)CurrentIntersection;
		DistanceIndex += (size_t)CurrentIntersection;

		ValidIntersection |= CurrentIntersection;
	}
	// 1 branch
	if (Distances[0] > Distances[1]) {
		std::swap(Distances[0], Distances[1]);
	}

	constexpr float kEpsilon = 1e-3f;
	// 1 branch
	if (
		ValidIntersection &&                      // An intersection occured
		Distances[0] > IntersectionInfo.Depth &&  // The intersection is not blocked by another intersection 
		Distances[1] > -kEpsilon)                 // The intersection is not behind us
	{
		IntersectionInfo.Min = Distances[0];
		IntersectionInfo.Max = Distances[1];
		
		return true;
	} else {
		return false;
	}
}

bool AABB::Intersect(const Ray& R, AABBIntersection& IntersectionInfo) const {
	return IntersectSlab(R, IntersectionInfo);
}

/*
union {
		float Values[7];
		glm::vec3 T[2];
	};

	T[0] = Min * R.Direction + R.Origin; // 3 mul 3 add
	T[1] = Max * R.Direction + R.Origin; // 3 mul 3 add
	Values[6] = IntersectionInfo.Depth;

	std::sort(Values, Values + 6);

	float Entry = Values[2];
	float Exit  = Values[3];
*/

/*
	glm::vec3 MinT = glm::min(MinPlaneT, MaxPlaneT); // 3 branches
	glm::vec3 MaxT = glm::max(MinPlaneT, MaxPlaneT); // 3 branches
*/

/*
// 3 branches
	if (MaxPlaneT.x < MinPlaneT.x) {
		std::swap(MaxPlaneT.x, MinPlaneT.x);
	}
	if (MaxPlaneT.y < MinPlaneT.y) {
		std::swap(MaxPlaneT.y, MinPlaneT.y);
	}
	if (MaxPlaneT.z < MinPlaneT.z) {
		std::swap(MaxPlaneT.z, MinPlaneT.z);
	}

	float Entry = glm::max(MinPlaneT.x, glm::max(MinPlaneT.y, MinPlaneT.z)); // 2 branches
	float Exit  = glm::min(MaxPlaneT.x, glm::min(MaxPlaneT.y, glm::min(MaxPlaneT.z, IntersectionInfo.Depth))); // 3 branches

	Exit += 1e-3f; // For numerical safety
*/