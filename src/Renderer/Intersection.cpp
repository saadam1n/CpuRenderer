#include "Intersection.h"
#include <float.h> // FLT_MAX
#include <limits>

Intersection::Intersection(void) : Depth(std::numeric_limits<float>::max()) {}

void Intersection::Reset(void) {
	Depth = std::numeric_limits<float>::max();
}