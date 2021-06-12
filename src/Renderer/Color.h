#pragma once

#include <stdint.h>
#include <glm/glm.hpp>

/*
template <typename T = float>
struct ColorRGB {
	T R, G, B;
};

using ColorRGB32F = ColorRGB<float>; 
using ColorRGB8UI = ColorRGB<uint8_t>;
*/

template<typename T>
using ColorRGB = glm::tvec3<T>;

using ColorRGB32F = ColorRGB<float>;
using ColorRGB8UI = ColorRGB<uint8_t>;