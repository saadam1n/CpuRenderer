#pragma once

#include <stdint.h>

template <typename T = float>
struct ColorRGB {
	T R, G, B;
};

using ColorRGB32F = ColorRGB<float>; 
using ColorRGB8UI = ColorRGB<uint8_t>;