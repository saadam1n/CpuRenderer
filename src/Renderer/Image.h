#pragma once

#include "Texture.h"
#include "Color.h"

namespace ImageIO {

	void CreateImage(const char* Path, Texture<ColorRGB32F>& ImgData);

}