#pragma once

#include "../Camera.h"
#include "../Scene.h"
#include "../Texture.h"
#include "../Color.h"

struct QualitySettings {};

struct PathTracerQuality {
	size_t SamplesPerPixel;
};

/*
A better name for this class would be renderer
But since we are in the field of computer graphics, I will call it the integrator (of the rendering and measurement equations)
*/
class Integrator {
public:
	/*
	Render an image of World using Eye into Image of Quality
	I simply leave it at a "RenderImage" function to allow for great flexibility 
	For example, some integrators require prepasses (such as photon tracing in photon mapping, or radiosity meshing) and that means adding a lot of functions like "PrepassN"
	Others don't even follow a traditional "eval Lo for pixel", for example, the MCMC MLT algorithm requires pixels to be randomly selected to integrate the image
	So the bottom line is that this is a very abstract concept-ish class to allow for a very, very, very wide variety of methods. In fact, go as far as writing very^infinity 
	*/
	// virtual void RenderImage(Texture<ColorRGB32F>& Image, const Camera& Eye, const Scene& World, const QualitySettings& Quality) = 0;
};