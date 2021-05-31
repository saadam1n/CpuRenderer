#include <iostream>
#include "Renderer/Texture.h"
#include "Renderer/Image.h"
#include "Renderer/Scene.h"
#include "Renderer/Camera.h"
#include "Renderer/Integrator/BasicUnidirectionalPathTracer.h"

int main() {
	std::cout << "Hello World!\nRendering starting\n";

	Texture<ColorRGB32F> Image;
	Image.Create(1280, 720);
	Image.Clear({ 0.0f, 0.0f, 0.0f });

	Camera Eye;
	Eye.SetPosition(glm::vec3(2.0f, 0.0f, 10.0f) / 10.0f);
	Eye.UpdateImagePlaneParameters(16.0f / 9.0f, glm::radians(45.0f));

	// Rendering code
	Scene World;
	World.LoadFile("res/objects/BunnyLowRes.obj");

	NaiveUnidirectionalIntegrator PathTracer;
	PathTracerQuality Quality{ 8 };

	PathTracer.RenderImage(Image, Eye, World, Quality);

	ImageIO::CreateImage("Render.ppm", Image);

	std::cout << "Rendering finished\n";
}