#include <iostream>
#include "Renderer/Texture.h"
#include "Renderer/Image.h"
#include "Renderer/Scene.h"
#include "Renderer/Camera.h"
#include "Renderer/Integrator/UnidirectionalPathTracer.h"
#include "Renderer/TimeUtil.h"
#include "Renderer/Constants.h"

constexpr size_t Width = 640;
constexpr size_t Height = 480;

/*
TODO:

self reminder

refractor code:
- Organize files
- Organize uni path tracer
- Proper BRDF and stuff
- Base it off nori or PBRT what ever is better
- DONE: Get NEE working properly

stuff to do:
- DONE: MIS
- BDPT integrator (will still have NEE)
- Photon map integrator
- Possible even radiosity
*/

int main() {
	std::cout << "Hello World!\nRendering starting\n";
	Timer TotalTime;
	TotalTime.Begin();

	Texture<ColorRGB32F> Image;
	Image.Create(Width, Height);
	Image.Clear({ 0.0f, 0.0f, 0.0f });

	Camera Eye;
	Eye.SetPosition(glm::vec3(0.0f, 0.75f, 2.5f)); //  + glm::vec3(0.75f, -0.71f, -1.65f)
	//Eye.AddRotation(glm::vec3(-Constants::kPi / 4.0f, Constants::kPi / 8.0f, 0.0f));
	Eye.GenerateViewTransform();
	Eye.UpdateImagePlaneParameters((float)Width / (float)Height, glm::radians(45.0f));

	// Rendering code
	Scene World;
	World.LoadFile("res/objects/Cornell/CornellBox-Sphere.obj");

	NaiveUnidirectionalIntegrator PathTracer;
	PathTracerQuality Quality;

	Quality.SamplesPerPixel = 512;
	Quality.NextEventEstimationSamples = 1;

	PathTracer.RenderImage(Image, Eye, World, Quality);

	ImageIO::CreateImage("Render.ppm", Image);

	std::cout << "Rendering finished\n";
	TotalTime.End();
	TotalTime.DebugTime();
}

/*
	AABB TestAABB;

	TestAABB.Min = glm::vec3(0.0f);
	TestAABB.Max = glm::vec3(1.0f);

	Ray TestRay;

	TestRay.Origin = glm::vec3(0.5f, 0.5f, 2.0f);
	TestRay.Direction = glm::vec3(0.0f, 0.0f, -1.0f);

	AABBIntersection FakeIntersection;

	std::cout << TestAABB.Intersect(TestRay, FakeIntersection);

	return 0;
*/