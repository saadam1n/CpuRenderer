#include "UnidirectionalPathTracer.h"
#include "../Constants.h"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <thread>
#include <atomic>
#include <iomanip>

std::ostream& operator<<(std::ostream& os, const glm::vec3& m) {
	return os << '(' << m.x << ", " << m.y << ", " << m.z << ')';
}

NaiveUnidirectionalIntegrator::NaiveUnidirectionalIntegrator(void) {}

void NaiveUnidirectionalIntegrator::RenderImage(Texture<ColorRGB32F>& Image, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality) {
	//std::cout << std::fixed;
	//std::cout << std::setprecision(3);

	float SensorX = 1.0f / Image.GetWidth ();
	float SensorY = 1.0f / Image.GetHeight();

	// Multithreaded code
	auto ImageTask = [this, Eye, World, Quality, SensorX, SensorY](Texture<ColorRGB32F>& Image, const size_t NumTasks, std::atomic<size_t>& TaskIndex) -> void {
		size_t CurrentPos;
		for (; TaskIndex < NumTasks;) {
			CurrentPos = TaskIndex++;
			size_t X = CurrentPos % Image.GetWidth();
			size_t Y = CurrentPos / Image.GetWidth();

			float CenterX = (0.5f + (float)X) * SensorX;
			float CenterY = (0.5f + (float)Y) * SensorY;

			Image.WriteFlipped(X, Y, ComputeColor(Eye, World, Quality, (uint32_t)CurrentPos, SensorX, SensorY, CenterX, CenterY));
		}
	};

	std::atomic<size_t> TaskIter = 0; // 640 * 10 + 505

	constexpr size_t kNumThreads = 6;

	std::vector<std::thread> WorkerThreads;
	WorkerThreads.reserve(kNumThreads);

	size_t NumTasks = Image.GetHeight() * Image.GetWidth();

	for (size_t Counter = 0; Counter < kNumThreads; Counter++) {
		WorkerThreads.emplace_back(ImageTask, std::ref(Image), NumTasks, std::ref(TaskIter));
	}

	for (std::thread& WorkerThread : WorkerThreads) {
		WorkerThread.join();
	}
}

// TODO: Stochastic anti aliasing
ColorRGB32F NaiveUnidirectionalIntegrator::ComputeColor(const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, const uint32_t Seed, const float SensorX, const float SensorY, const float X, const float Y) {
	// Use a different sampler for each pixel to allow a deterministic thread-safe RNG
	SamplerDefault CurrentSampler = Sampler;
	CurrentSampler.SetSeed(Seed);

	constexpr float kPositionPushOut = 1e-3f;
	glm::vec3 Radiance(0.0f);

	for (size_t SampleCounter = 0; SampleCounter < Quality.SamplesPerPixel; SampleCounter++) {
		float JitterX = SensorX * (CurrentSampler.NextFloat() - 0.5f);
		float JitterY = SensorY * (CurrentSampler.NextFloat() - 0.5f);

		Ray IrradianceRay = Eye.GenerateRay(Sampler, X + JitterX, Y + JitterY);

		Radiance += EvaluatePath(IrradianceRay, Eye, World, Quality, CurrentSampler);
	}
	Radiance /= Quality.SamplesPerPixel;
	return Radiance;
}

ColorRGB32F NaiveUnidirectionalIntegrator::EvaluatePath(Ray& IrradianceRay, const Camera& Eye, const Scene& World, const PathTracerQuality& Quality, SamplerDefault& CurrentSampler) {
	ColorRGB32F PathRadiance = glm::vec3(0.0f);

	Vertex CurrentVertex;
	TriangleIntersection CurrentIntersection;
	glm::vec3 IncidentDirection = IrradianceRay.Direction;

	float CosineTheta = glm::dot(IrradianceRay.Direction, Eye.GetDirection());
	auto PreviousVertexMaterial = MaterialProperties::SurfaceType::Diffuse;
	bool CameraEdge = true;

	glm::vec3 RadianceFlow(1.0f);

	while (true) {
		float PDF = 1.0f;
		glm::vec3 BRDF;

		if (!CameraEdge) {
			PreviousVertexMaterial = CurrentIntersection.Material->Type;
		}

		CurrentIntersection.Reset();
		if (!World.Intersect(IrradianceRay, CurrentIntersection)) {
			break;
		}

		CurrentVertex = CurrentIntersection.GetInterpolatedVertex();
		CurrentVertex.Position = IrradianceRay.Extend(CurrentIntersection.Depth);
		IncidentDirection = IrradianceRay.Direction;

		if (CurrentIntersection.Material->Type == MaterialProperties::SurfaceType::Emissive) {
			if (CameraEdge) {
				return CurrentIntersection.Material->Emission;
			} else {

				float HemispherePDF = 1.0f;
				float Weight = 1.0f;
				float Foreshortening = 1.0f;

				if (PreviousVertexMaterial == MaterialProperties::SurfaceType::Diffuse) {
					Foreshortening = (CosineTheta / Constants::kPi);
					float Conversion = CosineTheta / (CurrentIntersection.Depth * CurrentIntersection.Depth);

					HemispherePDF = Conversion * CosineTheta / Constants::kPi;
					float NEEPDF = 1.0f / World.GetTotalLightArea();

					Weight = HemispherePDF / (Quality.NextEventEstimationSamples * NEEPDF + HemispherePDF);
				}

				PathRadiance += RadianceFlow * CurrentIntersection.Material->Emission * Foreshortening * Weight / HemispherePDF;
				break;
			}
		}
		CameraEdge = false;


		if (CurrentIntersection.Material->Type == MaterialProperties::SurfaceType::Refractive) {

			float R0 = (1.0f - CurrentIntersection.Material->RefractiveIndex) / (1.0f + CurrentIntersection.Material->RefractiveIndex);
			R0 *= R0;

			float IdotN = glm::dot(CurrentVertex.Normal, IncidentDirection);

			float Angular = 1.0f + IdotN < 0.0f ? IdotN : -IdotN;
			float Angular2 = Angular * Angular;
			float Angular3 = Angular2 * Angular;

			float Fresnel = R0 + (1.0f - R0) * (Angular2 * Angular3);

			if (Fresnel < CurrentSampler.NextFloat()) {
				IrradianceRay.Direction = glm::reflect(IncidentDirection, CurrentVertex.Normal);
			} else {
				IrradianceRay.Direction = glm::refract(IncidentDirection, CurrentVertex.Normal, IdotN < 0.0f ? 1.0f / CurrentIntersection.Material->RefractiveIndex : CurrentIntersection.Material->RefractiveIndex);
			}

			BRDF = glm::vec3(1.0f);
		}
		else {

			glm::vec3 DirectLighting = glm::vec3(0.0f);
			glm::vec3 DirectLightingBRDF = RadianceFlow * CurrentIntersection.Material->Diffuse / Constants::kPi;

			for (size_t SampleCounter = 0; SampleCounter < Quality.NextEventEstimationSamples; SampleCounter++) {

				float RandomVariables[4] = {
					CurrentSampler.NextFloat(),
					CurrentSampler.NextFloat(),
					CurrentSampler.NextFloat(),
					CurrentSampler.NextFloat(),
				};

				const PolygonalLight* RandomLight = World.GetRandomLight(RandomVariables[0]);

				auto TempVertexTrianglePair = RandomLight->ChooseRandomPoint(RandomVariables[1], RandomVariables[2], RandomVariables[3]);
				Vertex RandomLightVertex = TempVertexTrianglePair.first;
				const Triangle* RandomLightTriangle = TempVertexTrianglePair.second;

				glm::vec3 Segment = RandomLightVertex.Position - CurrentVertex.Position;
				float Distance2 = glm::dot(Segment, Segment);
				float Distance = std::sqrtf(Distance2);

				Ray DirectLightingRay;

				DirectLightingRay.Origin = CurrentVertex.Position;
				DirectLightingRay.Direction = Segment / Distance;

				if (!World.ComputeVisibility(DirectLightingRay, RandomLightTriangle, Distance)) {
					continue;
				}

				float LightPDF = 1.0f / World.GetTotalLightArea();
				float BSDFPDF = (fabsf(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction)) / Distance2) * glm::max(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction), 0.0f) / Constants::kPi;

				float Weight = Quality.NextEventEstimationSamples * LightPDF / (Quality.NextEventEstimationSamples * LightPDF + BSDFPDF);

				float G = fabsf(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction) * /*-*/glm::dot(RandomLightVertex.Normal, DirectLightingRay.Direction)) / Distance2;
				DirectLighting += RandomLight->Material->Emission * DirectLightingBRDF * G * Weight / LightPDF;

			}

			DirectLighting /= Quality.NextEventEstimationSamples;
			PathRadiance += DirectLighting;

			CosineTheta = CurrentSampler.NextFloat();
			float Phi = 2.0f * Constants::kPi * CurrentSampler.NextFloat();

			float Radius = std::sqrtf(1.0f - CosineTheta * CosineTheta);

			IrradianceRay.Direction.x = std::cosf(Phi) * Radius;
			IrradianceRay.Direction.y = std::sinf(Phi) * Radius;
			IrradianceRay.Direction.z = CosineTheta;

			glm::vec3 B[2];

			B[0] = fabsf(CurrentVertex.Normal.x) > 0.995f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
			B[0] = B[0] - CurrentVertex.Normal * glm::dot(CurrentVertex.Normal, B[0]);

			B[1] = glm::cross(B[0], CurrentVertex.Normal);

			IrradianceRay.Direction =
				IrradianceRay.Direction.x * B[0] +
				IrradianceRay.Direction.y * B[1] +
				IrradianceRay.Direction.z * CurrentVertex.Normal;

			BRDF = CurrentIntersection.Material->Diffuse;
		}
		IrradianceRay.Origin = CurrentVertex.Position;

		RadianceFlow *= BRDF / PDF;

		float RandomTermination = glm::max(RadianceFlow.x, glm::max(RadianceFlow.y, RadianceFlow.z));

		if (CurrentSampler.NextFloat() > RandomTermination) {
			break;
		}

		RadianceFlow /= RandomTermination;
	}

	return PathRadiance;
}

/*
		// Choose random pair of spherical coordiantes (theta, phi)
		glm::vec2 SphereCoords(UniformDistribution(Generator) * 0.25f, UniformDistribution(Generator) * 2.0f);
		SphereCoords *= Constants::kPi;
		// Construct vector from it
		glm::vec3 Direction;
		Direction.y = glm::sin(SphereCoords.x);
		Direction.x = glm::cos(SphereCoords.x) * glm::cos(SphereCoords.y);
		Direction.z = glm::cos(SphereCoords.x) * glm::sin(SphereCoords.y);
		float Foreshortening = Direction.y;
		// Get radiance
		Ray SkyRay;
		SkyRay.Direction = TBN * Direction;
		Radiance += Foreshortening * World.GetSkyIrradiance(SkyRay) * BRDF / PDF;
*/

/*
// Get primary vertex on path and build tangent space to world space transform for hemisphere sampling
	Vertex PathVertex = IntersectionInfo.GetInterpolatedVertex();
	float kMaxDot = 1.0f - 1e-4f;
	glm::vec3 CrossVector = glm::vec3(0.0f, 1.0f, 0.0f);
	if (fabsf(glm::dot(PathVertex.Normal, CrossVector)) > kMaxDot) {
		CrossVector = glm::vec3(1.0f, 0.0f, 0.0f);
	}
	glm::vec3 Tangent = glm::cross(CrossVector, PathVertex.Normal);
	Tangent = glm::normalize(Tangent);
	glm::mat3 TBN = glm::mat3(Tangent, glm::cross(Tangent, PathVertex.Normal), PathVertex.Normal);
	// Basic sky irradiance integration, no scene occlusion/interreflection taken into account for simplicity
	glm::vec3 Radiance (0.0f);
	// Start monte carlo integration
	constexpr float PDF = 1.0f / (2.0f * Constants::kPi);
	constexpr float Lambertian = 1.0f / Constants::kPi;
	glm::vec3 BRDF = IntersectionInfo.Material->Diffuse * Lambertian;
	for (size_t Counter = 0; Counter < Quality.SamplesPerPixel; Counter++) {
		Ray LightRay;
		LightRay.Origin = PathVertex.Position;
		// Choose random hemispherical direction (based on https://github.com/matt77hias/cpp-smallpt/blob/master/cpp-smallpt/cpp-smallpt/src/sampling.hpp#L32)
		float CosTheta = UniformDistribution(Generator);
		float SineTheta = sqrtf(1.0f - CosTheta * CosTheta);
		float Phi = UniformDistribution(Generator) * 2.0f * Constants::kPi;
		LightRay.Direction.y = CosTheta;
		LightRay.Direction.x = SineTheta * cos(Phi);
		LightRay.Direction.z = SineTheta * sin(Phi);
		float Foreshortening = LightRay.Direction.y;
		Radiance += LightRay.Direction;
		LightRay.Direction = glm::normalize(TBN * glm::normalize(LightRay.Direction));
		// Intersect with scene
		TriangleIntersection LightIntersection;
		if (World.Intersect(LightRay, LightIntersection)) {
			// Eval radiance
			//Radiance += LightIntersection.Material->Diffuse * Foreshortening * IntersectionInfo.Material->Diffuse * Lambertian / PDF;

		} else {
			//Radiance += World.GetSkyIrradiance(LightRay) * Foreshortening * IntersectionInfo.Material->Diffuse * Lambertian / PDF;
		}
	}
	Radiance /= Quality.SamplesPerPixel;
	//Radiance = PathVertex.Normal;
	//Radiance = Tangent * 0.5f + 0.5f;
	//Radiance = IntersectionInfo.Material->Diffuse;
	return { Radiance.r, Radiance.g, Radiance.b };
*/

/*


	auto ImageTask = [this, Eye, World, Quality](Texture<ColorRGB32F>& Image, size_t Y0, size_t Y1) -> void {
		for (size_t Y = Y0; Y < Y1; Y++) {
			for (size_t X = 0; X < Image.GetWidth(); X++) {
				Image.WriteFlipped(X, Y, ComputeColor(Eye, World, Quality, (float)X / Image.GetWidth(), (float)Y / Image.GetHeight()));
			}
		}
	};

	constexpr size_t kNumThreads = 6;
	size_t TaskSize = Image.GetHeight() / kNumThreads;

	std::vector<std::future<void>> WorkerThreads;

	size_t Y = 0;
	size_t RoundedImageSize = TaskSize * (Image.GetHeight() / TaskSize);
	for (; Y < RoundedImageSize; Y += TaskSize) {
		WorkerThreads.emplace_back(std::async(std::launch::async, ImageTask, Image, Y, Y + TaskSize));
	}

	if (Y != RoundedImageSize) {
		WorkerThreads.emplace_back(std::async(std::launch::async, ImageTask, Image, Y, RoundedImageSize));
	}
*/

/*
Pre loop:
1. Init the vars (eye ray, etc)
Loop:
1. Trace the ray
2. Compute lighting information
3. Choose new direction
4. Russian roullete 
*/

/*

	/*




	// Ideally for DOF you would want to generate multiple samples
	Ray ViewRay = Eye.GenerateRay(X, Y);

	// Intersect with scene
	TriangleIntersection IntersectionInfo;
	if (!World.Intersect(ViewRay, IntersectionInfo)) {
		// If nothing was intersected return radiance of the sky
		return World.GetSkyIrradiance(ViewRay);
	}

	// Currently emissive objects have fr(wi, wo, x) = 0
	if(IntersectionInfo.Material->Type == MaterialProperties::SurfaceType::Emissive) {
		return IntersectionInfo.Material->Emission;
	}

	Vertex EyeVertex = IntersectionInfo.GetInterpolatedVertex();
	EyeVertex.Position = ViewRay.Extend(IntersectionInfo.Depth);

	// Collected radiance from monte carlo integration
	glm::vec3 Radiance(0.0f);

	for (size_t SampleCounter = 0; SampleCounter < Quality.SamplesPerPixel; SampleCounter++) {
		glm::vec3 RadianceFlow(1.0f);

		Vertex CurrentVertex = EyeVertex;
		TriangleIntersection CurrentIntersection;
		glm::vec3 IncidentDirection = ViewRay.Direction;

		CurrentIntersection.Material = IntersectionInfo.Material;

		while (true) {
			/*
			1. Generate direction
			2. Compute BRDF
			3. Compute reweighed radiance flow with proper PDF
			4. Russion roullete (spelling is probably wrong lol)
			5. Intersect with scene
			6. Add collected radiance if it is a light and break from loop
			7. Compute/set new wo variable for BRDF (not necessary for diffuse but needed for view dependent BRDFs)
			*/ /*

float PDF = 1.0f;
glm::vec3 BRDF;

Ray IrradianceRay;
IrradianceRay.Origin = CurrentVertex.Position; // Begin at the previous vertex

float CosineTheta = 1.0f;

if (CurrentIntersection.Material->Type == MaterialProperties::SurfaceType::Refractive) {
	// (1) Compute the perfect specular direction. We need this to compute BRDF and PDF. Schlick's formular works here

	float R0 = (1.0f - CurrentIntersection.Material->RefractiveIndex) / (1.0f + CurrentIntersection.Material->RefractiveIndex);
	R0 *= R0;

	// Positive if refractive vertex is being hit from underneath by ray
	float IdotN = glm::dot(CurrentVertex.Normal, IncidentDirection);

	float Angular = 1.0f + IdotN < 0.0f ? IdotN : -IdotN; // Reverse incident direction for outgoing direction, factor out -1 for addition
	float Angular2 = Angular * Angular;
	float Angular3 = Angular2 * Angular;

	float Fresnel = R0 + (1.0f - R0) * (Angular2 * Angular3);

	// Now use the fresnel to choose refraction or reflection
	if (Fresnel < CurrentSampler.NextFloat()) {
		// Reflect. Reflection is proportional to BRDF so we can set it to 1
		IrradianceRay.Direction = glm::reflect(IncidentDirection, CurrentVertex.Normal);
	}
	else {
		// (1) Refract. Refraction is also proportional to BRDF so it's 1 as well
		IrradianceRay.Direction = glm::refract(IncidentDirection, CurrentVertex.Normal, IdotN < 0.0f ? 1.0f / CurrentIntersection.Material->RefractiveIndex : CurrentIntersection.Material->RefractiveIndex);
	}

	BRDF = glm::vec3(1.0f);
}
else {

#if 1
	// NEE only works for diffuse surfaces

	glm::vec3 DirectLighting = glm::vec3(0.0f);
	glm::vec3 DirectLightingBRDF = RadianceFlow * CurrentIntersection.Material->Diffuse / Constants::kPi;

	for (size_t SampleCounter = 0; SampleCounter < Quality.NextEventEstimationSamples; SampleCounter++) {

		float RandomVariables[4] = {
			CurrentSampler.NextFloat(),
			CurrentSampler.NextFloat(),
			CurrentSampler.NextFloat(),
			CurrentSampler.NextFloat(),
		};

		const PolygonalLight* RandomLight = World.GetRandomLight(RandomVariables[0]);

		auto TempVertexTrianglePair = RandomLight->ChooseRandomPoint(RandomVariables[1], RandomVariables[2], RandomVariables[3]);
		Vertex RandomLightVertex = TempVertexTrianglePair.first;
		const Triangle* RandomLightTriangle = TempVertexTrianglePair.second;

		glm::vec3 Segment = RandomLightVertex.Position - CurrentVertex.Position;
		float Distance2 = glm::dot(Segment, Segment);
		float Distance = std::sqrtf(Distance2);

		Ray DirectLightingRay;

		DirectLightingRay.Origin = CurrentVertex.Position;
		DirectLightingRay.Direction = Segment / Distance;

		if (!World.ComputeVisibility(DirectLightingRay, RandomLightTriangle, Distance)) {
			continue;
		}

		float LightPDF = 1.0f / World.GetTotalLightArea();
		float BSDFPDF = (fabsf(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction)) / Distance2) * glm::max(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction), 0.0f) / Constants::kPi;

		// Combine using balance hurestic 
		float Weight = Quality.NextEventEstimationSamples * LightPDF / (Quality.NextEventEstimationSamples * LightPDF + BSDFPDF);

		// float G = fabsf(glm::dot(CurrentVertex.Normal, DirectLightingRay.Direction) * /*-*///glm::dot(RandomLightVertex.Normal, DirectLightingRay.Direction)) / Distance2; /*
/*DirectLighting += RandomLight->Material->Emission * DirectLightingBRDF * G * Weight / LightPDF;

	}

	DirectLighting /= Quality.NextEventEstimationSamples;
	Radiance += DirectLighting;
	//std::cout << Seed << ' ' << Radiance / (float)Quality.SamplesPerPixel << std::endl;

#endif

				// (1) Random direction can be computed using cosine weighted hemisphere

	CosineTheta = CurrentSampler.NextFloat(); // Foreshortening cancels out
	float Phi = 2.0f * Constants::kPi * CurrentSampler.NextFloat();

	float Radius = std::sqrtf(1.0f - CosineTheta * CosineTheta);

	// (1) Now compute the direction

	IrradianceRay.Direction.x = std::cosf(Phi) * Radius;
	IrradianceRay.Direction.y = std::sinf(Phi) * Radius;
	IrradianceRay.Direction.z = CosineTheta;

	// (1) Transform from hemisphere to world space. Better solutions availible here https://core.ac.uk/download/pdf/13797184.pdf

	glm::vec3 B[2];

	B[0] = fabsf(CurrentVertex.Normal.x) > 0.995f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
	B[0] = glm::normalize(glm::cross(B[0], CurrentVertex.Normal)); // B[0] - CurrentVertex.Normal * glm::dot(CurrentVertex.Normal, B[0])

	B[1] = glm::cross(B[0], CurrentVertex.Normal);

	IrradianceRay.Direction =
		IrradianceRay.Direction.x * B[0] +
		IrradianceRay.Direction.y * B[1] +
		IrradianceRay.Direction.z * CurrentVertex.Normal;

	// (2) Compute the BRDF (and PDF, if it is necessary, which in this case, it isn't since we cancel out terms)

	BRDF = CurrentIntersection.Material->Diffuse; // We don't need to divide by lambertian (Pi) since the PDF cancels it out
}

// (3) Reweight the randiance flow by mult with BRDF / (PDF * Termination prob)

RadianceFlow *= BRDF / PDF;

// (4) Probablistic unbiased termination with russian roullete 
float RandomTermination = glm::max(RadianceFlow.x, glm::max(RadianceFlow.y, RadianceFlow.z)); // Max allows use to not stop at specular vertices 
// glm::dot(RadianceFlow, glm::vec3(0.2126f, 0.7152f, 0.0722f)); // Luma allows us to terminate a lot more samples than max

if (CurrentSampler.NextFloat() > RandomTermination) {
	break;
}

// (4) Reweight 

RadianceFlow /= RandomTermination;

// (5) Intersect with the scene

auto PreviousVertexMaterial = CurrentIntersection.Material->Type;

CurrentIntersection.Reset();
if (!World.Intersect(IrradianceRay, CurrentIntersection)) {
	// Comment the next line to make sky black
	// Radiance += RadianceFlow * World.GetSkyIrradiance(IrradianceRay); // Collect sky radiance
	break;
}

// (6) If we are lucky enough to strike a light, we can add its radiance and break out of this loop to pursue another sample

if (CurrentIntersection.Material->Type == MaterialProperties::SurfaceType::Emissive) {
	// Divde by NEE PDF
	float HemispherePDF = 1.0f;
	float Weight = 1.0f;
	float Foreshortening = 1.0f;
	if (PreviousVertexMaterial == MaterialProperties::SurfaceType::Diffuse) {
		float Conversion = CosineTheta / (CurrentIntersection.Depth * CurrentIntersection.Depth);
		HemispherePDF = Conversion * CosineTheta / Constants::kPi;
		float NEEPDF = 1.0f / World.GetTotalLightArea();
		Weight = HemispherePDF / (Quality.NextEventEstimationSamples * NEEPDF + HemispherePDF);
		Foreshortening = (CosineTheta / Constants::kPi);
	}

	Radiance += RadianceFlow * CurrentIntersection.Material->Emission * Foreshortening * Weight / HemispherePDF;
	break;
}

// (7) If we did not strike a light, we need to continue onwards for another bounce

CurrentVertex = CurrentIntersection.GetInterpolatedVertex();
CurrentVertex.Position = IrradianceRay.Extend(CurrentIntersection.Depth);

IncidentDirection = IrradianceRay.Direction;
		}
	}
	//std::cout << "Done\n"; // Test for any infinite loops
	Radiance /= Quality.SamplesPerPixel;
	return Radiance;

}
*/