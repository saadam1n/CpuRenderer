#include "Camera.h"
#include "Constants.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

Camera::Camera(void) : Direction(0.0f, 0.0f, -1.0f), Position(0.0f), Rotation(glm::radians(-90.0f), 0.0f, 0.0f), ViewMatrix(1.0f), ApertureSize(0.35f), FocusDistance(2.5f) {}

void Camera::GenerateImagePlane(void) {
	glm::vec2 FOV = glm::vec2(AspectRatio * FieldOfView, FieldOfView);
	glm::vec2 Sine = glm::sin(FOV);
	glm::vec2 Cosine = glm::cos(FOV);

	glm::vec2 ScreenDirection = glm::vec2(Sine.x * Cosine.y, Sine.y);

	//ScreenDirection = -ScreenDirection;

	float DirectionLength = glm::length(glm::vec2(ScreenDirection.x, ScreenDirection.y)) - 1.0f;

	FilmPlane.Corner[1][1].x = ScreenDirection.x;
	FilmPlane.Corner[1][1].y = ScreenDirection.y;
	FilmPlane.Corner[1][1].z = DirectionLength;

	FilmPlane.Corner[0][0] = -FilmPlane.Corner[1][1];
	FilmPlane.Corner[0][0].z = FilmPlane.Corner[1][1].z;

	FilmPlane.Corner[1][0] = FilmPlane.Corner[1][1];
	FilmPlane.Corner[1][0].x = -FilmPlane.Corner[1][0].x;

	FilmPlane.Corner[0][1] = FilmPlane.Corner[1][1];
	FilmPlane.Corner[0][1].y = -FilmPlane.Corner[0][1].y;

	// Short for corner access
	//#define CNR_ACS [0][1]

	//printf("(%f, %f, %f)\n", FilmPlane.Corner CNR_ACS.x, FilmPlane.Corner CNR_ACS.y, FilmPlane.Corner CNR_ACS.z);

	// TODO: fix my code later, but this will be the temporary working solution for now

	glm::mat4 ProjectionMatrix = glm::perspective(FieldOfView, AspectRatio, 0.1f, 1.0f);
	glm::mat4 ProjectionInverseMatrix = glm::inverse(ProjectionMatrix);

	glm::mat3 ViewMatrixInverse = glm::inverse(ViewMatrix);

	auto GenerateDirection = [ProjectionInverseMatrix, ViewMatrixInverse](glm::vec3 DirectionSet[2][2], int X, int Y) -> void {
		glm::vec2 ScreenCoordinates = glm::vec2(X, Y);

		glm::vec4 NDC = glm::vec4(ScreenCoordinates * 2.0f - 1.0f, 1.0f, 1.0f);

		glm::vec4 ViewSpace = ProjectionInverseMatrix * NDC;

		glm::vec3 Direction = glm::normalize(glm::vec3(ViewSpace.x, ViewSpace.y, ViewSpace.z) / ViewSpace.w);

		Direction = ViewMatrixInverse * Direction;

		DirectionSet[Y][X] = Direction;
	};

	GenerateDirection(FilmPlane.Corner, 0, 0);
	GenerateDirection(FilmPlane.Corner, 0, 1);
	GenerateDirection(FilmPlane.Corner, 1, 0);
	GenerateDirection(FilmPlane.Corner, 1, 1);

	//printf("(%f, %f, %f)\n", ImagePlane.Corner CNR_ACS.x, ImagePlane.Corner CNR_ACS.y, ImagePlane.Corner CNR_ACS.z);
}

void Camera::SetImagePlaneParameters(float AR, float FOV) {
	AspectRatio = AR;
	FieldOfView = FOV;
}

void Camera::UpdateImagePlaneParameters(float AR, float FOV) {
	SetImagePlaneParameters(AR, FOV);
	GenerateImagePlane();
}

void Camera::GenerateViewTransform(void) {
	glm::vec3 Sine = glm::sin(Rotation);
	glm::vec3 Cosine = glm::cos(Rotation);

	Direction.x = Cosine.y * Cosine.x;
	Direction.y = Sine.y;
	Direction.z = Cosine.y * Sine.x;

	Direction = glm::normalize(Direction);

	ViewMatrix        = glm::mat3(glm::lookAt(glm::vec3(0.0f), Direction, glm::vec3(0.0f, 1.0f, 0.0f)));
	ViewMatrixInverse = glm::inverse(ViewMatrix);
}

Ray Camera::GenerateRay(SamplerDefault& Sampler, float X, float Y) const {
	Ray R;
	R.Origin = Position;

	glm::vec3 YBlend[2];

	YBlend[0] = glm::mix(FilmPlane.Corner[0][0], FilmPlane.Corner[0][1], X);
	YBlend[1] = glm::mix(FilmPlane.Corner[1][0], FilmPlane.Corner[1][1], X);
	R.Direction = glm::mix(YBlend[0], YBlend[1], Y);

	R.Direction = normalize(R.Direction);

	float T = -FocusDistance / R.Direction.z;
	glm::vec3 FocusPoint = R.Direction * T;

	float Radius   = ApertureSize * std::sqrtf(Sampler.NextFloat());
	float Phi = 2.0f * Constants::kPi * Sampler.NextFloat();

	glm::vec3 RandomAperturePoint = glm::vec3(Radius * glm::vec2(std::sinf(Phi), std::cosf(Phi)), 0.0f);
	R.Direction = glm::normalize(FocusPoint - RandomAperturePoint);
	R.Origin += ViewMatrixInverse * RandomAperturePoint;

	return R;
}

void Camera::Move(float Distance) {
	Position += Direction * Distance;
}

glm::vec3 Camera::GetRotation(void) const {
	return Rotation;
}

void Camera::SetRotation(const glm::vec3& Value) {
	Rotation = Value;
}

void Camera::AddRotation(const glm::vec3& Value) {
	Rotation += Value;
}

glm::vec3 Camera::GetPosition(void) const {
	return Position;
}

void Camera::SetPosition(const glm::vec3& Value) {
	Position = Value;
}

void Camera::AddPosition(const glm::vec3& Value) {
	Position += Value;
}

glm::vec3 Camera::GetDirection(void) const {
	return Direction;
}

const ImagePlane& Camera::GetImagePlane(void) const {
	return FilmPlane;
}