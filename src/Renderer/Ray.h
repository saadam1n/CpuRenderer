#pragma once

#include <glm/glm.hpp>

struct Ray {
	glm::vec3 Origin;
	glm::vec3 Direction;

	Ray(const glm::vec3& O, glm::vec3& D);
	Ray(void);

	glm::vec3 Extend(float T);
};