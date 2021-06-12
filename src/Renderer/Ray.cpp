#include "Ray.h"

Ray::Ray(const glm::vec3& O, const glm::vec3& D) : Origin(O), Direction(D) {}


Ray::Ray(void) : Ray(glm::vec3(0.0f), glm::vec3(0.0f)) {}

glm::vec3 Ray::Extend(float T) const {
	return Origin + Direction * T;
}