#include "Plane.h"

#include "utility.h"

Plane::Plane() : Object() {

}

Plane::Plane(const Plane& plane) : Object(plane) {

}

Plane::~Plane() {

}

Plane& Plane::operator=(const Plane& plane) {
	if (this != &plane) {
		Object::operator=(plane);
	}
	return *this;
}

std::vector<RayIntersection> Plane::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	Ray inverseRay 	= transform.applyInverse(ray);

	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);
	double t = -z0/dz;

	if (std::abs(dz) > epsilon) {
		if (t > 0) { /* If the distance is positive. */
			RayIntersection hit;
			hit.point = inverseRay.point + t*inverseRay.direction;
			/* if x and y are in the range [-1, 1] */
			if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(1)) <= 1) {
				hit.material = material;
				hit.normal = Normal(0, 0, 1);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(hit.normal);
				if (hit.normal.dot(ray.direction) > 0) { /* Reverse the normal if the normal is not the same direction as the ray. */
				hit.normal = -hit.normal;
				}
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}

		}
		
	}

	return result;
}