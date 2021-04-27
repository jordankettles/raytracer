/* $Rev: 250 $ */
#include "Cube.h"

#include "utility.h"

Cube::Cube() : Object() {

}

Cube::Cube(const Cube& cube) : Object(cube) {

}

Cube::~Cube() {

}

Cube& Cube::operator=(const Cube& cube) {
	if (this != &cube) {
		Object::operator=(cube);
	}
	return *this;
}

std::vector<RayIntersection> Cube::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;
	
	Ray inverseRay = transform.applyInverse(ray);
	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);
	double zt = (-1-z0)/dz; /* Front face */

	double y0 = inverseRay.point(1);
	double dy = inverseRay.direction(1);
	double yt = (-1-y0)/dy; /* Top face */

	double x0 = inverseRay.point(0);
	double dx = inverseRay.direction(0);
	double xt = (-1-x0)/dx; /* Left Face*/

	RayIntersection hit;
	hit.material = material;

	if (std::abs(dz) > epsilon) {
		if (zt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + zt*inverseRay.direction;
			/* if x and y are in the range [-1, 1] */
			if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(1)) <= 1) {
			hit.normal = Normal(0, 0, -1);
			hit.point = transform.apply(hit.point);
			hit.normal = transform.apply(hit.normal);
			hit.distance = (hit.point - ray.point).norm();
			result.push_back(hit);
			}
		}
		zt = (1-z0)/dz;  /* Back face. */
		if (zt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + zt*inverseRay.direction;
			/* if x and y are in the range [-1, 1] */
			if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(1)) <= 1) {
			hit.normal = Normal(0, 0, 1);
			hit.point = transform.apply(hit.point);
			hit.normal = transform.apply(hit.normal);
			hit.distance = (hit.point - ray.point).norm();
			result.push_back(hit);
			}
		}
	}
	if (std::abs(dy) > epsilon) {
		if (yt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + yt*inverseRay.direction;
			 /* if x and z are in the range [-1, 1] */
			if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(2)) <= 1) {
				hit.normal = Normal(0, -1, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(hit.normal);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		yt = (1-y0)/dy; /* Bottom face */
		if (yt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + yt*inverseRay.direction;
			 /*if x and z are in the range [-1, 1] */
			if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(2)) <= 1) {
				hit.normal = Normal(0, 1, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(hit.normal);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
	}
	if (std::abs(dx) > epsilon) {
		if (xt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + xt*inverseRay.direction;
			/* if y and z are in the range [-1, 1] */
			if(std::abs(hit.point(1)) <= 1 && std::abs(hit.point(2)) <= 1) {
				hit.normal = Normal(-1, 0, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(hit.normal);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		xt = (1-x0)/dx; /* Right Face */
		if (xt > 0) { /* If the distance is positive. */
			hit.point = inverseRay.point + xt*inverseRay.direction;
			/* if y and z are in the range [-1, 1] */
			if(std::abs(hit.point(1)) <= 1 && std::abs(hit.point(2)) <= 1) {
				hit.normal = Normal(1, 0, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(hit.normal);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		
	}

	return result;
}
