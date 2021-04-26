#include "Cylinder.h"

#include "utility.h"

Cylinder::Cylinder() : Object() {

}

Cylinder::Cylinder(const Cylinder& cylinder) : Object(cylinder) {

}

Cylinder::~Cylinder() {

}

Cylinder& Cylinder::operator=(const Cylinder& cylinder) {
	if (this != &cylinder) {
		Object::operator=(cylinder);
	}
	return *this;
}

std::vector<RayIntersection> Cylinder::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	Ray inverseRay = transform.applyInverse(ray);

	RayIntersection hit;
	hit.material = material;

	Point p = inverseRay.point;
	p(2) = 0;

	Direction d = inverseRay.direction;
	d(2) = 0;

	
	double a = d.dot(d);
	double b = 2 * d.dot(p);
	double c = p.dot(p) - 1;

	double discriminant = b*b - 4*a*c;
	double t;
	
	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);
	double zt = (-1-z0)/dz; /* Top cap */

	/* Check for an intersection with the curved surface of the cylinder. */
	switch (sign(discriminant)) {
	case -1:
		// No intersections
		break;
	case 0:
		// One intersection
		t = -b/(2*a);
		if (t > 0) {
			// Intersection is in front of the ray's start point
			hit.point = Point(inverseRay.point + t*inverseRay.direction);
			if (hit.point(2) <= 1 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(p + t * d));
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		break;
	case 1:
		// Two intersections
		t = (-b + sqrt(b*b - 4*a*c))/(2*a);
		if (t > 0) {
			// Intersection is in front of the ray's start point
			hit.point = Point(inverseRay.point + t * inverseRay.direction);
			if (hit.point(2) <= 1 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(p + t * d));
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}

		t = (-b - sqrt(b*b - 4*a*c))/(2*a);
		if (t > 0) {
			// Intersection is in front of the ray's start point
			hit.point = Point(inverseRay.point + t * inverseRay.direction);
			if (hit.point(2) <= 1 and hit.point(2) > -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(p + t * d));
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		break;
	default:
		std::cerr << "Something's wrong - sign(x) should be -1, +1 or 0" << std::endl;
		exit(-1);
		break;
	}

	/* Check for an intersection with the top and bottom cap. */
	if (std::abs(dz) > epsilon) {
		if (zt > 0) { 
			hit.point = inverseRay.point + zt*inverseRay.direction;
			if (pow(hit.point(0), 2) + pow(hit.point(1), 2) <= 1) { 
			hit.normal = Normal(0, 0, -1);
			hit.point = transform.apply(hit.point);
			hit.normal = transform.apply(hit.normal);
			hit.distance = (hit.point - ray.point).norm();
			result.push_back(hit);
			}
		}
		zt = (1-z0)/dz;  /* Bottom cap */
		if (zt > 0) {
			hit.point = inverseRay.point + zt*inverseRay.direction;
			if (pow(hit.point(0), 2) + pow(hit.point(1), 2) <= 1) { 
			hit.normal = Normal(0, 0, 1);
			hit.point = transform.apply(hit.point);
			hit.normal = transform.apply(hit.normal);
			hit.distance = (hit.point - ray.point).norm();
			result.push_back(hit);
			}
		}
	}





	
	return result;
}
