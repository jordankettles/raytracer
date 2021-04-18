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

	Point p = inverseRay.point;
	p(2) = 0;
	Direction d = inverseRay.direction;
	d(2) = 0;

	RayIntersection hit;
	hit.material = material;

	double a = d.dot(d);
	double b = 2 * d.dot(p);
	double c = p.dot(p) - 1;

	double b2_4ac = b*b - 4*a*c;
	double t;
	switch (sign(b2_4ac)) {
	case -1:
		// No intersections
		break;
	case 0:
		// One intersection
		t = -b/(2*a);
		if (t > 0) {
			// Intersection is in front of the ray's start point
			hit.point = Point(inverseRay.point + t*inverseRay.direction);
			if (hit.point(2) <= 1.0 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
				if (hit.normal.dot(ray.direction) > 0) {
					hit.normal = -hit.normal;
				}
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
			if (hit.point(2) <= 1.0 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
				if (hit.normal.dot(ray.direction) > 0) {
					hit.normal = -hit.normal;
				}
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}

		t = (-b - sqrt(b*b - 4*a*c))/(2*a);
		if (t > 0) {
			// Intersection is in front of the ray's start point
			hit.point = Point(inverseRay.point + t * inverseRay.direction);
			if (hit.point(2) <= 1.0 and hit.point(2) > -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
				if (hit.normal.dot(ray.direction) > 0) {
					hit.normal = -hit.normal;
				}
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
		break;
	default:
		// Shouldn't be possible, but just in case
		std::cerr << "Something's wrong - sign(x) should be -1, +1 or 0" << std::endl;
		exit(-1);
		break;
	}



	
	return result;
}
