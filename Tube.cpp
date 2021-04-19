#include "Tube.h"

#include "utility.h"


Tube::Tube(double ratio) : Object(), ratio_(ratio) {
}

Tube::Tube(const Tube& tube) : Object(tube), ratio_(tube.ratio_) {

}

Tube::~Tube() {

}

Tube& Tube::operator=(const Tube& tube) {
	if (this != &tube) {
		Object::operator=(tube);
		ratio_ = tube.ratio_;
	}
	return *this;
}

std::vector<RayIntersection> Tube::intersect(const Ray& ray) const {
	std::vector<RayIntersection> result;

	Ray inverseRay = transform.applyInverse(ray);

	Point p = inverseRay.point;
	p(2) = 0;
	Direction d = inverseRay.direction;
	d(2) = 0;

	
	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);
	double zt = (-1-z0)/dz; /* Front face */

	RayIntersection hit;
	hit.material = material;

	double a = d.dot(d);
	double b = 2 * d.dot(p);
	//Outer Tube
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
			if (hit.point(2) <= 1 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
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
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
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

	//Inner Tube
	c = p.dot(p) - (1/this->ratio_);

	b2_4ac = b*b - 4*a*c;

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
			if (hit.point(2) <= 1 and hit.point(2) >= -1) {
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
				hit.normal = -hit.normal;
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
				hit.normal = transform.apply(Normal(inverseRay.point + t*inverseRay.direction));
				hit.normal = -hit.normal;
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
				hit.normal = -hit.normal;
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
	if (std::abs(dz) > epsilon) {
		if (zt > 0) {
			hit.point = inverseRay.point + zt*inverseRay.direction;
			// if (std::abs(hit.point(0)) <= 1 && std::abs(hit.point(1)) <= 1) {
			if (pow(hit.point(0), 2) + pow(hit.point(1), 2) <= 1 and pow(hit.point(0), 2) + pow(hit.point(1), 2) >= 1/this->ratio_) { 
			hit.normal = Normal(0, 0, -1);
			hit.point = transform.apply(hit.point);
			hit.normal = transform.apply(hit.normal);
			hit.distance = (hit.point - ray.point).norm();
			result.push_back(hit);
			}
		}
		zt = (1-z0)/dz;  /* Back face. */
		if (zt > 0) {
			hit.point = inverseRay.point + zt*inverseRay.direction;
			if (pow(hit.point(0), 2) + pow(hit.point(1), 2) <= 1 and pow(hit.point(0), 2) + pow(hit.point(1), 2) >= 1/this->ratio_) { 
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
