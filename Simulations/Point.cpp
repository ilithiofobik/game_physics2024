#include "Point.h"

Point::Point(Vec3 p, Vec3 v, float m, float d, bool isF) {
	damping = d;
	force = Vec3();
	isFixed = isF;
	mass = m;
	position = p;
	velocity = v;
}

float Point::distance(const Point& another) {
	return sqrt(position.squaredDistanceTo(another.position));
}

Vec3 Point::direction(const Point& another) {
	Vec3 diff = position - another.position;
	float len = distance(another);

	return diff / len;
}

void Point::clearForce() {
	force = Vec3();
}

void Point::addForce(const Vec3& newForce) {
	force += newForce;
}
