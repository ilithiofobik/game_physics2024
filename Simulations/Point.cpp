#include "Point.h"

Point::Point(Vec3 p, Vec3 v, float m, float d, bool isF) {
	position = p;
	velocity = v;
	force = Vec3();
	mass = m;
	damping = d;
	isFixed = isF;
}

float Point::distance(const Point& another) {
	return sqrt(position.squaredDistanceTo(another.position));
}