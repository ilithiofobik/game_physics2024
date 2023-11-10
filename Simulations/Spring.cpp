#include "Spring.h"
#include <vector>

Spring::Spring(int p1, int p2, float s, float iL) {
	point1 = p1;
	point2 = p2;
	stiffness = s;
	initialLength = iL;
	elasticForce = Vec3();
}

void Spring::computeElasticForces(const std::vector<Point>& points) {
	Point p1 = points[point1];
	Point p2 = points[point2];

	Vec3 diff = p1.position - p2.position;
	float l = p1.distance(p2);
	Vec3 direction = diff / l;

	float force = -stiffness * (l - initialLength);

	elasticForce = force * direction;
}

void Spring::addToEndPoints(std::vector<Point>& points) {
	points[point1].addForce(elasticForce);
	points[point2].addForce((-1.0) * elasticForce);
}
