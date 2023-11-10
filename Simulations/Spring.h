#ifndef SPRING_h
#define SPRING_h
#include "util/vectorbase.h"
#include "Point.h"
#include <vector>

using namespace GamePhysics;

class Spring {
public:
	// Construtors
	Spring(int p1, int p2, float s, float iL);

	int point1;
	int point2;
	float stiffness;
	float initialLength;

	void computeElasticForces(const std::vector<Point>& points);
	void addToEndPoints(std::vector<Point>& points);
private:
	Vec3 elasticForce;
};
#endif