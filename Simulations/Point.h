#ifndef POINT_h
#define POINT_h
#include "util/vectorbase.h"

using namespace GamePhysics;

class Point {
public:
	// Construtors
	Point(Vec3 p, Vec3 v, float m, float d, bool isF);

	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	float damping;
	bool isFixed;

	float distance(const Point& another);
	void clearForce();
	void addForce(const Vec3& newForce);
};
#endif