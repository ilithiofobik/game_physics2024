#ifndef POINT_h
#define POINT_h
#include "util/vectorbase.h"

using namespace GamePhysics;

class Point {
public:
	// Construtors
	Point(Vec3 p, Vec3 v, float m, float d, bool isF);

	bool isFixed;
	float damping;
	float mass;
	Vec3 force;
	Vec3 position;
	Vec3 velocity;

	float distance(const Point& another);
	Vec3 direction(const Point& another);
	void addForce(const Vec3& newForce);
	void clearForce();
};
#endif