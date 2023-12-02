#ifndef POINT_h
#define POINT_h
#include "util/vectorbase.h"
#include "Simulator.h"

using namespace GamePhysics;

class RigidBody {
public:
	// Construtors
	RigidBody(Vec3 position, Vec3 size, int mass);

	float mass;
	Quat orientation;
	Vec3 ang_velocity;
	Vec3 force;
	Vec3 lin_velocity;
	Vec3 position;
	Vec3 size;
};
#endif
