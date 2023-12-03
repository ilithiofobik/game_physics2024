#ifndef POINT_h
#define POINT_h
#include "util/vectorbase.h"
#include "Simulator.h"

using namespace GamePhysics;

class RigidBody {
public:
	// Construtors
	RigidBody(Vec3 position, Vec3 size, int mass);

	void addForce(const Vec3& loc, const Vec3& force);
	void clearForce();
	Mat4 objToWorldMatrix();

	float mass;
	Quat orientation;
	Vec3 ang_velocity;
	Vec3 force;
	Vec3 torque;
	Vec3 lin_velocity;
	Vec3 position;
	Vec3 size;

private:
	Mat4 scaleMat();
	Mat4 rotMat();
	Mat4 translatMat();
};
#endif
