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
	Mat4 invIntertia(); // current moment of inertia changes
	void simulateTimestep(float timeStep);
	Vec3 getPosition();
	Vec3 getAngVel();
	void setOrientation(const Quat& r);

	Vec3 linVel;

private:
	Mat4 rotMat();
	Mat4 translatMat();

	Mat4 scaleMat; // body doesn't change size
	Vec3 invIntertia0; // basic moment of intertia does not change
	float mass;
	Quat orientation; // r
	Vec3 angVel; // w
	Vec3 force;
	Vec3 momentum; // L
	Vec3 torque; // q
	Vec3 position;
	Vec3 size;
};
#endif
