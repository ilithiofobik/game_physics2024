#ifndef POINT_h
#define POINT_h
#include "util/vectorbase.h"
#include "Simulator.h"

using namespace GamePhysics;

class RigidBody {
public:
	// Construtors
	RigidBody(Vec3 position, Vec3 size, int mass);
	RigidBody(Vec3 position, Vec3 size);

	float getInvMass();
	Mat4 invIntertia(); // current moment of inertia changes
	Mat4 objToWorldMatrix();
	Vec3 getAngVel();
	Vec3 getPosition();
	Vec3 pointVelocity(const Vec3& worldPoint); // Gives world velocity of worldPoint
	Vec3 relativePosition(const Vec3& worldPoint); // Gives position relative to center of mass
	void addForce(const Vec3& loc, const Vec3& force);
	void clearForce();
	void setOrientation(const Quat& r);
	void simulateTimestep(float timeStep);

	Vec3 linVel;
	Vec3 momentum; // L

private:
	Mat4 rotMat();
	Mat4 translatMat();

	float invMass;
	Mat4 scaleMat; // body doesn't change size
	Quat orientation; // r
	Vec3 force;
	Vec3 invIntertia0; // basic moment of intertia does not change
	Vec3 position;
	Vec3 size;
	Vec3 torque; // q
};
#endif
