#include "RigidBody.h"

RigidBody::RigidBody(Vec3 p, Vec3 s, int m)
{
	ang_velocity = Vec3();
	force = Vec3();
	lin_velocity = Vec3();
	mass = m;
	orientation = Quat(0.0, 0.0, 0.0, 1.0);
	position = p;
	size = s;
	torque = Vec3();
}

void RigidBody::addForce(const Vec3& loc, const Vec3& f)
{
	// add as force
	force += f;
	// add as torque
	torque += cross(loc, f);
}

void RigidBody::clearForce()
{
	force = Vec3();
	torque = Vec3();
}
