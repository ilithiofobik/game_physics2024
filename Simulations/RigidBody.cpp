#include "RigidBody.h"

// Assume all rigid bodies are boxes

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

Mat4 RigidBody::scaleMat()
{
	Mat4 m = Mat4();
	m.initScaling(size.x, size.y, size.z);
	return m;
}

Mat4 RigidBody::rotMat()
{
	return orientation.getRotMat();
}

Mat4 RigidBody::translatMat()
{
	Mat4 m = Mat4();
	m.initTranslation(position.x, position.y, position.z);
	return m;
}

Mat4 RigidBody::objToWorldMatrix()
{
	Mat4 sm = scaleMat();
	Mat4 rm = rotMat();
	Mat4 tm = translatMat();
	return sm * rm * tm;
}
