#include "RigidBody.h"

// Assume all rigid bodies are boxes

RigidBody::RigidBody(Vec3 p, Vec3 s, int m)
{
	// precalculation
	Mat4 sm = Mat4();
	sm.initScaling(s.x, s.y, s.z);

	double ix = 12.0 / (m * (s.y * s.y + s.z * s.z));
	double iy = 12.0 / (m * (s.x * s.x + s.z * s.z));
	double iz = 12.0 / (m * (s.y * s.y + s.x * s.x));
	Vec3 ii0 = Vec3(ix, iy, iz);

	// setting
	force = Vec3();
	invIntertia0 = ii0;
	momentum = Vec3();
	linVel = Vec3();
	mass = m;
	orientation = Quat(0.0, 0.0, 0.0, 1.0);
	position = p;
	scaleMat = sm;
	size = s;
	torque = Vec3();
}

void RigidBody::addForce(const Vec3& loc, const Vec3& f)
{
	// add as force
	force += f;
	// add as torque
	torque += cross(loc - position, f); // loc is in real world
}

void RigidBody::clearForce()
{
	force = Vec3();
	torque = Vec3();
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
	Mat4 rm = rotMat();
	Mat4 tm = translatMat();
	return scaleMat * rm * tm;
}

Mat4 RigidBody::invIntertia()
{
	Mat4 rot = rotMat();
	Mat4 rotT = rot.inverse();
	Mat4 ii0 = Mat4();
	ii0.initScaling(invIntertia0.x, invIntertia0.y, invIntertia0.z);
	return rot * ii0 * rotT;
}

void RigidBody::simulateTimestep(float timeStep)
{
	position += timeStep * linVel;
	linVel += timeStep * (force / mass);

	Vec3 angVel = getAngVel();
	Quat w0 = Quat(angVel.x, angVel.y, angVel.z, 0.0);
	orientation += (timeStep / 2.0) * w0 * orientation;
	orientation = orientation.unit();
	momentum += timeStep * torque;

	Mat4 ii = invIntertia();
	angVel = ii.transformVector(momentum);

	clearForce();
}

Vec3 RigidBody::getPosition()
{
	return position;
}

float RigidBody::getMass()
{
	return mass;
}

Vec3 RigidBody::relativePosition(const Vec3& worldPoint)
{
	return worldPoint - position;
}

Vec3 RigidBody::pointVelocity(const Vec3& relativePoint)
{
	return linVel + cross(getAngVel(), relativePoint);
}

Vec3 RigidBody::getAngVel()
{
	Mat4 ii = invIntertia();
	return ii.transformVector(momentum);;
}

void RigidBody::setOrientation(const Quat& r)
{
	orientation = r.unit();
}
