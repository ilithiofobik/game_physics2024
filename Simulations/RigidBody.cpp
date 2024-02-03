#include "RigidBody.h"

// Assume all rigid bodies are boxes

RigidBody::RigidBody(Vec3 p, Vec3 s, float m)
{
	// precalculation
	Mat4 sm = Mat4();
	sm.initScaling(s.x, s.y, s.z);

	float im = 1.0 / static_cast<float>(m);
	double ix = im * 12.0 / (s.y * s.y + s.z * s.z);
	double iy = im * 12.0 / (s.x * s.x + s.z * s.z);
	double iz = im * 12.0 / (s.y * s.y + s.x * s.x);
	Vec3 ii0 = Vec3(ix, iy, iz);

	// setting
	force = Vec3();
	invIntertia0 = ii0;
	momentum = Vec3();
	linVel = Vec3();
	invMass = im;
	orientation = Quat(0.0, 0.0, 0.0, 1.0);
	position = p;
	scaleMat = sm;
	size = s;
	torque = Vec3();
}

RigidBody::RigidBody(Vec3 p, Vec3 s)
{
	// precalculation
	Mat4 sm = Mat4();
	sm.initScaling(s.x, s.y, s.z);

	// setting
	force = Vec3();
	invIntertia0 = Vec3();
	momentum = Vec3();
	linVel = Vec3();
	invMass = 0.0;
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

const Mat4 RigidBody::objToWorldMatrix()
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
	linVel += timeStep * force * invMass;

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

float RigidBody::getInvMass()
{
	return invMass;
}

Vec3 RigidBody::relativePosition(const Vec3& worldPoint)
{
	return worldPoint - position;
}

Vec3 RigidBody::pointVelocity(const Vec3& worldPoint)
{
	Vec3 relPoint = relativePosition(worldPoint);
	return linVel + cross(getAngVel(), relPoint);
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

bool RigidBody::isWall()
{
	return invMass == 0.0;
}

std::tuple<int, int, int, int, int, int> RigidBody::calculateBV(float h, float particleSize)
{
	Mat4 mat = objToWorldMatrix();
	auto x = mat.toDirectXMatrix();
	x.r;
	Real minX = std::numeric_limits<Real>::max();
	Real minY = minX;
	Real minZ = minX;
	Real maxX = -minX;
	Real maxY = -minX;
	Real maxZ = -minX;

	// check vertices
	for (float x : {-1.0, 1.0}) {
		for (float y : {-1.0, 1.0}) {
			for (float z : {-1.0, 1.0}) {
				Vec3 v = Vec3(x, y, z);
				Vec3 u = mat.transformVector(v);
				//cout << "x=" << u.x << ", y=" << u.y << ", z=" << u.z << endl;
				minX = std::min(v.x, minX);
				minY = std::min(v.y, minY);
				minZ = std::min(v.z, minZ);
				maxX = std::max(v.x, maxX);
				maxY = std::max(v.y, maxY);
				maxZ = std::max(v.z, maxZ);
			}
		}
	}

	int iminX = static_cast<int> ((-particleSize + minX) / h);
	int iminY = static_cast<int> ((-particleSize + minY) / h);
	int iminZ = static_cast<int> ((-particleSize + minZ) / h);
	int imaxX = static_cast<int> ((particleSize + maxX) / h);
	int imaxY = static_cast<int> ((particleSize + maxY) / h);
	int imaxZ = static_cast<int> ((particleSize + maxZ) / h);

	return { iminX, iminY, iminZ, imaxX, imaxY, imaxZ };
}
