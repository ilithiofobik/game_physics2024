#include "Particle.h"

Particle::Particle(Vec3 position, float d)
{
	pos = position;
	vel = Vec3();
	density = d;
	pressure = 0.0;
}

Vec3 Particle::getPosition() {
	return pos;
}

Vec3 Particle::getVelocity()
{
	return vel;
}

void Particle::correctPosition(float bound, float dampingFactor) {
	if (pos.x > bound) {
		pos.x = bound;
		vel.x *= -dampingFactor;
	}

	if (pos.y > bound) {
		pos.y = bound;
		vel.y *= -dampingFactor;
	}

	if (pos.z > bound) {
		pos.z = bound;
		vel.z *= -dampingFactor;
	}

	if (pos.x < -bound) {
		pos.x = -bound;
		vel.x *= -dampingFactor;
	}

	if (pos.y < -bound) {
		pos.y = -bound;
		vel.y *= -dampingFactor;
	}

	if (pos.z < -bound) {
		pos.z = -bound;
		vel.z *= -dampingFactor;
	}
}

void Particle::fromRigidBody(RigidBody& rb)
{
	pos = rb.getPosition();
	vel = rb.linVel;
}

void Particle::resetForces()
{
	forcePress = 0.0;
	forceVisc = 0.0;
	forceGrav = 0.0;
}


const float pressureGradientCoeff = -45.0 / M_PI;

Vec3 Particle::pressureGradient(Vec3 r, float rlen, float h)
{
	if (rlen > h) {
		return 0.0;
	}

	Vec3 direction = r / rlen;
	const float h2 = h * h;
	const float h6 = h2 * h2 * h2;
	const float coeff = pressureGradientCoeff / h6;
	const float diff2 = (h - rlen) * (h - rlen);
	return -coeff * diff2 * direction;
}

tuple<int, int, int> Particle::getGridKey()
{
	return gridKey;
}

const float viscosityLaplacianCoeff = -45.0 / M_PI;

float Particle::viscosityLaplacian(float rlen, float h)
{
	if (rlen > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h6 = h2 * h2 * h2;
	const float coeff = viscosityLaplacianCoeff / h6;
	return coeff * (h - rlen);
}

RigidBody Particle::toRigidBody(float particleSize, float particleMass)
{
	Vec3 size = particleSize * sqrt(0.5) * Vec3(1.0, 1.0, 1.0);
	return RigidBody(pos, particleSize, particleMass);
}

void Particle::calculateForces(std::vector<Particle>& particles, SpatialGrid& sg, float h)
{
	int x0, y0, z0;
	std::tie(x0, y0, z0) = gridKey;

	for (int x : {x0 - 1, x0, x0 + 1}) {
		for (int y : {y0 - 1, y0, y0 + 1}) {
			for (int z : {z0 - 1, z0, z0 + 1}) {
				if (sg.isEmpty(x, y, z)) {
					continue;
				}

				for (const int& j : sg.get(x, y, z)) {
					if (this == &particles[j]) {
						continue;
					}

					Vec3 rj = particles[j].getPosition();
					Vec3 rij = pos - rj;
					float rlen = sqrt(rij.x * rij.x + rij.y * rij.y + rij.z * rij.z);

					if (rlen < h) {
						float pi = pressure;
						float pj = particles[j].pressure;
						Vec3 ui = vel;
						Vec3 uj = particles[j].getVelocity();
						float rhoi = density;
						float rhoj = particles[j].density;

						Vec3 pressDiff = ((pi / (rhoi * rhoi)) + (pj / (rhoj * rhoj))) * pressureGradient(rij, rlen, h);
						Vec3 viscDiff = ((uj - ui) / rhoj) * viscosityLaplacian(rlen, h);

						forcePress += pressDiff;
						forceVisc += viscDiff;
					}
				}
			}
		}
	}
}

void Particle::correctForces(float pm, float visc, Vec3 g)
{
	// multiply pressure force by common coefficient
	forcePress *= -density * pm;
	// multiply viscosity force by common coefficient
	forceVisc *= visc * pm;
	// set gravity force according to formula 4.24
	forceGrav = g * density;
}

void Particle::resetDensPres()
{
	density = 0.0;
	pressure = 0.0;
}

const float defaultKernelCoeff = 315.0 / (64.0 * M_PI);

float Particle::defaultKernel(float r, float h)
{
	if (r > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h4 = h2 * h2;
	const float h9 = h4 * h4 * h;
	const float coeff = defaultKernelCoeff / h9;
	const float d = h2 - (r * r);
	return coeff * d * d * d;
}

void Particle::calculateDensPres(std::vector<Particle>& particles, SpatialGrid& sg, float h)
{
	int x0, y0, z0;
	std::tie(x0, y0, z0) = gridKey;

	for (int x : {x0 - 1, x0, x0 + 1}) {
		for (int y : {y0 - 1, y0, y0 + 1}) {
			for (int z : {z0 - 1, z0, z0 + 1}) {
				if (sg.isEmpty(x, y, z)) {
					continue;
				}

				for (const int& j : sg.get(x, y, z)) {
					Vec3 pos2 = particles[j].getPosition();
					float r = sqrt(pos.squaredDistanceTo(pos2));
					density += defaultKernel(r, h);
				}
			}
		}
	}
}

void Particle::correctDensPres(float pm, float gasStiffness, float restDensity)
{
	// multiply density by common coefficient
	density *= pm;
	pressure = gasStiffness * (density - restDensity);
}

void Particle::recalulateGridKey(float h)
{
	int x = static_cast<int> (pos.x / h);
	int y = static_cast<int> (pos.y / h);
	int z = static_cast<int> (pos.z / h);
	gridKey = { x,y,z };
}

Vec3 Particle::getForce()
{
	return forcePress + forceVisc + forceGrav;
}

void Particle::simulateTimestep(float timeStep)
{
	pos += timeStep * vel;
	vel += timeStep * (getForce() / density);
}
