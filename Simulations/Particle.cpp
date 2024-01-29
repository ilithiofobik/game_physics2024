#include "Particle.h"

Particle::Particle(Vec3 position)
{
	pos = position;
	vel = Vec3();
	density = 1.0;
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

pair<int, int> Particle::gridKey(float h)
{
	int x = static_cast<int> (pos.x / h);
	int y = static_cast<int> (pos.y / h);
	return make_pair(x, y);
}

Vec3 Particle::force()
{
	return forcePress + forceVisc + forceGrav;
}

void Particle::simulateTimestep(float timeStep)
{
	pos += timeStep * vel;
	vel += timeStep * (force() / density);
}
