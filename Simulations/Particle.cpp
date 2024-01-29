#include "Particle.h"

Particle::Particle(Vec3 position)
{
	pos = position;
	vel = Vec3();
	force = Vec3();
	density = 1.0;
	pressure = 0.0;
}

Vec3 Particle::getPosition() {
	return pos;
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

void Particle::simulateTimestep(float timeStep)
{
	pos += timeStep * vel;
	vel += timeStep * (force / density);
}
