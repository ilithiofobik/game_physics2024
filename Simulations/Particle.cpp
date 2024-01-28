#include "Particle.h"

Particle::Particle(Vec3 position, Vec3 velocity)
{
	pos = position;
	vel = velocity;
}

Vec3 Particle::getPosition() {
	return pos;
}

void Particle::simulateTimestep(float timeStep)
{
	pos += timeStep * vel;
	vel.y -= 0.01;

	if (pos.x > 0.5) {
		pos.x = 0.5;
		vel.x = -vel.x;
	}

	if (pos.y > 0.5) {
		pos.y = 0.5;
		vel.y = -vel.y;
	}

	if (pos.z > 0.5) {
		pos.z = 0.5;
		vel.z = -vel.z;
	}

	if (pos.x < -0.5) {
		pos.x = -0.5;
		vel.x = -vel.x;
	}

	if (pos.y < -0.5) {
		pos.y = -0.5;
		vel.y = -vel.y;
	}

	if (pos.z < -0.5) {
		pos.z = -0.5;
		vel.z = -vel.z;
	}
}
