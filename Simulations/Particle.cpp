#include "Particle.h"

Particle::Particle(Vec3 position, Vec3 velocity)
{
	pos = position;
	vel = velocity;
}

Vec3 Particle::getPosition() {
	return pos;
}