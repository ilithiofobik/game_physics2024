#ifndef PARTICLE_H
#define PARTICLE_H
#include "util/vectorbase.h"
#include "Simulator.h"

using namespace GamePhysics;

#define PARTICLE_MASS = 1.0 // assume all particles have the same mass
#define PARTICLE_SIZE = 0.1 // assume all particles have the same size

class Particle {
public:
	// Construtors
	Particle(Vec3 position, Vec3 velocity);

	Vec3 getPosition();

private:
	float density;
	float pressure;

	Vec3 pos;
	Vec3 vel;
	Vec3 force;
};
#endif
