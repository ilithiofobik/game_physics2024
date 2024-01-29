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
	Particle(Vec3 position);

	Vec3 getPosition();
	void correctPosition(float bound, float dampingFactor);
	void simulateTimestep(float timeStep);

	float density;
	float pressure;


private:
	Vec3 pos;
	Vec3 vel;
	Vec3 force;
};
#endif
