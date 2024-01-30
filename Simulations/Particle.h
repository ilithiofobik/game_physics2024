#ifndef PARTICLE_H
#define PARTICLE_H
#include "util/vectorbase.h"
#include "Simulator.h"

using namespace GamePhysics;

#define PARTICLE_MASS = 1.0 // assume all particles have the same mass
#define PARTICLE_SIZE = 0.1 // assume all particles have the same size
#define CELL_PRECISION = 100;

class Particle {
public:
	// Construtors
	Particle(Vec3 position, float d);

	Vec3 getPosition();
	Vec3 getVelocity();
	void correctPosition(float bound, float dampingFactor);
	void simulateTimestep(float timeStep);

	float density;
	float pressure;

	Vec3 force();
	Vec3 forcePress;
	Vec3 forceVisc;
	Vec3 forceGrav;

	pair<int, int> gridKey;
	void Particle::recalulateGridKey(float h);

private:
	Vec3 pos;
	Vec3 vel;
};
#endif
