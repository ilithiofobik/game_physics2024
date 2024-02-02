#ifndef PARTICLE_H
#define PARTICLE_H
#include "util/vectorbase.h"
#include "Simulator.h"
#include "SpatialGrid.h"

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
	float defaultKernel(float r, float h);
	void resetForces();
	void calculateForces(std::vector<Particle>& particles, SpatialGrid& sg, float h);
	void correctForces(float pm, float visc, Vec3 g);
	void resetDensPres();
	void calculateDensPres(std::vector<Particle>& particles, SpatialGrid& sg, float h);
	void correctDensPres(float pm, float gasStiffness, float restDensity);
	float viscosityLaplacian(float rlen, float h);
	Vec3 pressureGradient(Vec3 r, float rlen, float h);
	tuple<int, int, int> getGridKey();
	Vec3 getForce();

	void Particle::recalulateGridKey(float h);

private:
	Vec3 pos;
	Vec3 vel;
	Vec3 forcePress;
	Vec3 forceVisc;
	Vec3 forceGrav;
	tuple<int, int, int> gridKey;
	float density;
	float pressure;
};
#endif
