#ifndef PARTICLE_H
#define PARTICLE_H
#include "util/vectorbase.h"
#include "Simulator.h"
#include "SpatialGrid.h"
#include "RigidBody.h"

using namespace GamePhysics;

class Particle {
public:
	// Construtors
	Particle(Vec3 position, float d);

	float defaultKernel(float r, float h);
	float viscosityLaplacian(float rlen, float h);
	RigidBody toRigidBody(float particleSize, float particleMass);
	tuple<int, int, int> getGridKey();
	Vec3 getForce();
	Vec3 getPosition();
	Vec3 getVelocity();
	Vec3 pressureGradient(Vec3 r, float rlen, float h);
	void calculateDensPres(std::vector<Particle>& particles, SpatialGrid& sg, float h);
	void calculateForces(std::vector<Particle>& particles, SpatialGrid& sg, float h);
	void correctDensPres(float pm, float gasStiffness, float restDensity);
	void correctForces(float pm, float visc, float g);
	void correctPosition(float bound, float dampingFactor);
	void fromRigidBody(RigidBody& rb);
	void integrateVelocity(float timeStep);
	void recalulateGridKey(float h);
	void resetDensPres();
	void resetForces();
	void simulateTimestep(float timeStep);

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
