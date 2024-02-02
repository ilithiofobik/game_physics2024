#ifndef SPHSystemSimulator_h
#define SPHSystemSimulator_h
#include "Simulator.h"
#include "RigidBody.h"
#include "Particle.h"
#include "collisionDetect.h"
#include "SpatialGrid.h"
#define TESTCASEUSEDTORUNTEST 2

class SPHSystemSimulator :public Simulator {
public:
	// Construtors
	SPHSystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	CollisionInfo getCollisionInfo(int a, int b);
	float defaultKernel(float r, float h);
	float randFloat();
	float viscosityLaplacian(float rlen, float h);
	int getNumberOfRigidBodies();
	Vec3 getAngularVelocityOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getPositionOfRigidBody(int i);
	Vec3 pressureGradient(Vec3 r, float rlen, float h);
	void addParticle(Vec3 position);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void addWall(Vec3 position, Vec3 size);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void calculateParticleForces();
	void calculatePressureAndDensity();
	void fixCollisions();
	void initComplex();
	void initSphSystem();
	void setMomentumOf(int i, Vec3 momentum);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	float bound;
	float dampingFactor;
	float gasStiffness;
	float h;
	float particleMass;
	float particleSize;
	float restDensity;
	float viscosity;
	SpatialGrid sGrid;
	Vec3 gravity;
	Vec3 m_externalForce;
	vector<Particle> m_vParticles;
	vector<RigidBody> m_vRigidBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif