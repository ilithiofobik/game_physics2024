#ifndef SPHSystemSimulator_h
#define SPHSystemSimulator_h
#include "Simulator.h"
#include "RigidBody.h"
#include "Particle.h"
#include "collisionDetect.h"
#include "SpatialGrid.h"
#include <unordered_map>
#include <unordered_set>
#define TESTCASEUSEDTORUNTEST 2

// taken from: https://lucasschuermann.com/writing/implementing-sph-in-2d
const float POLY6 = 4.0 / M_PI;
const float SPIKY = -10.0 / M_PI;
const float VISC = 40.0 / M_PI;

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
	int getNumberOfRigidBodies();
	Vec3 getAngularVelocityOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getPositionOfRigidBody(int i);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void addParticle(Vec3 position, Vec3 velocity, float density);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void fixCollisions();
	void initFluid();
	void calculatePressureAndDensity();
	void calculateParticleForces();
	void setMomentumOf(int i, Vec3 momentum);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	float randFloat();

private:
	// Attributes
	Vec3 m_externalForce;
	vector<RigidBody> m_vRigidBodies;
	vector<Particle> m_vParticles;
	SpatialGrid sGrid;
	float dampingFactor;
	float bound;
	float h;
	float restDensity;
	float gasConstant;
	Vec3 gravity;
	float particleMass;
	float particleSize;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif