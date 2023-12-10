#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "RigidBody.h"
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

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
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void fixCollisions();
	void initComplex();
	void setMomentumOf(int i, Vec3 momentum);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	Vec3 m_externalForce;
	vector<RigidBody> m_vRigidBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif