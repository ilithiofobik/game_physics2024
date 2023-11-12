#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Point.h"
#include "Spring.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

#define RED Vec3(1.0, 0.0, 0.0)
#define BLUE Vec3(0.0, 0.0, 1.0)

class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void externalForcesCalculations(float timeElapsed);
	void initUI(DrawingUtilitiesClass* DUC);
	void notifyCaseChanged(int testCase);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void reset();
	void simulateTimestep(float timeStep);

	// Specific Functions
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void applyExternalForce(Vec3 force);
	void calcAndApplyAllForce(float timeStep);
	void calcAndApplyInternalForce();
	void clearAllForces();
	void initClothScene();
	void initTaskScene();
	void integrateEuler(float timeStep);
	void integrateLeapFrog(float timeStep);
	void integrateMidpoint(float timeStep);
	void printState();
	void setDampingFactor(float damping);
	void setMass(float mass);
	void setStiffness(float stiffness);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fDamping;
	float m_fFloorLevel;
	float m_fMass;
	float m_fStiffness;
	float m_fWindForce;
	float m_fFloorBounciness;
	int m_iIntegrator;
	Vec3 m_externalForce;
	Vec3 m_gravity;
	vector<Point> m_vMassPoints;
	vector<Spring> m_vSprings;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fSphereSize;
};
#endif