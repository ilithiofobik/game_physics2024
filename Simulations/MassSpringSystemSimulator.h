#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Point.h"
#include "Spring.h"
#include <thread>
#include <mutex>

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
	~MassSpringSystemSimulator();

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
	void ThreadStuff();
	void PartitionPoints();
	void FillJobQueue(std::function<void(int)>, const std::vector<std::vector<int>>& v);
	int m_iWorkerNumber;
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

	//thread stuff
	public: //I think I am doing something wrong with my program structure and have to make these public
	std::vector<std::vector<int>> m_PointPartition;
	std::vector<std::vector<std::vector<int>>> m_SpringPartition;
	std::atomic<int> m_iJobsDone;
	int m_iJobsTaken;
	int m_iNumberOfJobsToDo;
	bool m_brunning;
	std::vector<std::thread> m_Workers;
	//pointers are only evil, if they modify stuff, my opinion
	//yes, the always have the problem of the object they are pointing to getting deleted,
	//but that should not be a problem in our case
	const std::vector<std::vector<int>> * jobQueue; //not a const pointer, a pointer to a const vector
	std::function<void(int)> func;
	std::mutex mutexForAccessToQueue; //anyone who wants access to the queue should have this
	std::condition_variable cv;
	std::condition_variable cv2;
};
#endif