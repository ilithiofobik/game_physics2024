#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(uint32_t n = 5, uint32_t m = 5);

	float getCurr(uint32_t i, uint32_t j);
	void setNext(uint32_t i, uint32_t j, float v);
	void update();
	void resize(uint32_t n, uint32_t m);

	// Attributes
	uint32_t n;
	uint32_t m;

private:
	// Attributes
	std::vector<float> vec_a;
	std::vector<float> vec_b;
	bool is_a_curr;
};

class DiffusionSimulator :public Simulator {
public:
	// Construtors
	DiffusionSimulator();
	// Deconstrutors
	~DiffusionSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();
	Real sigmoid(Real x);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid* T; //save results of every time step
};

#endif