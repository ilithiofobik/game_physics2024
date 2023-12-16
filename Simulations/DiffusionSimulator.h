#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

class Grid {
public:
	// Construtors
	Grid(uint32_t n = 16, uint32_t m = 16);

	float getCurr(uint32_t i, uint32_t j);
	void setNext(uint32_t i, uint32_t j, float v);
	void update();
	void updateSize(uint32_t n, uint32_t m);
	uint32_t pairToIdx(uint32_t i, uint32_t j);
	Real dx();
	Real dy();
	uint32_t totalSize();

	// Attributes
	uint32_t n;
	uint32_t m;

private:
	// Attributes
	std::vector<Real> vec_a;
	std::vector<Real> vec_b;
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
	void fillT(std::vector<Real>& x);
	SparseMatrix<Real> getMatrixA(Real timestep);
	std::vector<Real> getVectorB();
	std::vector<Real> getVectorX();
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(Real timestep);
	void diffuseTemperatureImplicit(Real timestep);
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
	Real global_alpha;
	Real global_n;
	Real global_m;
};

#endif