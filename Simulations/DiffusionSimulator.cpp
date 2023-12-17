#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(uint32_t nn, uint32_t mm)
{
	n = nn;
	m = mm;
	is_a_curr = true;

	for (uint32_t i = 0; i < MAX_DIM; i++) {
		for (uint32_t j = 0; j < MAX_DIM; j++) {
			vec_a[i][j] = 0;
			vec_b[i][j] = 0;
		}
	}
}

Real Grid::getCurr(uint32_t i, uint32_t j)
{
	if (is_a_curr) {
		return vec_a[i][j];
	}
	return vec_b[i][j];
}

void Grid::setNext(uint32_t i, uint32_t j, Real v)
{
	if (is_a_curr) {
		vec_b[i][j] = v;
	}
	else {
		vec_a[i][j] = v;
	}
}

void Grid::update()
{
	is_a_curr = !is_a_curr;
}

void Grid::updateSize(uint32_t newN, uint32_t newM)
{
	if (m != newM || n != newN) {
		m = newM;
		n = newN;

		for (uint32_t i = 0; i < m; i++) {
			vec_a[i][n - 1] = 0;
			vec_b[i][n - 1] = 0;
		}

		for (uint32_t j = 0; j < n; j++) {
			vec_a[m - 1][j] = 0;
			vec_b[m - 1][j] = 0;
		}
	}
}

uint32_t Grid::pairToIdx(uint32_t i, uint32_t j)
{
	return n * i + j;
}

Real Grid::dx()
{
	Real md = m;
	return 1.0 / md;
}

Real Grid::dy()
{
	Real nd = n;
	return 1.0 / nd;
}

uint32_t Grid::totalSize()
{
	return n * m;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	global_alpha = 0.1;
	global_n = MAX_DIM / 2;
	global_m = MAX_DIM / 2;
	T = new Grid(global_m, global_n);
}

DiffusionSimulator::~DiffusionSimulator()
{
	delete(T);
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_DOUBLE, &global_alpha, "min=0.0 max=1.0");
	// max = MAX_DIM
	TwAddVarRW(DUC->g_pTweakBar, "Length", TW_TYPE_UINT32, &global_n, "min=5 step=5 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Width", TW_TYPE_UINT32, &global_m, "min=5 step=5 max=100");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);

	uint32_t n = T->n;
	uint32_t m = T->m;

	for (uint32_t i = 1; i < m - 1; i++) {
		for (uint32_t j = 1; j < n - 1; j++) {
			uint32_t di = i - m / 2;
			uint32_t dj = j - n / 2;
			T->setNext(i, j, 1000 * cos(di * di + dj * dj));
		}
	}
	T->update();

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(Real timestep) {
	uint32_t n = T->n;
	uint32_t m = T->m;
	Real dx = T->dx();
	Real dy = T->dy();

	for (uint32_t i = 1; i < m - 1; i++) {
		for (uint32_t j = 1; j < n - 1; j++) {
			Real t_ij = T->getCurr(i, j);
			Real t_ipj = T->getCurr(i + 1, j);
			Real t_imj = T->getCurr(i - 1, j);
			Real t_ijp = T->getCurr(i, j + 1);
			Real t_ijm = T->getCurr(i, j - 1);
			Real diffx = (t_ipj - 2.0 * t_ij + t_imj) / dx;
			Real diffy = (t_ijp - 2.0 * t_ij + t_ijm) / dy;
			Real diff = global_alpha * timestep * (diffx + diffy);
			T->setNext(i, j, t_ij + diff);
		}
	}

	T->update();
}


void DiffusionSimulator::fillT(std::vector<Real>& x) {
	uint32_t n = T->n;
	uint32_t m = T->m;

	for (int i = 1; i < m - 1; i++) {
		for (int j = 1; j < n - 1; j++) {
			uint32_t idx = T->pairToIdx(i, j);
			T->setNext(i, j, x[idx]);
		}
	}

	T->update();
}

SparseMatrix<Real> DiffusionSimulator::getMatrixA(Real timestep)
{
	const int n = T->n;
	const int m = T->m;
	const Real dx = T->dx();
	const Real dy = T->dy();
	const int N = T->totalSize();
	SparseMatrix<Real> A = SparseMatrix<Real>(N);

	Real lambdaX = global_alpha * timestep / (dx * dx);
	Real lambdaY = global_alpha * timestep / (dy * dy);

	for (uint32_t i = 0; i < m; i++) {
		for (uint32_t j = 0; j < n; j++) {
			uint32_t idx = T->pairToIdx(i, j);

			A.set_element(idx, idx, 1 + 2 * lambdaX + 2 * lambdaY);

			if (i > 0) {
				uint32_t newIdx = T->pairToIdx(i - 1, j);
				A.set_element(idx, newIdx, -lambdaX);
			}

			if (i < m - 1) {
				uint32_t newIdx = T->pairToIdx(i + 1, j);
				A.set_element(idx, newIdx, -lambdaX);
			}

			if (j > 0) {
				uint32_t newIdx = T->pairToIdx(i, j - 1);
				A.set_element(idx, newIdx, -lambdaY);
			}

			if (j < n - 1) {
				uint32_t newIdx = T->pairToIdx(i, j + 1);
				A.set_element(idx, newIdx, -lambdaY);
			}
		}
	}

	return A;
}

std::vector<Real> DiffusionSimulator::getVectorB()
{
	const int N = T->totalSize();
	std::vector<Real> b(N);

	uint32_t n = T->n;
	uint32_t m = T->m;

	for (int i = 1; i < m - 1; i++) {
		for (int j = 1; j < n - 1; j++) {
			uint32_t idx = T->pairToIdx(i, j);
			b[idx] = T->getCurr(i, j);
		}
	}

	return b;
}

std::vector<Real> DiffusionSimulator::getVectorX()
{
	std::vector<Real> x(T->totalSize());
	return x;
}


void DiffusionSimulator::diffuseTemperatureImplicit(Real timestep) {
	// solve A T = b

	SparseMatrix<Real> A = getMatrixA(timestep);
	std::vector<Real> b = getVectorB();
	std::vector<Real> x = getVectorX();

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);//copy x to T
}

Real DiffusionSimulator::sigmoid(Real x)
{
	return 0.5 + (0.5 * x / (1.0 + abs(x)));
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	T->updateSize(global_n, global_m);

	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	int n = T->n;
	int m = T->m;
	Real dx = T->dx();
	Real dy = T->dy();
	Real size = (dx + dy) / 2;
	const Vec3 sphereSize = size * Vec3(1.0, 1.0, 1.0);

	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			Real t = T->getCurr(i, j);
			Real a = sigmoid(t); // making the color change smooth
			Vec3 color = Vec3(a, 0.0, 1.0 - a);
			DUC->setUpLighting(Vec3(), color, 1.0, color);
			DUC->drawSphere(Vec3((i - m / 2) * size, (j - n / 2) * size, 0), sphereSize);
		}
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
