#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(uint32_t nn, uint32_t mm)
{
	n = nn;
	m = mm;
	vec_a = std::vector<float>(n * m);
	vec_b = std::vector<float>(n * m);
	is_a_curr = true;
}

float Grid::getCurr(uint32_t i, uint32_t j)
{
	uint32_t idx = pairToIdx(i, j);
	if (is_a_curr) {
		return vec_a[idx];
	}
	return vec_b[idx];
}

void Grid::setNext(uint32_t i, uint32_t j, float v)
{
	uint32_t idx = pairToIdx(i, j);
	if (is_a_curr) {
		vec_b[idx] = v;
	}
	else {
		vec_a[idx] = v;
	}
}

void Grid::update()
{
	is_a_curr = !is_a_curr;
}

//void Grid::resize(uint32_t n, uint32_t m)
//{
//}

uint32_t Grid::pairToIdx(uint32_t i, uint32_t j)
{
	return n * i + j;
}

Real Grid::dx()
{
	double md = m;
	return 1.0 / md;
}

Real Grid::dy()
{
	double nd = n;
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
	alpha = 0.5;
	T = new Grid();
	// to be implemented
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

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	uint32_t n = T->n;
	uint32_t m = T->m;

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		for (uint32_t i = 1; i < m - 1; i++) {
			for (uint32_t j = 1; j < n - 1; j++) {
				T->setNext(i, j, cos(i + j));
			}
		}
		T->update();
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(Real timestep) {//add your own parameters
	// to be implemented
	//make sure that the temperature in boundary cells stays zero

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
			Real diff = alpha * timestep * (diffx + diffy);
			T->setNext(i, j, t_ij + diff);
		}
	}

	T->update();
}


void DiffusionSimulator::fillT(std::vector<Real>& x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	uint32_t n = T->n;
	uint32_t m = T->m;

	for (int i = 1; i < m - 1; i++) {
		for (int j = 1; j < n - 1; j++) {
			uint32_t idx = T->pairToIdx(i, j);
			T->setNext(i, j, idx);
		}
	}

	T->update();
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
		A.set_element(i, i, 1); // set diagonal
	}
}

void setupb(std::vector<Real>& A) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	//for (int i = 0; i < 25; i++) {
	//	A.set_element(i, i, 1); // set diagonal
	//}
}

void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = T->totalSize();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!
	setupA(*A, alpha);
	setupb(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}

Real DiffusionSimulator::sigmoid(Real x)
{
	Real sigmoid = 0.5 + (0.5 * x / (1.0 + abs(x)));
	return sigmoid;
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	const Vec3 sphereSize = 0.01 * Vec3(1.0, 1.0, 1.0);
	uint32_t n = T->n;
	uint32_t m = T->m;
	Real dx = T->dx();
	Real dy = T->dy();

	for (uint32_t i = 0; i < m; i++) {
		for (uint32_t j = 0; j < n; j++) {
			Real t = T->getCurr(i, j);
			Real a = sigmoid(t); // making the color change smooth
			Vec3 color = Vec3(a, 0.0, 1.0 - a);
			DUC->setUpLighting(Vec3(), color, 1.0, color);
			DUC->drawSphere(Vec3(i * dx, j * dy, 0), sphereSize);
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
