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
	if (is_a_curr) {
		return vec_a[n * i + j];
	}
	else {
		return vec_b[n * i + j];
	}
}

void Grid::setNext(uint32_t i, uint32_t j, float v)
{
	if (is_a_curr) {
		vec_b[n * i + j] = v;
	}
	else {
		vec_a[n * i + j] = v;
	}
}

void Grid::update()
{
	is_a_curr = !is_a_curr;
}

void Grid::resize(uint32_t n, uint32_t m)
{
	vec_a.resize(n * m);
	vec_b.resize(n * m);
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	T = new Grid();
	uint32_t n = T->n;
	uint32_t m = T->m;
	for (uint32_t i = 1; i < m - 1; i++) {
		for (uint32_t j = 1; j < n - 1; j++) {
			T->setNext(i, j, i * j);
		}
	}
	T->update();
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
	//to be implemented
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

Grid* DiffusionSimulator::diffuseTemperatureExplicit() {//add your own parameters
	//Grid* newT = new Grid();
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return T;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT(Grid* T, std::vector<Real>& v) {//add your own parameters
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	uint32_t n = T->n;
	uint32_t m = T->m;
	for (uint32_t i = 1; i < m - 1; i++) {
		for (uint32_t j = 1; j < n - 1; j++) {
			T->setNext(i, j, v[n * i + j]);
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

void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	uint32_t n = T->n;
	uint32_t m = T->m;
	uint32_t N = n * m; //N = sizeX*sizeY*sizeZ
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

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
	fillT(T, x);//copy x to T
}

Real DiffusionSimulator::sigmoid(Real x)
{
	Real sigmoid = 0.5 + (0.5 * x / (1.0 + abs(x)));
	//std::cout << "sigmoid for x=" << x << " equals s=" << sigmoid << std::endl;
	return sigmoid;
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	const Vec3 sphereSize = 0.1 * Vec3(1.0, 1.0, 1.0);
	uint32_t n = T->n;
	uint32_t m = T->m;

	for (uint32_t i = 0; i < m; i++) {
		for (uint32_t j = 0; j < n; j++) {
			Real t = T->getCurr(i, j);
			Real a = sigmoid(t); // making the color change smooth
			Vec3 color = Vec3(a, 0.0, 1.0 - a);
			DUC->setUpLighting(Vec3(), color, 1.0, color);
			DUC->drawSphere(Vec3(i, j, 0), sphereSize);
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
