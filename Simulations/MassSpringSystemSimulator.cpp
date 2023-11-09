#include "MassSpringSystemSimulator.h"

// Constructors
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	// Data Attributes
	m_fMass = 1.0;
	m_fStiffness = 10.0;
	m_fDamping = 0.1;
	m_iIntegrator = EULER;
	// UI Attributes
	m_externalForce = Vec3();
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
};

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void setMass(float mass) { throw std::logic_error("Implement me!"); }
void setStiffness(float stiffness) { throw std::logic_error("Implement me!"); }
void setDampingFactor(float damping) { throw std::logic_error("Implement me!"); }
int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) { throw std::logic_error("Implement me!"); }
void addSpring(int masspoint1, int masspoint2, float initialLength) { throw std::logic_error("Implement me!"); }
int getNumberOfMassPoints() { throw std::logic_error("Implement me!"); }
int getNumberOfSprings() { throw std::logic_error("Implement me!"); }
Vec3 getPositionOfMassPoint(int index) { throw std::logic_error("Implement me!"); }
Vec3 getVelocityOfMassPoint(int index) { throw std::logic_error("Implement me!"); }
void applyExternalForce(Vec3 force) { throw std::logic_error("Implement me!"); }
