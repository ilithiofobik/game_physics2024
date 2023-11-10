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
	m_fSphereSize = 0.05;
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
	this->DUC = DUC;
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	m_externalForce = Vec3(); // for now
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	throw std::logic_error("Implement me!");
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (Point& p : m_vMassPoints) {
		DUC->drawSphere(p.position, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}

	for (Spring& s : m_vSprings) {
		Vec3 pos1 = m_vMassPoints[s.point1].position;
		Vec3 pos2 = m_vMassPoints[s.point2].position;

		DUC->beginLine();
		DUC->drawLine(pos1, RED, pos2, BLUE);
		DUC->endLine();
	}
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

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	m_vMassPoints.push_back(Point(position, velocity, m_fMass, m_fDamping, isFixed));
	return m_vMassPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	m_vSprings.push_back(Spring(masspoint1, masspoint2, m_fStiffness, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_vMassPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return m_vMassPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return m_vMassPoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	for (Point& point : m_vMassPoints) {
		point.force = force;
	}
}
