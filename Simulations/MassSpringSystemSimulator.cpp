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
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, LeapFrog, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase) {
	case 0:
		m_vMassPoints.clear();
		m_vSprings.clear();
		m_fMass = 10.0;
		m_fStiffness = 40.0;

		// p0
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		// p1 
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		// add spring
		addSpring(0, 1, 1.0);

		break;
	default: break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	m_externalForce = Vec3(); // for now
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	externalForcesCalculations(timeStep);
	applyExternalForce(m_externalForce);
	calcAndApplyElasticForce();

	switch (m_iIntegrator) {
	case EULER: integrateEuler(timeStep); break;
	case LEAPFROG: integrateLeapFrog(timeStep); break;
	case MIDPOINT: integrateMidpoint(timeStep); break;
	default: break;
	}
}



void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (Point& p : m_vMassPoints) {
		DUC->drawSphere(p.position, SPHERESIZE);
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
		point.clearForce();
		point.addForce(force);
	}
}

void MassSpringSystemSimulator::calcAndApplyElasticForce()
{
	for (Spring& s : m_vSprings) {
		s.computeElasticForces(m_vMassPoints);
		s.addToEndPoints(m_vMassPoints);
	}
}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	for (Point& p : m_vMassPoints) {
		if (!p.isFixed) {
			p.position += p.velocity * timeStep;
			p.velocity += (p.force / p.mass) * timeStep;
		}
	}
}

void MassSpringSystemSimulator::integrateLeapFrog(float timeStep)
{
	for (Point& p : m_vMassPoints) {
		if (!p.isFixed) {
			p.velocity += (p.force / p.mass) * timeStep;
			p.position += p.velocity * timeStep;
		}
	}
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep)
{
	// save old states
	int numOfPoints = m_vMassPoints.size();
	vector<Vec3> oldPositions(numOfPoints);
	vector<Vec3> oldVelocities(numOfPoints);

	for (int i = 0; i < numOfPoints; i++) {
		oldPositions[i] = getPositionOfMassPoint(i);
		oldVelocities[i] = getVelocityOfMassPoint(i);
	}

	// half Euler to go to the midpoint
	integrateEuler(timeStep / 2.0);

	// recalculate forces in midpoint
	externalForcesCalculations(timeStep);
	applyExternalForce(m_externalForce);
	calcAndApplyElasticForce();

	// use midpoint values to integrate
	int idx = 0;
	for (Point& p : m_vMassPoints) {
		if (!p.isFixed) {
			p.position = oldPositions[idx] + p.velocity * timeStep;
			p.velocity = oldVelocities[idx] + (p.force / p.mass) * timeStep;
		}
		idx++;
	}
}
