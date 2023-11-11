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
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, LeapFrog, Midpoint");

	switch (m_iTestCase)
	{
	case 0:break;
	case 1: m_iIntegrator = EULER; break;
	case 2: m_iIntegrator = MIDPOINT; break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_fMass = 10.0;
	m_fStiffness = 40.0;
	m_fDamping = 0.0;

	switch (testCase) {
	case 0:
		// set the scene
		initTaskScene();
		calcAndApplyInternalForce();
		integrateEuler(0.1);
		std::cout << "After Euler" << std::endl;
		printState();
		// set the scene
		initTaskScene();
		calcAndApplyInternalForce();
		integrateMidpoint(0.1);
		std::cout << "After Midpoint" << std::endl;
		printState();
		// empty scene after printing
		m_vMassPoints.clear();
		m_vSprings.clear();
		break;

	case 1: case 2:
		initTaskScene();
		break;

	case 3:
		initRandomScene();
		break;

	default: break;
	}
}

void MassSpringSystemSimulator::initRandomScene() {
	setDampingFactor(1.0);
	m_vMassPoints.clear();
	m_vSprings.clear();

	std::mt19937 eng;
	std::uniform_real_distribution<float> randPos(-2.0f, 2.0f);
	std::uniform_real_distribution<float> randVel(-1.0f, 1.0f);
	std::uniform_real_distribution<float> randLen(0.5f, 1.5f);

	// add a fixed point
	addMassPoint(Vec3(0.0, 0.0, 0.0), Vec3(), true);

	for (int i = 1; i < 10; i++)
	{
		// add random point 
		Vec3 pos = Vec3(randPos(eng), randPos(eng), randPos(eng));
		Vec3 vel = Vec3(randVel(eng), randVel(eng), randVel(eng));
		addMassPoint(pos, vel, false);

		// add random springs to previous ones 
		for (int j = 0; j < i; j++) {
			addSpring(i, j, randLen(eng));
		}
	}
}

void MassSpringSystemSimulator::initTaskScene()
{
	m_vMassPoints.clear();
	m_vSprings.clear();
	// p0
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	// p1 
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	// add spring
	addSpring(0, 1, 1.0);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	m_externalForce = Vec3(); // for now
}

void MassSpringSystemSimulator::printState() {
	int idx = 0;

	for (Point& p : m_vMassPoints) {
		Vec3 pos = p.position;
		Vec3 vel = p.velocity;

		std::cout << "Point " << idx << ":\n";
		std::cout << "	Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
		std::cout << "	Velocity: (" << vel.x << ", " << vel.y << ", " << vel.z << ")\n";

		idx++;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	externalForcesCalculations(timeStep);
	applyExternalForce(m_externalForce);
	calcAndApplyInternalForce();

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
		Vec3 pos1 = getPositionOfMassPoint(s.point1);
		Vec3 pos2 = getPositionOfMassPoint(s.point2);

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

void MassSpringSystemSimulator::calcAndApplyInternalForce()
{
	for (Spring& s : m_vSprings) {
		s.computeElasticForces(m_vMassPoints);
		s.addToEndPoints(m_vMassPoints);
	}

	for (Point& p : m_vMassPoints) {
		Vec3 dampForce = (-m_fDamping) * p.velocity;
		p.addForce(dampForce);
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
	externalForcesCalculations(timeStep / 2.0);
	applyExternalForce(m_externalForce);
	calcAndApplyInternalForce();

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
