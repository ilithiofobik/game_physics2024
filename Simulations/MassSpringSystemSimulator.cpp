#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{

}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "MassSpringSystem";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{

}

void MassSpringSystemSimulator::reset()
{

}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	//main loop
}

void MassSpringSystemSimulator::onClick(int x, int y)
{

}

void MassSpringSystemSimulator::onMouse(int x, int y)
{

}


void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int outcome = 0;
	Point ini_point;
	ini_point.position = position;
	ini_point.velocity = Velocity;
	ini_point.isFixed = isFixed;
	ini_point.mass = m_fMass;
	ini_point.damping = m_fDamping;
	PointList.push_back(ini_point);
	return PointList.size();
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring ini_spring;
	ini_spring.stiffness = m_fDamping;
	ini_spring.point1 = masspoint1;
	ini_spring.point2 = masspoint2;
	ini_spring.initialLength = initialLength;
	SpringList.push_back(ini_spring);
}
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	int outcome = PointList.size();
	return outcome;
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	int outcome = SpringList.size();
	return outcome;
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	Vec3 outcome = PointList[index].position;
	return outcome;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	Vec3 outcome = PointList[index].velocity;
	return outcome;
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}