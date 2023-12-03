#include "RigidBodySystemSimulator.h"

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_vRigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_vRigidBodies[i].getPosition();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_vRigidBodies[i].linVel;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_vRigidBodies[i].getAngVel();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_vRigidBodies[i].addForce(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody newBody = RigidBody(position, size, mass);
	m_vRigidBodies.push_back(newBody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_vRigidBodies[i].setOrientation(orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_vRigidBodies[i].linVel = velocity;
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_externalForce = Vec3();
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	/*
		Demo 1 = a simple one-step test
		Demo 2 = simple single-body simulation
		Demo 3 = two-rigid-body collision scene
		Demo 4 = complex situation
	*/
	return "Demo 1, Demo 2, Demo 3, Demo 4, Reflection Test";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	// TODO: maybe add sth to the tweakbar
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_vRigidBodies.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (RigidBody& rb : m_vRigidBodies) {
		DUC->drawRigidBody(rb.objToWorldMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (testCase) {
	case 0:
		// TODO: calculations for demo 1
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		break;
	case 1:
		// TODO: init first 
		break;
	case 2:
		// TODO: init second 
		break;
	case 3:
		// TODO: init third 
		break;
	default: break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	m_externalForce = Vec3();

	// copy paste from example
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_externalForce += m_externalForce + inputWorld;
	}

	//copy past end
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (RigidBody& rb : m_vRigidBodies) {
		rb.simulateTimestep(timeStep);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
