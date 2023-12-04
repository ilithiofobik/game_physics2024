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

void RigidBodySystemSimulator::setMomentumOf(int i, Vec3 momentum)
{
	m_vRigidBodies[i].momentum = momentum;
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
	reset();

	switch (testCase) {
	case 0:
		// TODO: calculations for demo 1
		addRigidBody(Vec3(), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI / 2));
		applyForceOnBody(0, Vec3(.3, .5, .25), Vec3(1, 1, 0));
		simulateTimestep(2);

		std::cout << "Linear velocity:  " << getLinearVelocityOfRigidBody(0) << "\n";
		std::cout << "Angular velocity: " << getAngularVelocityOfRigidBody(0) << "\n";
		std::cout << "World velocity:   " << m_vRigidBodies[0].pointVelocity(Vec3(-0.3, -0.5, -0.25)) << "\n";

		m_vRigidBodies.clear();
		break;
	case 1:
		addRigidBody(Vec3(), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI / 2));
		applyForceOnBody(0, Vec3(.3, .5, .25), Vec3(1, 1, 0));
		setMomentumOf(0, Vec3(0.1, 0.1, 0.1));
		break;
	case 2:
		addRigidBody(Vec3(-0.25, 0, 0), Vec3(0.1, 0.1, 0.1), 1);
		addRigidBody(Vec3(0.25, 0, 0), Vec3(0.1, 0.1, 0.1), 1);
		setVelocityOf(0, Vec3(0.01, 0, 0));
		setVelocityOf(1, Vec3(-0.01, 0, 0));
		setMomentumOf(0, Vec3(-0.01, 0, 0));
		setMomentumOf(1, Vec3(0.01, 0, 0));
		break;
	case 3:
		initComplex();
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
	externalForcesCalculations(timeStep);

	for (RigidBody& rb : m_vRigidBodies) {
		rb.addForce(rb.getPosition(), m_externalForce);
	}

	for (RigidBody& rb : m_vRigidBodies) {
		rb.simulateTimestep(timeStep);
	}

	fixCollisions();
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


void RigidBodySystemSimulator::fixCollisions() {
	for (int a = 1; a < m_vRigidBodies.size(); a++) {
		for (int b = 0; b < a; b++) {
			XMMATRIX obj2WorldA = m_vRigidBodies[a].objToWorldMatrix().toDirectXMatrix();
			XMMATRIX obj2WorldB = m_vRigidBodies[b].objToWorldMatrix().toDirectXMatrix();

			XMVECTOR sizeA = m_vRigidBodies[a].getPosition().toDirectXVector();
			XMVECTOR sizeB = m_vRigidBodies[b].getPosition().toDirectXVector();

			CollisionInfo info = collisionTools::checkCollisionSATHelper(obj2WorldA, obj2WorldB, sizeA, sizeB);

			if (info.isValid) {
				Vec3 xWorld = info.collisionPointWorld;

				Vec3 xA = m_vRigidBodies[a].relativePosition(xWorld);
				Vec3 xB = m_vRigidBodies[b].relativePosition(xWorld);

				Vec3 vA = m_vRigidBodies[a].pointVelocity(xWorld);
				Vec3 vB = m_vRigidBodies[b].pointVelocity(xWorld);

				Vec3 vRel = vA - vB;

				Vec3 n = info.normalWorld;

				float invMassA = 1.0 / m_vRigidBodies[a].getMass();
				float invMassB = 1.0 / m_vRigidBodies[b].getMass();

				Mat4 iiA = m_vRigidBodies[a].invIntertia();
				Mat4 iiB = m_vRigidBodies[b].invIntertia();

				Vec3 prodA = cross(iiA.transformVector(cross(xA, n)), xA);
				Vec3 prodB = cross(iiB.transformVector(cross(xB, n)), xB);

				float prodAB = dot(prodA + prodB, n);

				float impulse = -2 * dot(vRel, n) / (invMassA + invMassB + prodAB);

				m_vRigidBodies[a].linVel += impulse * n * invMassA;
				m_vRigidBodies[b].linVel -= impulse * n * invMassB;

				m_vRigidBodies[a].momentum += cross(xA, impulse * n);
				m_vRigidBodies[b].momentum -= cross(xB, impulse * n);
			}
		}
	}
}

void RigidBodySystemSimulator::initComplex()
{
	Vec3 size = Vec3(0.1, 0.1, 0.1);
	int idx = 0;

	for (float x : { -1, 1 }) {
		for (float y : { -1, 1 }) {
			for (float z : { -1, 1 }) {
				float coeff = 1.0 + 0.2 * sin((float)idx); // pseudorandom, slight difference
				addRigidBody(0.5 * Vec3(x, y, z), size, 1);
				setVelocityOf(idx, -0.1 * coeff * Vec3(x, y, z));
				setMomentumOf(idx, 0.01 * coeff * Vec3(y * z, z * x, x * y));
				idx++;
			}
		}
	}
}
