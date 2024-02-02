#include "SPHSystemSimulator.h"

SPHSystemSimulator::SPHSystemSimulator()
{
	m_externalForce = Vec3();
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	// my constants
	// water parameters
	restDensity = 998.29;
	particleMass = 0.02;
	viscosity = 3.5; // 1.003 x 10^{-3} ?
	gasStiffness = 3.0;
	h = 0.0457;
	particleSize = 0.01; // constant
	dampingFactor = 0.9;
	bound = 0.5;
	gravity = Vec3(0.0, -9.81, 0.0);
}

int SPHSystemSimulator::getNumberOfRigidBodies()
{
	return m_vRigidBodies.size();
}

Vec3 SPHSystemSimulator::getPositionOfRigidBody(int i)
{
	return m_vRigidBodies[i].getPosition();
}

Vec3 SPHSystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_vRigidBodies[i].linVel;
}

Vec3 SPHSystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_vRigidBodies[i].getAngVel();
}

void SPHSystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_vRigidBodies[i].addForce(loc, force);
}

void SPHSystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody newBody = RigidBody(position, size, mass);
	m_vRigidBodies.push_back(newBody);
}

void SPHSystemSimulator::addWall(Vec3 position, Vec3 size)
{
	RigidBody newBody = RigidBody(position, size);
	m_vRigidBodies.push_back(newBody);
}

void SPHSystemSimulator::addParticle(Vec3 position)
{
	Particle newParticle = Particle(position, restDensity);
	m_vParticles.push_back(newParticle);
}

void SPHSystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_vRigidBodies[i].setOrientation(orientation);
}

void SPHSystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_vRigidBodies[i].linVel = velocity;
}

CollisionInfo SPHSystemSimulator::getCollisionInfo(RigidBody* a, RigidBody* b)
{
	Mat4 matA = a->objToWorldMatrix();
	Mat4 matB = b->objToWorldMatrix();

	XMMATRIX obj2WorldA = matA.toDirectXMatrix();
	XMMATRIX obj2WorldB = matB.toDirectXMatrix();

	Vec3 posA = a->getPosition();
	Vec3 posB = b->getPosition();

	XMVECTOR sizeA = posA.toDirectXVector();
	XMVECTOR sizeB = posB.toDirectXVector();

	return collisionTools::checkCollisionSATHelper(obj2WorldA, obj2WorldB, sizeA, sizeB);
}

void SPHSystemSimulator::setMomentumOf(int i, Vec3 momentum)
{
	m_vRigidBodies[i].momentum = momentum;
}

const char* SPHSystemSimulator::getTestCasesStr()
{
	return "Demo";
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "h", TW_TYPE_FLOAT, &h, "min=0.01 max=0.1");
}

void SPHSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_vRigidBodies.clear();
	m_vParticles.clear();
}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 white = Vec3(1.0, 1.0, 1.0);
	Vec3 blue = Vec3(0.3, 0.3, 1.0);
	Vec3 red = Vec3(1.0, 0.3, 0.3);
	Vec3 particleScale = particleSize * Vec3(1.0, 1.0, 1.0);

	for (RigidBody& rb : m_vRigidBodies) {
		if (rb.isWall()) {
			DUC->setUpLighting(Vec3(), red, 0.5, red);
		}
		else {
			DUC->setUpLighting(Vec3(), white, 0.5, white);
		}

		DUC->drawRigidBody(rb.objToWorldMatrix());
	}

	DUC->setUpLighting(Vec3(), blue, 0.5, blue);

	for (Particle& par : m_vParticles) {
		DUC->drawSphere(par.getPosition(), particleScale);
	}
}

void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();

	switch (testCase) {
	case 0:
		initSphSystem();
		initComplex();
		break;
	default: break;
	}
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	m_externalForce = gravity;

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
}

void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	// rigid body part
	externalForcesCalculations(timeStep);

	for (RigidBody& rb : m_vRigidBodies) {
		rb.addForce(rb.getPosition(), m_externalForce);
	}

	for (RigidBody& rb : m_vRigidBodies) {
		rb.simulateTimestep(timeStep);
	}

	fixCollisions();
	calculatePressureAndDensity();
	calculateParticleForces();

	int i = 0;
	for (Particle& p : m_vParticles) {
		tuple<int, int, int> oldIdx = p.getGridKey();

		p.simulateTimestep(timeStep);
		p.correctPosition(bound, dampingFactor);
		p.recalulateGridKey(h);

		if (oldIdx != p.getGridKey()) {
			sGrid.removeValue(oldIdx, i);
			sGrid.addValue(p.getGridKey(), i);
		}

		i++;
	}
}

void SPHSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSystemSimulator::applyImpulse(CollisionInfo& info, RigidBody* a, RigidBody* b)
{
	Vec3 xWorld = info.collisionPointWorld;

	Vec3 xA = a->relativePosition(xWorld);
	Vec3 xB = b->relativePosition(xWorld);

	Vec3 vA = a->pointVelocity(xWorld);
	Vec3 vB = b->pointVelocity(xWorld);
	Vec3 vRel = vA - vB;

	Vec3 n = info.normalWorld;

	float invMassA = a->getInvMass();
	float invMassB = b->getInvMass();

	Mat4 iiA = a->invIntertia();
	Mat4 iiB = b->invIntertia();

	Vec3 prodA = cross(iiA.transformVector(cross(xA, n)), xA);
	Vec3 prodB = cross(iiB.transformVector(cross(xB, n)), xB);

	float prodAB = dot(prodA + prodB, n);

	// suppose c=1
	float impulse = -2 * dot(vRel, n) / (invMassA + invMassB + prodAB);

	a->linVel += impulse * n * invMassA;
	b->linVel -= impulse * n * invMassB;

	a->momentum += cross(xA, impulse * n);
	b->momentum -= cross(xB, impulse * n);
}

void SPHSystemSimulator::fixCollisions() {
	// fix collisions between rigid bodies and particles
	for (int a = 1; a < m_vRigidBodies.size(); a++) {
		for (int b = 0; b < a; b++) {
			CollisionInfo info = getCollisionInfo(&m_vRigidBodies[a], &m_vRigidBodies[b]);

			if (info.isValid) {
				applyImpulse(info, &m_vRigidBodies[a], &m_vRigidBodies[b]);
			}
		}
	}

	// fix collisions between rigid bodies and particles
}

float SPHSystemSimulator::randFloat() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void SPHSystemSimulator::initSphSystem()
{
	int dimensionSize = 5; // 17^3 is more or less 5000
	float particleDist = h;
	float sideLen = h * dimensionSize; // more or less 0.1^0.333333
	float halfLen = 0.5 * sideLen;

	int i = 0;
	for (int x = 0; x < dimensionSize; x++) {
		float px = -halfLen + x * particleDist;
		for (int y = 0; y < dimensionSize; y++) {
			float py = -halfLen + y * particleDist;
			for (int z = 0; z < dimensionSize; z++) {
				float pz = -halfLen + z * particleDist;
				Vec3 pos = Vec3(px, py, pz);
				addParticle(pos);
				m_vParticles[i].recalulateGridKey(h);
				sGrid.addValue(m_vParticles[i].getGridKey(), i);
				i++;
			}
		}
	}
}

void SPHSystemSimulator::initComplex()
{
	Vec3 size = Vec3(0.1, 0.1, 0.1);
	int idx = 0;
	srand(0);
	vector<int> range = { -1, 0, 1 };

	for (float x : range) {
		for (float y : range) {
			for (float z : range) {
				float phi = rand();
				float mu = rand();
				float coeff = 1.0 + 0.5 * sin(phi); // pseudorandom, slight difference
				addRigidBody(0.3 * Vec3(x, y, z), size, 1);
				setVelocityOf(idx, -0.5 * coeff * Vec3(x, y, z));
				setMomentumOf(idx, 0.05 * coeff * Vec3(sin(phi), cos(phi) * sin(mu), cos(phi) * cos(mu)));
				idx++;
			}
		}
	}

	Vec3 wallPos = Vec3(0.0, -5.5, 0.0);
	Vec3 wallSize = Vec3(100.0, 10.0, 100.0);
	addWall(wallPos, wallSize);
}

void SPHSystemSimulator::calculatePressureAndDensity()
{
	for (Particle& p : m_vParticles) {
		p.resetDensPres();
	}

	for (Particle& p : m_vParticles) {
		p.calculateDensPres(m_vParticles, sGrid, h);
	}

	for (Particle& p : m_vParticles) {
		p.correctDensPres(particleMass, gasStiffness, restDensity);
	}
}

void SPHSystemSimulator::calculateParticleForces()
{
	// reset forces
	for (Particle& p : m_vParticles) {
		p.resetForces();
	}

	// sum up all forces
	for (Particle& p : m_vParticles) {
		p.calculateForces(m_vParticles, sGrid, h);
	}

	// correct forces
	for (Particle& p : m_vParticles) {
		p.correctForces(particleMass, viscosity, gravity);
	}
}
