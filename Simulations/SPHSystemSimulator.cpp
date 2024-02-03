#include "SPHSystemSimulator.h"

SPHSystemSimulator::SPHSystemSimulator()
{
	m_externalForce = Vec3();
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;

	bound = 0.5;
	dampingFactor = 1.0;
	gasStiffness = 3.0;
	gravity = -9.81;
	h = 0.0457;
	particleMass = 0.02;
	restDensity = 998.29;
	sGrid = SpatialGrid();
	viscosity = 3.5;
	particleSize = powf(3.0 * particleMass / (4.0 * M_PI * restDensity), 0.3333334);
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
	TwAddVarRW(DUC->g_pTweakBar, "Particle Size", TW_TYPE_FLOAT, &particleSize, "min=0.01 max=0.1 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Particle Mass", TW_TYPE_FLOAT, &particleMass, "min=0.01 max=0.1 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Gas Stiffness", TW_TYPE_FLOAT, &gasStiffness, "min=1.0 max=10.0 step=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Viscosity", TW_TYPE_FLOAT, &viscosity, "min=1.0 max=10.0 step=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "min=1.0 max=10.0 step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Rest Density", TW_TYPE_FLOAT, &restDensity, "min=100.0 max=1000.0 step=1.0");
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
		initLeapFrog(0.001); // not nice but whatever
		initComplex();
		break;
	default: break;
	}
}

// with gravity
void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	m_externalForce = Vec3(0.0, gravity, 0.0);

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

	// particle part
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

	// collisions
	fixCollisions();
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

void SPHSystemSimulator::applyLinearImpulse(CollisionInfo& info, RigidBody* a, RigidBody* b)
{
	Vec3 xWorld = info.collisionPointWorld;

	Vec3 vA = a->pointVelocity(xWorld);
	Vec3 vB = b->pointVelocity(xWorld);
	Vec3 vRel = vA - vB;

	Vec3 n = info.normalWorld;

	float invMassA = a->getInvMass();
	float invMassB = b->getInvMass();

	// suppose c=1
	float impulse = -2 * dot(vRel, n) / (invMassA + invMassB);

	a->linVel += impulse * n * invMassA;
	b->linVel -= impulse * n * invMassB;
}

void SPHSystemSimulator::fixCollisions() {
	// fix collisions between rigid bodies
	for (int a = 1; a < m_vRigidBodies.size(); a++) {
		for (int b = 0; b < a; b++) {
			CollisionInfo info = getCollisionInfo(&m_vRigidBodies[a], &m_vRigidBodies[b]);

			if (info.isValid) {
				applyImpulse(info, &m_vRigidBodies[a], &m_vRigidBodies[b]);
			}
		}
	}

	// fix collisions between rigid bodies and particles
	for (Particle& p : m_vParticles) {
		auto pos = p.getPosition();
		RigidBody pAsRb = p.toRigidBody(particleSize, particleMass);
		for (int b = 0; b < m_vRigidBodies.size(); b++) {
			CollisionInfo info = getCollisionInfo(&pAsRb, &m_vRigidBodies[b]);

			if (info.isValid) {
				auto n = info.normalWorld;
				auto d = info.depth;
				applyLinearImpulse(info, &pAsRb, &m_vRigidBodies[b]);
				pAsRb.position += 2.0 * d * n;
				//break;
			}
		}
		p.fromRigidBody(pAsRb);
	}
}

float SPHSystemSimulator::randFloat() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void SPHSystemSimulator::initLeapFrog(float timeStep)
{
	calculatePressureAndDensity();
	calculateParticleForces();

	int i = 0;
	for (Particle& p : m_vParticles) {
		p.integrateVelocity(-0.5 * timeStep);
	}
}

void SPHSystemSimulator::initComplex()
{
	int dimensionSize = 10;
	float particleDist = 0.7 * h;
	float sideLen = particleDist * dimensionSize;
	float halfLen = 0.5 * sideLen;
	float halfLen2 = halfLen * halfLen;

	int i = 0;
	for (int x = 0; x < dimensionSize; x++) {
		float px = -halfLen + x * particleDist;
		float px2 = px * px;
		for (int y = 0; y < dimensionSize; y++) {
			float py = -halfLen + y * particleDist;
			float py2 = py * py;
			for (int z = 0; z < dimensionSize; z++) {
				float pz = -halfLen + z * particleDist;
				float pz2 = pz * pz;

				if (px2 + py2 + pz2 > halfLen2) {
					continue;
				}

				Vec3 pos = Vec3(px, py, pz);
				addParticle(pos);
				m_vParticles[i].recalulateGridKey(h);
				sGrid.addValue(m_vParticles[i].getGridKey(), i);
				i++;
			}
		}
	}

	Vec3 wallPos = Vec3(0.0, -1.0, 0.0);
	Vec3 wallSize = Vec3(1.0, 1.0, 1.0);
	addWall(wallPos, wallSize);

	Vec3 boxPos = Vec3(0.0, 0.5, 0.0);
	Vec3 boxSize = 0.1 * Vec3(1.0, 1.0, 1.0);
	Vec3 momentum = 0.1 * Vec3(randFloat(), randFloat(), randFloat());
	addRigidBody(boxPos, boxSize, 1.0);
	setMomentumOf(1, momentum);
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
