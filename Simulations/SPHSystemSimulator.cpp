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

CollisionInfo SPHSystemSimulator::getCollisionInfo(int a, int b)
{
	XMMATRIX obj2WorldA = m_vRigidBodies[a].objToWorldMatrix().toDirectXMatrix();
	XMMATRIX obj2WorldB = m_vRigidBodies[b].objToWorldMatrix().toDirectXMatrix();

	XMVECTOR sizeA = m_vRigidBodies[a].getPosition().toDirectXVector();
	XMVECTOR sizeB = m_vRigidBodies[b].getPosition().toDirectXVector();

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
	Vec3 blue = Vec3(0.2, 0.2, 1.0);
	Vec3 particleScale = particleSize * Vec3(1.0, 1.0, 1.0);

	DUC->setUpLighting(Vec3(), white, 0.5, white);

	for (RigidBody& rb : m_vRigidBodies) {
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
		tuple<int, int, int> oldIdx = p.gridKey;

		p.simulateTimestep(timeStep);
		p.correctPosition(bound, dampingFactor);
		p.recalulateGridKey(h);

		if (oldIdx != p.gridKey) {
			sGrid.removeValue(oldIdx, i);
			sGrid.addValue(p.gridKey, i);
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

void SPHSystemSimulator::fixCollisions() {
	for (int a = 1; a < m_vRigidBodies.size(); a++) {
		for (int b = 0; b < a; b++) {
			CollisionInfo info = getCollisionInfo(a, b);

			if (info.isValid) {
				Vec3 xWorld = info.collisionPointWorld;

				Vec3 xA = m_vRigidBodies[a].relativePosition(xWorld);
				Vec3 xB = m_vRigidBodies[b].relativePosition(xWorld);

				Vec3 vA = m_vRigidBodies[a].pointVelocity(xWorld);
				Vec3 vB = m_vRigidBodies[b].pointVelocity(xWorld);
				Vec3 vRel = vA - vB;

				Vec3 n = info.normalWorld;

				float invMassA = m_vRigidBodies[a].getInvMass();
				float invMassB = m_vRigidBodies[b].getInvMass();

				Mat4 iiA = m_vRigidBodies[a].invIntertia();
				Mat4 iiB = m_vRigidBodies[b].invIntertia();

				Vec3 prodA = cross(iiA.transformVector(cross(xA, n)), xA);
				Vec3 prodB = cross(iiB.transformVector(cross(xB, n)), xB);

				float prodAB = dot(prodA + prodB, n);

				// suppose c=1
				float impulse = -2 * dot(vRel, n) / (invMassA + invMassB + prodAB);

				m_vRigidBodies[a].linVel += impulse * n * invMassA;
				m_vRigidBodies[b].linVel -= impulse * n * invMassB;

				m_vRigidBodies[a].momentum += cross(xA, impulse * n);
				m_vRigidBodies[b].momentum -= cross(xB, impulse * n);
			}
		}
	}
}

float SPHSystemSimulator::randFloat() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

const float defaultKernelCoeff = 315.0 / (64.0 * M_PI);

float SPHSystemSimulator::defaultKernel(float r, float h)
{
	if (r > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h4 = h2 * h2;
	const float h9 = h4 * h4 * h;
	const float coeff = defaultKernelCoeff / h9;
	const float d = h2 - (r * r);
	return coeff * d * d * d;
}

const float pressureGradientCoeff = -45.0 / M_PI;

Vec3 SPHSystemSimulator::pressureGradient(Vec3 r, float rlen, float h)
{
	if (rlen > h) {
		return 0.0;
	}

	Vec3 direction = r / rlen;
	const float h2 = h * h;
	const float h6 = h2 * h2 * h2;
	const float coeff = pressureGradientCoeff / h6;
	const float diff2 = (h - rlen) * (h - rlen);
	return -coeff * diff2 * direction;
}

const float viscosityLaplacianCoeff = -45.0 / M_PI;

float SPHSystemSimulator::viscosityLaplacian(float rlen, float h)
{
	if (rlen > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h6 = h2 * h2 * h2;
	const float coeff = viscosityLaplacianCoeff / h6;
	return coeff * (h - rlen);
}

void SPHSystemSimulator::initSphSystem()
{
	int dimensionSize = 9; // 17^3 is more or less 5000
	float sideLen = 0.24575294117; // more or less 0.1^0.333333
	float particleDist = sideLen / (dimensionSize - 1);
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
				sGrid.addValue(m_vParticles[i].gridKey, i);
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
		p.density = 0.0;
		p.pressure = 0.0;
	}

	for (Particle& p : m_vParticles) {
		Vec3 pos1 = p.getPosition();
		int x0, y0, z0;
		std::tie(x0, y0, z0) = p.gridKey;

		for (int x : {x0 - 1, x0, x0 + 1}) {
			for (int y : {y0 - 1, y0, y0 + 1}) {
				for (int z : {z0 - 1, z0, z0 + 1}) {
					if (sGrid.isEmpty(x, y, z)) {
						continue;
					}

					for (const int& j : sGrid.get(x, y, z)) {
						Vec3 pos2 = m_vParticles[j].getPosition();
						float r = sqrt(pos1.squaredDistanceTo(pos2));
						p.density += defaultKernel(r, h);
					}
				}
			}
		}
	}

	// set pressure
	for (Particle& p : m_vParticles) {
		// multiply density by common coefficient
		p.density *= particleMass;
		p.pressure = gasStiffness * (p.density - restDensity);
	}
}

void SPHSystemSimulator::calculateParticleForces()
{
	// reset forces
	for (Particle& p : m_vParticles) {
		p.forcePress = 0.0;
		p.forceVisc = 0.0;
	}

	for (Particle& p : m_vParticles) {
		Vec3 ri = p.getPosition();
		int x0, y0, z0;
		std::tie(x0, y0, z0) = p.gridKey;

		for (int x : {x0 - 1, x0, x0 + 1}) {
			for (int y : {y0 - 1, y0, y0 + 1}) {
				for (int z : {z0 - 1, z0, z0 + 1}) {
					if (sGrid.isEmpty(x, y, z)) {
						continue;
					}

					for (const int& j : sGrid.get(x, y, z)) {
						if (&p == &m_vParticles[j]) {
							continue;
						}

						Vec3 rj = m_vParticles[j].getPosition();
						Vec3 rij = ri - rj;
						float rlen = sqrt(rij.x * rij.x + rij.y * rij.y + rij.z * rij.z);

						if (rlen < h) {
							float pi = p.pressure;
							float pj = m_vParticles[j].pressure;
							Vec3 ui = p.getVelocity();
							Vec3 uj = m_vParticles[j].getVelocity();
							float rhoi = p.density;
							float rhoj = m_vParticles[j].density;

							Vec3 pressDiff = ((pi / (rhoi * rhoi)) + (pj / (rhoj * rhoj))) * pressureGradient(rij, rlen, h);
							Vec3 viscDiff = ((uj - ui) / rhoj) * viscosityLaplacian(rlen, h);

							p.forcePress += pressDiff;
							p.forceVisc += viscDiff;
						}
					}
				}
			}
		}
	}

	for (Particle& p : m_vParticles) {
		// multiply pressure force by common coefficient
		p.forcePress *= -p.density * particleMass;
		// multiply viscosity force by common coefficient
		p.forceVisc *= viscosity * particleMass;
		// set gravity force according to formula 4.24
		p.forceGrav = gravity * p.density;
	}
}
