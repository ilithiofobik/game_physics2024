#include "SPHSystemSimulator.h"
#include <chrono>


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
	gasStiffness = 3;
	h = 0.0457;

	particleSize = 0.01; // constant
	dampingFactor = 0.9;
	bound = 0.5;
	gravity = Vec3(0.0, -9.81, 0.0);
	gasConstant = 2.0;
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

void SPHSystemSimulator::addParticle(Vec3 position, float density)
{
	Particle newParticle = Particle(position, density);
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
	/*
		Demo 1 = fluid
		Demo 2 = simple single-body simulation
		Demo 3 = two-rigid-body collision scene
	*/
	return "Demo 1, Demo 2, Demo 3";
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
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
	for (RigidBody& rb : m_vRigidBodies) {
		DUC->drawRigidBody(rb.objToWorldMatrix());
	}

	Vec3 particleScale = particleSize * Vec3(1.0, 1.0, 1.0);
	//Vec3 particleColor = Vec3(1.0, 0.0, 0.0);

	for (Particle& par : m_vParticles) {
		Vec3 particleColor = par.getPosition() + Vec3(0.5, 0.5, 0.5);
		DUC->setUpLighting(Vec3(), particleColor, 0.5, particleColor);
		DUC->drawSphere(par.getPosition(), particleScale);
	}
}

void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();

	switch (testCase) {
	case 0:
		//initFluid();
		initSphSystem();
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
	default: break;
	}
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	m_externalForce = Vec3();

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

	// fluid part
	//auto start = chrono::high_resolution_clock::now();

	calculatePressureAndDensity();
	calculateParticleForces();

	int i = 0;
	for (Particle& p : m_vParticles) {
		pair<int, int> oldIdx = p.gridKey;

		p.simulateTimestep(timeStep);
		p.correctPosition(bound, dampingFactor);

		p.recalulateGridKey(h);

		if (oldIdx != p.gridKey) {
			sGrid.removeValue(oldIdx.first, oldIdx.second, i);
			sGrid.addValue(p.gridKey.first, p.gridKey.second, i);
		}

		i++;
	}

	//cout << "Position[0]=" << m_vParticles[0].getPosition() << "Velocity[0] = " << m_vParticles[0].getVelocity() << endl;

	//auto stop = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
	//cout << "Time taken by function: " << duration.count() << " microseconds" << endl;
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

// also named spiky
float SPHSystemSimulator::pressureKernel(float r, float h)
{
	if (r > h) {
		return 0.0;
	}

	const float coeff = 15.0 / (M_PI * h * h * h * h * h * h);
	const float d = h - r;
	return coeff * d * d * d;
}

float SPHSystemSimulator::viscosityKernel(float r, float h)
{
	if (r > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h3 = h2 * h;
	const float coeff = 15.0 / (2.0 * M_PI * h3);
	const float rn = max(r, 0.000001f);
	const float rn2 = rn * rn;
	const float rn3 = rn2 * rn;
	const float a = -rn3 / (2.0 * h3);
	const float b = rn2 / h2;
	const float c = h / (2.0 * rn);
	return coeff * (a + b + c - 1.0);
}

float SPHSystemSimulator::defaultKernel(float r, float h)
{
	if (r > h) {
		return 0.0;
	}

	const float h2 = h * h;
	const float h4 = h2 * h2;
	const float h8 = h4 * h4;
	const float coeff = 315.0 / (64.0 * M_PI * h * h8);
	const float d = h2 - (r * r);
	return coeff * d * d * d;
}

void SPHSystemSimulator::initFluid()
{
	int dimensionSize = 5;
	float maxRadius = static_cast<float> (dimensionSize);
	float maxRadius2 = maxRadius * maxRadius;

	int i = 0;
	for (int x = -dimensionSize; x <= dimensionSize; x++) {
		float px = static_cast<float> (x);
		for (int y = -dimensionSize; y <= dimensionSize; y++) {
			float py = static_cast<float> (y);
			for (int z = -dimensionSize; z <= dimensionSize; z++) {
				float pz = static_cast<float> (z);
				float radius2 = px * px + py * py + pz * pz;

				if (radius2 <= maxRadius2) {
					float density = (maxRadius2 - radius2) / maxRadius2 + 0.5;
					Vec3 pos = 0.5 * h * Vec3(px, py, pz);
					addParticle(pos, density);
					m_vParticles[i].recalulateGridKey(h);
					sGrid.addValue(m_vParticles[i].gridKey.first, m_vParticles[i].gridKey.second, i);
					i++;
				}
			}
		}
	}

	srand(time(NULL));

	//for (int i = 0; i < numOfParticles; i++) {
	//	float px = (2.0 * randFloat() - 1.0) * bound;
	//	float py = (2.0 * randFloat() - 1.0) * bound;
	//	float pz = 0.0; // randInBox();
	//	Vec3 pos = Vec3(px, py, pz);

	//	float vx = 0.0;
	//	float vy = 0.0;
	//	float vz = 0.0; // randInBox();
	//	Vec3 vel = Vec3(vx, vy, vz);

		//addParticle(pos, vel);
		//m_vParticles[i].recalulateGridKey(h);
		//sGrid.addValue(m_vParticles[i].gridKey.first, m_vParticles[i].gridKey.second, i);
	//}
}

void SPHSystemSimulator::initSphSystem()
{
	int dimensionSize = 17; // 17^3 is more or less 5000
	float sideLen = 0.4642; // more or less 0.1^0.333333
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
				addParticle(pos, restDensity);
				m_vParticles[i].recalulateGridKey(h);
				sGrid.addValue(m_vParticles[i].gridKey.first, m_vParticles[i].gridKey.second, i);
				i++;
			}
		}
	}
}

void SPHSystemSimulator::calculatePressureAndDensity()
{
	const int numOfParticles = m_vParticles.size();
	const float poly6 = POLY6 / pow(h, 8.0);
	const float hsq = h * h;

	for (Particle& p : m_vParticles) {
		p.density = 0.0;
	}

	// sum up all densities
	// using kernel poly6
	for (Particle& p : m_vParticles) {
		Vec3 pos1 = p.getPosition();
		pair<int, int> idx = p.gridKey;

		for (int x = idx.first - 1; x <= idx.first + 1; x++) {
			for (int y = idx.second - 1; y <= idx.second + 1; y++) {
				if (sGrid.isEmpty(x, y)) {
					continue;
				}

				for (const int& j : sGrid.get(x, y)) {
					Vec3 pos2 = m_vParticles[j].getPosition();
					float dist = pos1.squaredDistanceTo(pos2);

					if (dist < hsq) {
						float diff = pow(hsq - dist, 3.0);
						p.density += diff;
						m_vParticles[j].density += diff;
					}
				}
			}
		}
	}

	// set pressure
	for (Particle& p : m_vParticles) {
		// multiply by partilceMass * poly6 only once
		p.density *= particleMass * poly6;
		p.pressure = gasConstant * (p.density - restDensity);
	}
}

void SPHSystemSimulator::calculateParticleForces()
{
	const int numOfParticles = m_vParticles.size();
	const float spiky = SPIKY / pow(h, 5.0);
	const float visc = VISC / pow(h, 5.0);
	const float hsq = h * h;

	// reset forces
	for (Particle& p : m_vParticles) {
		p.forcePress = 0.0;
		p.forceVisc = 0.0;
		// set according to density computed before
		p.forceGrav = gravity * particleMass / p.density;
	}

	for (Particle& p : m_vParticles) {
		Vec3 ri = p.getPosition();
		pair<int, int> idx = p.gridKey;

		for (int x = idx.first - 1; x <= idx.first + 1; x++) {
			for (int y = idx.second - 1; y <= idx.second + 1; y++) {
				if (sGrid.isEmpty(x, y)) {
					continue;
				}

				for (const int& j : sGrid.get(x, y)) {
					if (&p == &m_vParticles[j]) {
						continue;
					}

					Vec3 rj = m_vParticles[j].getPosition();
					Vec3 rij = rj - ri;
					float r = sqrt(rij.x * rij.x + rij.y * rij.y + rij.z * rij.z);

					if (r < h) {
						auto pi = p.pressure;
						auto pj = m_vParticles[j].pressure;
						auto vi = p.getVelocity();
						auto vj = m_vParticles[j].getVelocity();
						auto rhoi = p.density;
						auto rhoj = m_vParticles[j].density;

						auto pressDiff = (rij / r) * particleMass * ((pi + pj) / 2.0) * spiky * pow(h - r, 3.0);
						auto viscDiff = visc * particleMass * (vj - vi) * (h - r);

						p.forcePress += pressDiff / rhoj;
						p.forceVisc += viscDiff / rhoj;
					}
				}
			}
		}
	}
}


/*
- N_h (average num of neighbours) should be around 30

*/