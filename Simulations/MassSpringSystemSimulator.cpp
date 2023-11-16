#include "MassSpringSystemSimulator.h"
#include "math.h"
#include <map>
#include <list>
#include <cmath>


void workerLoop(MassSpringSystemSimulator* object)
{
	auto &  o = *object;
	while (o.m_brunning)
	{
		std::unique_lock<std::mutex> lock(o.mutexForAccessToQueue);
		o.cv.wait(lock, [&o] {return !(o.m_iJobsTaken == o.jobQueue->size()); });
		const auto& p = (*(o.jobQueue))[o.m_iJobsTaken];
		++o.m_iJobsTaken;
		lock.unlock();
		for (const auto i : p)
		{
			o.func(i);
		}
		
		++o.m_iJobsDone;
		if (o.m_iJobsDone == o.m_iNumberOfJobsToDo)
		{
			o.cv2.notify_all();
		}
	}
}

void MassSpringSystemSimulator::FillJobQueue(std::function<void(int)> f, const std::vector<std::vector<int>>& v)
{
	std::unique_lock<std::mutex> lock(mutexForAccessToQueue);
	func = f;
	jobQueue = &v;
	m_iJobsDone = 0;
	m_iJobsTaken = 0;
	m_iNumberOfJobsToDo = jobQueue->size();
	cv.notify_all();
	cv2.wait(lock, [this]() { return m_iJobsDone == m_iNumberOfJobsToDo; });
	lock.unlock();
}

// Constructors
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_brunning = true;
	m_iJobsDone = 0;
	m_iJobsTaken = 0;
	m_PointPartition = std::vector<std::vector<int>>{};
	m_SpringPartition = std::vector<std::vector<std::vector<int>>>{};
	jobQueue = &m_PointPartition; // it would be a problem if this Pointer does not always have a valid value
	m_externalForce = Vec3();
	m_fDamping = 0.0;
	m_fFloorLevel = -.3;
	m_fMass = 1.0;
	m_fSphereSize = 0.02;
	m_fStiffness = 10.0;
	m_fWindForce = 0.0;
	m_gravity = Vec3();
	m_iIntegrator = EULER;
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_fFloorBounciness = 0.25;
	m_iJobsDone = 0;
	m_iWorkerNumber = std::max(std::thread::hardware_concurrency(), 1u);
	//testing with my CPU showed me, that too many threads are a waste
	//probably because the problem is too small
	//so it would be optimal if the number of threads would change dynamically with the problem size,
	//but who got time for that
	//m_iWorkerNumber = 2;
	//std::cout << "Worker number = " << m_iWorkerNumber<< std::endl;
	func = [](int i) {std::cout << i << std::endl; };
	for (int i = 0; i < m_iWorkerNumber; i++)
	{
		m_Workers.push_back(std::thread(workerLoop,this)); //todo what happens if the program ends prematurely?
	}

}
MassSpringSystemSimulator::~MassSpringSystemSimulator()
{
	m_brunning = false;
	for (auto& t : m_Workers)
	{
		t.join();
	}
}
void MassSpringSystemSimulator::PartitionPoints()
{
	m_PointPartition = std::vector<std::vector<int>>{};
	//this should partition the points so that there are as many sets as threads
	m_PointPartition;

	for (int i = 0; i < m_iWorkerNumber; i++)
	{
		m_PointPartition.push_back( std::vector<int>{});
	}
	int n = 0;
	for (int j = 0; j < m_iWorkerNumber; j++)
	{
		for (int i = 0; i < (m_vMassPoints.size() / m_iWorkerNumber); ++i)
		{
			m_PointPartition[j].push_back(n); //this might be bad?
			++n;
		}
	}
	for (; n < m_vMassPoints.size(); ++n)
	{
		m_PointPartition[m_iWorkerNumber - 1].push_back(n);
	}
	

}

void MassSpringSystemSimulator::PartitionSprings()
{
	std::map<int, std::list<int>> firstPointFirst{};
	std::map<int, std::list<int>> secondPointFirst{};
	std::vector<int> p1{};
	std::vector<int> p2{};

	for (int i = 0; i < m_vMassPoints.size(); i++)
	{
		p1.push_back(i);
	}

	for (int i = 0; i < m_vSprings.size(); i++)
	{
		int p = m_vSprings[i].point1;
		auto a = firstPointFirst.find(p);
		if (a != firstPointFirst.end())
		{
			a->second.push_back(p);
		}
		else
		{
			firstPointFirst[p] = std::list<int>{p};
		}
		p = m_vSprings[i].point2;
		a = secondPointFirst.find(p);
		if (a != secondPointFirst.end())
		{
			a->second.push_back(p);
		}
		else
		{
			secondPointFirst[p] = std::list<int>{p};
		}
	}
	for (int i = 0; i < m_vSprings.size(); i++)
	{
		firstPointFirst[m_vSprings[i].point2].push_back(i);
	}
	while (p1.size() > 0)
	{
		
		while (p1.size() > 0)
		{
			std::vector<int> v{};

			int index = p1.back();
			p1.pop_back();
			auto a = firstPointFirst.find(index);
			if (a != firstPointFirst.end())
			{
				v.insert(v.end(), a->second.begin(), a->second.end());
				firstPointFirst[index] = std::list<int>{}; //empty this one
			}
			 a = secondPointFirst.find(index);
			if (a != secondPointFirst.end())
			{
				v.insert(v.end(), a->second.begin(), a->second.end());
				firstPointFirst[index] = std::list<int>{};;
			}

			for (auto i : v)
			{

			}


		}
		p1 = p2; //if p2 is empty this terminates obviously
		p2 = std::vector<int>{};
	}
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	/*
		Demo 1 = no printing, just calculate the first step
		Demo 2 = exercise example with Euler
		Demo 3 = exercise example with Midpoint
		Demo 4 = cloth simulation
	*/
	return "Demo 1, Demo 2, Demo 3, Demo 4, Reflection Test";
}



void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_vMassPoints.clear();
	m_vSprings.clear();
}



void MassSpringSystemSimulator::ThreadStuff()
{
	PartitionPoints();
	PartitionSprings();
}

void MassSpringSystemSimulator::clearAllForces() {
	m_gravity = Vec3();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_gravity = force;
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

void MassSpringSystemSimulator::calcAndApplyAllForce(float timeStep)
{
	//clearAllForces();
	externalForcesCalculations(timeStep);

	/*for (auto& p : m_vMassPoints)
	{
		p.force = m_externalForce + (p.mass * m_gravity);
	}*/
	//too slow for our number of points
	MassSpringSystemSimulator::FillJobQueue([this] (int index)
		{
			auto& p = m_vMassPoints[index];
		p.force = m_externalForce + (p.mass * m_gravity);
		}, m_PointPartition);

	calcAndApplyInternalForce();
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeStep) {
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

		static float t = 0.0;

		t += timeStep;

		if (t > 2.0 * M_PI) {
			t -= 2.0 * M_PI;
		}

		switch (m_iTestCase) {
		case 3:
			m_externalForce += Vec3(1, 0, 0) * m_fWindForce * std::abs(std::sin(t * 2.0));
			break;
		default: break;
		}
}


void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (m_iIntegrator) {
	case EULER: integrateEuler(timeStep); break;
	case LEAPFROG: integrateLeapFrog(timeStep); break;
	case MIDPOINT: integrateMidpoint(timeStep); break;
	default: break;
	}

	//for (auto& p : m_vMassPoints)
	//{
	//	if (p.position.y < m_fFloorLevel) {

	//		if (2 * (m_gravity * timeStep).y > p.velocity.y) //I do not want hopping balls because of gravity, so my rule of thumb is that it has to be accelerated for at least 2 time steps
	//		{
	//			Vec3 oldPos = p.position - timeStep * p.velocity; // is this even correct for midpoint intersection? let's just pretend it is
	//			float t = (m_fFloorLevel - oldPos.y) / p.velocity.y;
	//			oldPos += t * p.velocity;
	//			p.velocity = m_fFloorBounciness * Vec3(p.velocity.x, -p.velocity.y, p.velocity.z); //perfect reflection direction, is this really worth the trouble?
	//			p.position = oldPos + (timeStep - t) * p.velocity;
	//		}
	//		else
	//		{
	//			p.velocity.y = 0;
	//			p.position.y = m_fFloorLevel;
	//		}
	//	}
	//}
	
	MassSpringSystemSimulator::FillJobQueue([this, &timeStep](int index) {
		Point& p = m_vMassPoints[index];
		if (p.position.y < m_fFloorLevel) {

			if (2 * (m_gravity * timeStep).y > p.velocity.y) //I do not want hopping balls because of gravity, so my rule of thumb is that it has to be accelerated for at least 2 time steps
			{
				Vec3 oldPos = p.position - timeStep * p.velocity; // is this even correct for midpoint intersection? let's just pretend it is
				float t = (m_fFloorLevel - oldPos.y) / p.velocity.y;
				oldPos += t * p.velocity;
				p.velocity = m_fFloorBounciness * Vec3(p.velocity.x, -p.velocity.y, p.velocity.z); //perfect reflection direction, is this really worth the trouble?
				p.position = oldPos + (timeStep - t) * p.velocity;
			}
			else
			{
				p.velocity.y = 0;
				p.position.y = m_fFloorLevel;
			}
		}
		}, m_PointPartition);

}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	calcAndApplyAllForce(timeStep);

	for (Point& p : m_vMassPoints) {
		if (!p.isFixed) {
			p.position += p.velocity * timeStep;
			p.velocity += (p.force / p.mass) * timeStep;
		}
	}
}

void MassSpringSystemSimulator::integrateLeapFrog(float timeStep)
{
	calcAndApplyAllForce(timeStep);

	for (Point& p : m_vMassPoints) {
		if (!p.isFixed) {
			p.velocity += (p.force / p.mass) * timeStep;
			p.position += p.velocity * timeStep;
		}
	}
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep)
{
	calcAndApplyAllForce(timeStep);

	// save old states
	int numOfPoints = m_vMassPoints.size();
	vector<Vec3> oldPositions(numOfPoints);
	vector<Vec3> oldVelocities(numOfPoints);

	for (int i = 0; i < numOfPoints; i++) {
		oldPositions[i] = getPositionOfMassPoint(i);
		oldVelocities[i] = getVelocityOfMassPoint(i);
	}

	// half Euler to go to the midpoint
	integrateEuler(timeStep * 0.5);

	// recalculate forces in midpoint
	calcAndApplyAllForce(timeStep * 0.5);

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

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, LeapFrog, Midpoint");

	switch (m_iTestCase)
	{
	case 0: break;
	case 1: m_iIntegrator = EULER; break;
	case 2: m_iIntegrator = MIDPOINT; break;
	case 3:

		TwAddVarRW(DUC->g_pTweakBar, "Wind (Force)", TW_TYPE_FLOAT, &m_fWindForce, "min=0.0 step=0.05");
		break;
	default:
		break;
	}

	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	TwAddVarRW(DUC->g_pTweakBar, "Floor Level", TW_TYPE_FLOAT, &m_fFloorLevel, "min=-1.0 step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Floor Bounciness", TW_TYPE_FLOAT, &m_fFloorBounciness, "min=0.0 step=0.05");
	TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.001");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

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
		initClothScene();
		break;
	case 4:
		reset();
		m_fFloorLevel = -.3;
		m_iIntegrator = MIDPOINT;
		m_gravity = Vec3(0, -9.81, 0);
		m_fFloorBounciness = 0.9;
		addMassPoint(Vec3(-1, 0, 0), Vec3(1, 0, 0), false);
		break;
	default: break;
	}
	ThreadStuff();
}

void MassSpringSystemSimulator::initClothScene() {
	reset();

	int n = 20;

	m_externalForce = Vec3(0, 0, 0);
	m_fSphereSize = 5.0 / (n * n);
	m_gravity = Vec3(0, -9.81, 0);
	m_fFloorLevel = -.3;
	setDampingFactor(4.0 / (n * n));
	setMass(0.01f);
	setStiffness(40.0f);

	double len = 2.0f;
	Vec3 offset = Vec3(-len * 0.5, 1.0, -len * 0.5);
	float sideLen = len / n;
	float diagonalLen = M_SQRT2 * sideLen;

	// fixed
	for (int j = 0; j < n; j++) {
		addMassPoint(offset + Vec3(0, 0, j * sideLen), Vec3(), true);
	}

	// moving
	for (int i = 1; i < n; i++) {
		for (int j = 0; j < n; j++) {
			addMassPoint(offset + Vec3(i * sideLen, 0, j * sideLen), Vec3(), false);
		}
	}

	for (int i = 0; i < n - 1; i++) {
		for (int j = 0; j < n; j++) {
			// same row
			addSpring(n * j + i, n * j + i + 1, sideLen);
			// same column
			addSpring(n * i + j, n * (i + 1) + j, sideLen);

			if (j < n - 1) {
				// diagonal
				addSpring(n * i + j, n * (i + 1) + j + 1, diagonalLen);
				addSpring(n * i + j + 1, n * (i + 1) + j, diagonalLen);
			}
		}
	}

	m_fWindForce = 0.1;
	m_iIntegrator = MIDPOINT;
}

void MassSpringSystemSimulator::initTaskScene()
{
	reset();

	m_externalForce = Vec3(0, 0, 0);
	m_fDamping = 0.0;
	m_fMass = 10.0;
	m_fStiffness = 40.0;
	m_fWindForce = 0;
	m_gravity = Vec3(0, 0, 0);
	m_fFloorLevel = -.3;

	setMass(10.0f);
	setDampingFactor(0);
	setStiffness(40.0f);

	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0, 1, 1.0f);
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

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (const Point& p : m_vMassPoints) {
		DUC->drawSphere(p.position, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}

	for (const Spring& s : m_vSprings) {
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

