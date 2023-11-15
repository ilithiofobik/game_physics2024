#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 1.0f;
	m_fStiffness = 3.0f;
	m_fDamping = 0.5f;
	m_fGravity = 0.5f;
	m_externalForce = Vec3(0,-0.01,0);
	PointsRadius = 0.05;

}

void MassSpringSystemSimulator::drawSpringPoints()
{
	for (int i = 0; i < PointList.size(); i++)
	{
		DUC->drawSphere(PointList[i].position, Vec3(PointsRadius, PointsRadius, PointsRadius));
	}
}

void MassSpringSystemSimulator::drawSprings()
{
	for (int i = 0; i < SpringList.size(); i++)
	{
		DUC->drawSphere(0.5 * (PointList[SpringList[i].point2].position + PointList[SpringList[i].point1].position), Vec3(0.01, 0.01, 0.01));
		DUC->drawSphere((0.25 * PointList[SpringList[i].point2].position + 0.75 * PointList[SpringList[i].point1].position), Vec3(0.01, 0.01, 0.01));
		DUC->drawSphere((0.75 * PointList[SpringList[i].point2].position + 0.25 * PointList[SpringList[i].point1].position), Vec3(0.01, 0.01, 0.01));
	}
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler_Method, Mid_Point_Method";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.01 step=0.01");

	addMassPoint(Vec3(0.5, 0.5, 0.5), Vec3(0, 0, 0), true);
	addMassPoint(Vec3(0.5, 0.5, -0.5), Vec3(0, 0, 0), true);
	addMassPoint(Vec3(-0.5, 0.5, 0.5), Vec3(0, 0, 0), true);
	addMassPoint(Vec3(-0.5, 0.5, -0.5), Vec3(0, 0, 0), true);
	addMassPoint(Vec3(0.1, 0.1, 0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(0.1, 0.1, -0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(-0.1, 0.1, 0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(-0.1, 0.1, -0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(0.1, -0.1, 0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(0.1, -0.1, -0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(-0.1,-0.1, 0.1), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(-0.1, -0.1, -0.1), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);

	addSpring(0, 4, 0.3);
	addSpring(1, 5, 0.3);
	addSpring(2, 6, 0.3);
	addSpring(3, 7, 0.3);
	addSpring(4, 5, 0.3);
	addSpring(4, 6, 0.3);
	addSpring(6, 7, 0.3);
	addSpring(5, 7, 0.3);
	addSpring(8, 4, 0.5);
	addSpring(9, 5, 0.5);
	addSpring(10, 6, 0.5);
	addSpring(11, 7, 0.5);
	addSpring(8, 9, 0.5);
	addSpring(8, 10, 0.5);
	addSpring(10, 11, 0.5);
	addSpring(11, 9, 0.5);

}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawSpringPoints();
	drawSprings();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	m_externalForce = Vec3(0, -1 * m_fGravity, 0);
	for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].mass = m_fMass;
	}
	Vec3 move = Vec3(0,0,0);
	float extract = 0;
	for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].ExternalForce = m_externalForce;
		PointList[i].force = m_externalForce;

	}
	for (int i = 0; i < SpringList.size(); i++)
	{
		move = PointList[SpringList[i].point1].position - PointList[SpringList[i].point2].position;
		SpringList[i].currentLength = pow(pow(move.x, 2) + pow(move.Y, 2) + pow(move.z, 2), 0.5);
		extract = MassSpringSystemSimulator::m_fStiffness * (SpringList[i].currentLength - SpringList[i].initialLength);
		PointList[SpringList[i].point1].force += -1 * extract * move / SpringList[i].currentLength;
		PointList[SpringList[i].point2].force += extract * move / SpringList[i].currentLength;
	}
	//Euler Method
	for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].velocity = PointList[i].velocity + timeStep * (PointList[i].force - m_fDamping * PointList[i].velocity) / PointList[i].mass;
		if (!PointList[i].isFixed)
		{	
			PointList[i].position = PointList[i].position + timeStep * PointList[i].velocity;
	 		if (PointList[i].position.y < (PointsRadius - 1))
			{
				PointList[i].position.y = PointsRadius - 1;
			}
		}
	}

	//Midpoint Method
	/*for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].mid_velocity = PointList[i].velocity + timeStep / 2 * (PointList[i].force - m_fDamping * PointList[i].velocity) / PointList[i].mass;
		PointList[i].mid_position = PointList[i].position + timeStep / 2 * PointList[i].velocity;
		if (!PointList[i].isFixed)
		{
			PointList[i].position = PointList[i].position + timeStep * PointList[i].mid_velocity;
			if (PointList[i].position.y < (PointsRadius - 1))
			{
				PointList[i].position.y = PointsRadius - 1;
			}
		}
	}
	for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].mid_force = m_externalForce;
	}
	for (int i = 0; i < SpringList.size(); i++)
	{
		move = PointList[SpringList[i].point1].mid_position - PointList[SpringList[i].point2].mid_position;
		SpringList[i].currentLength = pow(pow(move.x, 2) + pow(move.Y, 2) + pow(move.z, 2), 0.5);
		extract = MassSpringSystemSimulator::m_fStiffness * (SpringList[i].currentLength - SpringList[i].initialLength);
		PointList[SpringList[i].point1].mid_force += -1 * extract * move / SpringList[i].currentLength;
		PointList[SpringList[i].point2].mid_force += extract * move / SpringList[i].currentLength;
	}
	for (int i = 0; i < PointList.size(); i++)
	{
		PointList[i].velocity = PointList[i].velocity + timeStep * (PointList[i].mid_force - m_fDamping * PointList[i].mid_velocity) / PointList[i].mass;
	}*/

}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
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