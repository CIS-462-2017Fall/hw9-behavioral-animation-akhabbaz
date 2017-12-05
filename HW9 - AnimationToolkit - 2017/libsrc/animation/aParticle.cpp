// AParticle.cpp: implementation of the AParticle class.
//
//////////////////////////////////////////////////////////////////////

#include "AParticle.h"

#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

AParticle::AParticle()
{
	int dim = 12;
	m_dim = dim;

	m_state.resize(m_dim);
	m_stateDot.resize(m_dim);

	m_mass = 1.0;
	setMass(m_mass);

	m_alive = true;
	m_lifeSpan = 10.0;
	setLifeSpan(m_lifeSpan);

	m_gravity = vec3(0.0, -GRAVITY, 0.0);

 }

AParticle::~AParticle()
{

}

void AParticle::setState(vector<float>& newState)
{
	for (int i = 0; i < m_dim; i++)
		m_state[i] = newState[i];
	
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[2]; 
	m_Pos[2] = m_state[3];

	m_Vel[0] = m_state[4];
	m_Vel[1] = m_state[5];
	m_Vel[2] = m_state[6];

}

void AParticle::setState(float *newState)
{
	for (int i = 0; i < m_dim; i++)
		m_state[i] = newState[i];
	
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[2];
	m_Pos[2] = m_state[3];

	m_Vel[0] = m_state[4];
	m_Vel[1] = m_state[5];
	m_Vel[2] = m_state[6];
}
vector<float> AParticle::getState()
{
	return m_state;
}

vector<float> AParticle::getStateDot()
{
	return m_stateDot;
}

//Get the state vector dimension (multiples of 3)
int AParticle::getDim()
{
	return m_dim;
}

//Set the state vector dimension (multiples of 3)
void AParticle::setDim(int dim)
{
	m_dim = dim;
}


//Set mass
void AParticle::setMass(float mass)
{
	m_state[9] = mass;
	m_mass = mass;
}

//Get mass
float AParticle::getMass()
{
	return m_state[9];
}

bool AParticle::isAlive()
{
	if (m_state[10] <= 0.0)
		m_alive = false;

	return m_alive;
}

void AParticle::setAlive()
{
	m_state[10] = m_lifeSpan;
	m_alive = true;

}

// kills particle and sets time to live to 0
void AParticle::kill()
{
	m_state[10] = 0.0;
	m_alive = false;

}

//Get time to live
float AParticle::getTimeToLive()
{
	return m_state[10];
}

//Set time to live
void AParticle::setLifeSpan(float time)
{
	m_lifeSpan = time;
	m_state[10] = time;
}

void AParticle::addForce(vec3 force)
{
	m_state[6] += force[0];
	m_state[7] += force[1];
	m_state[8] += force[2];
}

void AParticle::computeForces(int mode)
{
	// zero out all forces
	m_state[6] = 0.0;
	m_state[7] = 0.0;
	m_state[8] = 0.0;


	// default is gravity force
	addForce(m_mass*m_gravity);


}

void AParticle::computeDynamics(vector<float>& state, vector<float>& stateDot, float deltaT)
{
//       update the derivative of position and velocity using the kinematic equations.
         stateDot[0] = state[3];
	 stateDot[1] = state[4];
	 stateDot[2] = state[5];
	 // 
	 float massInv { 1/ state[9]};
	 stateDot[3] = state[6] * massInv;
	 stateDot[4] = state[7] * massInv;
	 stateDot[5] = state[8] * massInv;
}

void AParticle::updateState(float deltaT, int integratorType)
{

 	computeDynamics(m_state, m_stateDot, deltaT);

	//TODO:  Add your code here to update the state using EULER and Runge Kutta2  integration
	switch (integratorType)
	{
		case EULER:
			for (int i {0}; i < 6; ++i) 
			{
				m_state[i] += m_stateDot[i] * deltaT;
			}
			break;

		case RK2:
		{
			// calculate the next velocity
			vec3 vnext;
			for (int i {0}; i < 3; ++i) 
			{
				 vnext[i] = m_state[i+3] + m_stateDot[i + 3] * deltaT;
			}
			// use the average of the velocity at i and i + 1 to update position
			for (int i{0} ; i < 3; ++i) {
				m_state[i] += 0.5 * (vnext[i] + m_stateDot[i]) * deltaT;
			}
			// the derivative of the velocity is constant so no need to average
			for (int i{3}; i < 6; ++i) {
				m_state[i] += m_stateDot[i] * deltaT;
			}
			break;
		}
	}
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[1];
	m_Pos[2] = m_state[2];

	m_Vel[0] = m_state[3];
	m_Vel[1] = m_state[4];
	m_Vel[2] = m_state[5];
	// added to decrement the time after each step
	m_state[10] -= deltaT;
}

void AParticle::update(float deltaT, int forceMode)
{
	m_deltaT = deltaT;
	if (m_state[10] <= 0.0)
	{
		m_alive = false;
		return;
	}
	computeForces(forceMode);
    updateState(deltaT, RK2);
	
}

void AParticle::initialize(AParticleSystem& parent)
{
	mStartAlpha = parent.mStartAlpha;
	mEndAlpha = parent.mEndAlpha;
	mAlpha = mStartAlpha;

	mStartColor = parent.mStartColor + AJitterVec(parent.mColorJitter);
	mEndColor = parent.mEndColor + AJitterVec(parent.mColorJitter);
	mColor = mStartColor;

	mStartScale = parent.mStartScale + AJitterVal(parent.mScaleJitter);
	mEndScale = parent.mEndScale + AJitterVal(parent.mScaleJitter);
	mScale = mStartScale;

	m_Pos = parent.mStartPos + AJitterVec(parent.mPositionJitter);
	m_Vel = parent.mStartVel + AJitterVec(parent.mVelocityJitter);

	m_gravity = parent.mGravity;

	m_state[0] = m_Pos[0];
	m_state[1] = m_Pos[1];
	m_state[2] = m_Pos[2];
	m_state[3] = m_Vel[0];
	m_state[4] = m_Vel[1];
	m_state[5] = m_Vel[2];
	m_state[6] = m_mass * m_gravity[0];
	m_state[7] = m_mass * m_gravity[1];
	m_state[8] = m_mass * m_gravity[2];
	m_state[9] = m_mass;
	m_state[10] = m_lifeSpan;

	m_alive = true;
}
