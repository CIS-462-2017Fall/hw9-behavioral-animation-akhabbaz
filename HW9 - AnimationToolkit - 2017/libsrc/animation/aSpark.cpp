// Spark.cpp: implementation of the ASpark class.
//
//////////////////////////////////////////////////////////////////////

#include "aSpark.h"
#include <math.h>
#include <algorithm>
#include <random>
#ifndef GRAVITY
#define GRAVITY 9.8f
#endif
static const double RepellerStrength { 100000};
static const double AttractorStrength {100000};
static const double DragStrength   {20};
static const double windStrength { 10};
//coefficients of restitution equals 0.25
static const double M_COR_Strength { 0.25};
static const double randomStrength { 3000};
// particles within minDistance of attractor/repeller do not get 
// an infinite force
static const double minDistance { 20};
static std::default_random_engine  dre;
static std::uniform_real_distribution<double>  unireal(-1, 1);
static int RandomForceInterval {10};
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ASpark::ASpark(): m_COR{M_COR_Strength}, AttractorScale{ AttractorStrength},
	      RepellerScale {  RepellerStrength}, dragScale { DragStrength},
	      windScale { windStrength}, randomScale{randomStrength}, count {0}
{}

ASpark::ASpark(float* color): ASpark()
{
	for (int i = 0; i < 3; i++)
		m_color[i] = color[i];
 
}

ASpark::~ASpark()
{

}


//Set attractor position
void ASpark::setAttractor(vec3 position)
{
	m_attractorPos = position;
}

//Set repeller position
void ASpark::setRepeller(vec3 position)
{
	m_repellerPos = position;
}

void ASpark::setWind(vec3 wind)
{
	m_windForce = wind;
}

void ASpark::display()
{
	float fadeTime = 3.0;
	if (m_alive)
	{
		float alpha = 1.0;
		if (m_state[10] < fadeTime)
		{
			alpha = m_state[10] / 10.0f;
		}
		float scale = 1.0;

		glPushMatrix();
		glColor4f(m_color[0], m_color[1], m_color[2], alpha);
		glTranslatef(m_state[0], m_state[1], m_state[2]);
		glScalef(scale, scale, scale);
		glutSolidSphere(1.0, 10, 10);
		glPopMatrix();
	}

}
	


void ASpark::update(float deltaT, int extForceMode)
{
	m_deltaT = deltaT;
	if (m_state[10] <= 0.0)
	{
		m_alive = false;
		return;
	}

	if (!(extForceMode & EXT_SPARKFORCES_ACTIVE))
		extForceMode = 0;
	
	computeForces(extForceMode);
	
	updateState(deltaT, EULER);

	resolveCollisions();
	
	
}


 
void ASpark::computeForces(int extForceMode)
//	computes the forces applied to this spark
{
	// zero out all forces
	m_state[6] = 0.0;
	m_state[7] = 0.0;
	m_state[8] = 0.0;

	// gravity force
	addForce(m_mass*m_gravity);
        ++count;

	// wind force added in
	if (extForceMode & WIND_ACTIVE)
	{
		addForce(windScale * m_windForce);
	}
        // drag force is opposite to velocity
	if (extForceMode & DRAG_ACTIVE)
	{
		
		addForce( - dragScale * m_Vel);
	}


	// attractor force
	if (extForceMode & ATTRACTOR_ACTIVE)
	{
              vec3 forceDir = m_attractorPos - m_Pos;
	      float length = std::max(forceDir.Length(), 
			      minDistance);
	      float invLength = 1/length;
	      forceDir *= AttractorScale * m_mass * invLength *
		      invLength * invLength;
	      addForce(forceDir);
	}
	// repeller force
	if (extForceMode & REPELLER_ACTIVE)
	{
              vec3 forceDir = m_Pos  - m_repellerPos;
	      float length = std::max(forceDir.Length(), 
			      minDistance);
	      float invLength = 1/length;
	      forceDir *= RepellerScale * m_mass * invLength *
		      invLength * invLength;
	      addForce(forceDir);
	}

	// random force
	if ( (extForceMode & RANDOM_ACTIVE) && (count % RandomForceInterval == 0))
	{
          vec3 randomForce( randomScale * unireal(dre), 
			    randomScale * unireal(dre),
			    randomScale * unireal(dre));
	  addForce(randomForce);


	}

}
// reverse the y velocity if the position is negative; then scale 
// the velocity my M_COR to account for collision losses.
void ASpark::resolveCollisions()
// resolves collisions of the spark with the ground
{
	//Added code here that reverses the y value of the spark velocity vector when the y position value of the spark is < 0
	if ( m_Pos[1] < 0) {
		m_state[4] = - m_COR * m_state[4];
		  m_Vel[1] = - m_COR *   m_Vel[1];
	}

}
