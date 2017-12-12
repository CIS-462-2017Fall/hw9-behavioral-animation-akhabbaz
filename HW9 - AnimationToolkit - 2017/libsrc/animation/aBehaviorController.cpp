#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#include "GL/glew.h"
#include "GL/glut.h"



#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 500.0;
// orig Kv should be 32 so that the settling time is 1/4 of a second
double BehaviorController::dampingRatio = 1;
double BehaviorController::AngularFreq  = 16;
double BehaviorController::gOriKv = 2 * BehaviorController::AngularFreq * 
                                   BehaviorController::dampingRatio;
double BehaviorController::gOriKp = BehaviorController::AngularFreq *
                                    BehaviorController::AngularFreq;
// tv should be .1 second to have a settling time of .4
// so kv should be 10.0
double BehaviorController::gVelKv = 10.0;  
double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 1000.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  

const double M2_PI = M_PI * 2.0;
static const double M_PI_O2 = M_PI/2.0;
void  BehaviorController::updateTimeConstant()
{
	gOriKp = AngularFreq * AngularFreq;
	gOriKv = 2 * dampingRatio * AngularFreq;
}
BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;
        updateTimeConstant();
	reset();

}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();

}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}
	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.
	}
	
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands

{

	if (mpActiveBehavior)
	{ 
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_vd = m_Vdesired.Length();
		m_Vdesired[_Y] = 0;

		//  force and torque inputs are computed from vd and thetad as follows:
		//              Velocity P controller : force = mass * Kv * (vd - v)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot -Kp * (thetad - theta))
		//  where the values of the gains Kv and Kp are different for each controller

		// TODO: insert your code here to compute m_force and m_torque
		vec3 forceWorld {gMass * gVelKv * ( m_Vdesired -  m_Vel0)};
		// the minus sign gets you from world to body
		mat3 worldToBody { mat3::Rotation3D(axisY, -M_PI_O2 + m_state[ORI][_Y])};
		//mat3 worldToBody {m_Guide.getLocalRotation().Transpose()};
		m_force = worldToBody * forceWorld;
		// compute desiredAngle use the angle with respect to the x axis 
		double thetad = atan2(m_Vdesired[_Z], m_Vdesired[_X]);
		double anglediff = thetad - m_state[ORI][_Y];
		ClampAngle(anglediff);
		vec3 angleError(0, anglediff, 0);
		vec3 torque1 = gInertia * gOriKp * angleError;
		vec3 torque2 = gInertia * gOriKv * m_state[AVEL];
		m_torque = torque1 - torque2;
		if ( std::fabs (m_torque[_Y]) < 10 &&  (std::fabs(anglediff - M_PI_O2) < 0.01  || std::fabs(anglediff + M_PI_O2) < 0.01))
		{
			m_torque = 2* torque1;
			m_force *= 2;
		}

		// when agent desired agent velocity and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force[2] = 0.0;
			m_torque[1] = 0.0;
		}
	}
	else
	{
		m_force[2] = 0.0;
		m_torque[1] = 0.0;
	}
        // limit forces
	// maxForce;
	double inputsq = m_force.SqrLength();
	double factor;
	if (inputsq > gMaxForce * gMaxForce)
	{
		factor = gMaxForce/std::sqrt(inputsq);
		m_force *= factor;
	}
	// maxTorque
	inputsq = m_torque.SqrLength();
	if (inputsq > gMaxTorque * gMaxTorque)
	{
		factor = gMaxTorque/std::sqrt(inputsq);
		m_torque *= factor;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}
// use RK2
void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, RK2);
}

// compute dynamics for the no slip condition;  the torque is along the y axis, the force is in the zx plane
// the force y is set in controlInput so that the centrifugal and coriolis forces cancel keeping vx body = 0;
// this sates the state dot and the control input. 
// 	This can change the controlInput in two conditions.  It adds a force to cancel the coriolis force
void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the control input and state vectors
//  This function sets derive vector to appropriate values after being called
//  make this a static function that only depends on the inputs not the stored values
//  does not depend on deltaT
{
	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];
	// the angle used in the body frame is with respect to the xaxis, theta B = pi/2 - thetaworld.
	// this changes the conversions from body to world. Can use the prior rotation matrix. Theta B is
	// stored in the state vectors.
    mat3 bodyToWorld { mat3::Rotation3D(axisY, M_PI_O2 - state[ORI][_Y])};
     //mat3 bodyToWorld { m_Guide.getLocalRotation()};
	// Compute the stateDot vector given the values of the current state vector and control input vector
	stateDot[POS] = bodyToWorld * state[VEL];
	stateDot[ORI] = state[AVEL];
	double y = stateDot[ORI][0];
	assert (std::fabs(y) < .001);
	// if the actor is moving backwards set the force to zero
	//  ( or possibly reverse the force so that it moves forward).
	if (state[VEL][_Z] <= 0.0 && force[_Z] < 0) {
		force[_Z] = 0;
	}
	// add in the Coriolis force to get the body velocities correct
       //	stateDot[VEL] = force/gMass - state[AVEL].Cross(state[VEL]);
	stateDot[VEL] = force/gMass;
	vec3 CoriolisForce = state[AVEL].Cross(state[VEL]);
	// enforce the no slip condition so that Vb x is zero
	// change the force accordingly.
	// the coriolis force along the z and y axes will be zero
	// force = [gmass * corx, 0, inputted] 
	force[_X] = gMass * CoriolisForce[_X];
	// stateDot[VEL] = [0, 0, something]
	stateDot[VEL] = force/gMass - CoriolisForce;
	//torque is [ 0, something, 0]
	stateDot[AVEL] = torque/gInertia;
}

void BehaviorController::updateState(float deltaT, int integratorType)
{
	//  Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integratio
	//  this should be similar to what you implemented in the particle system assignment

	if (integratorType == 0)
	{
		for (size_t i {0}; i < m_state.size(); ++i) {
			m_state[i] += m_stateDot[i] * deltaT;
		}
	}
	// rK2
	else if (integratorType = 1) 
	{
		vector<vec3> stateDotTemp(4);
		vector<vec3> stateTemp(4);
		for (size_t i{0}; i < m_state.size(); ++i) 
		{
			stateTemp[i] = m_state[i] + m_stateDot[i] * deltaT;
		}
		computeDynamics(stateTemp, m_controlInput, stateDotTemp, deltaT);
		for ( size_t i {0}; i < m_state.size(); ++i) 
		{
			// the best estimate of the velocity is the average of 
			// the estimate at the beginning and end of the interval
			m_stateDot[i] = 0.5 * ( m_stateDot[i] + stateDotTemp[i]);
			m_state[i] += deltaT * m_stateDot[i];
		}
	}
	ClampAngle(m_state[ORI][_Y]);
	//  Perform validation check to make sure all values are within MAX values
	// max Velocity in the body frame
	double speed2 = m_state[VEL].SqrLength();
	float factor;
	if (speed2 > gMaxSpeed * gMaxSpeed) {
		factor = gMaxSpeed /std::sqrt(speed2);
		m_VelB *= factor;
		m_state[VEL] = m_VelB;
	}
	// max angular speed
	speed2 = m_state[AVEL].SqrLength();
	if (speed2 > gMaxAngularSpeed * gMaxAngularSpeed)
	{
		factor = gMaxAngularSpeed/std::sqrt(speed2);
		m_AVelB *= factor;
		m_state[AVEL] = m_AVelB;
	}
	// update the guide orientation
	// compute direction from nonzero velocity vector
	// In the end the torque is near zero and this stops
	// wandering.  if the velocity is too small, the current
	// angle is a good proxy and is used to get the direction.
	vec3 dir;
	if (m_Vel0.Length() < 35.0)
	{
//		dir = m_lastVel0;
//		dir.Normalize();;
//		m_state[ORI] = atan2(dir[_Z], dir[_X]);
//		The angle is solid but the direction based on the velocity
//		is not.  Set the direction based on the angle.
		double angle = m_state[ORI][_Y];
		dir = vec3(cos(angle), 0, sin(angle));
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}
	//  given the new values in m_state, these are the new component state values 
	m_Pos0 = m_state[POS];
	m_Vel0 = m_stateDot[POS];
	m_Euler = m_state[ORI];
	m_VelB = m_state[VEL];
	m_AVelB = m_state[AVEL];
	// set the force in the x direction to enforce no slip
	m_force = m_controlInput[0];
	// there are two ways to get the orientation: use the current angle or use 
	// the direction or the velocity vector.  These two are about the same
	// but I think in cases of difference, the angle is better.
	//dir.Normalize();
	//vec3 up(0.0, 1.0, 0.0);
	//vec3 right = up.Cross(dir);
	//right.Normalize();
	//mat3 rot(right, up, dir);
	//rot = rot.Transpose();
	//vec3 diff{ rot.GetRow(0) - bodyToWorld.GetRow(0) };
	//double l{ diff.SqrLength() };
	//assert(l < .3);
	//m_Guide.setLocalRotation(rot);
	mat3 bodyToWorld { mat3::Rotation3D(axisY, M_PI_O2 - m_state[ORI][_Y])};
	m_Guide.setLocalRotation(bodyToWorld);
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}



}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale* getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale* getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(90 - angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehaviorType();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(this);

}

