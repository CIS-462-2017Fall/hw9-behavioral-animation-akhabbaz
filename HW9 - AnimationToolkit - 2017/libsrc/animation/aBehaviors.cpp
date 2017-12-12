#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"
#include <random>
#include <limits>
std::default_random_engine dre(10231232);
// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
//
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

// Vdesired should be proportional to the distance from the target to the actor
vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	// the bigger this is the higher the velocity
	vec3 Vdesired =  (targetPos - m_actorPos);
	Vdesired.Normalize();
	// angle in body coordinates;
	//mat3 bodyToWorld = actor->getGuide().getLocalRotation();
	//vec3 vbody = bodyToWorld.Transpose() * Vdesired;
	//// angle with respect to the x axis in body coordinates
	//float thetad = atan2(vbody[0], vbody[2]);
	//float theta = actor->getOrientation()[1] - thetad;
	//Vdesired =  BehaviorController::gMaxSpeed * (cos(theta), 0, sin(theta));
	Vdesired *= BehaviorController::gMaxSpeed;
	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}
// flee: velocity should be proportional to the vector from the target
// to the actor
vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 Vdesired = m_actorPos - targetPos;
	Vdesired.Normalize();
	Vdesired *= BehaviorController::gMaxSpeed;
	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}
// added code to get the arrival velocity
vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 Vdesired = BehaviorController::KArrival * (targetPos - m_actorPos);
	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 Vdesired = (m_actorPos - targetPos);
	float constval = BehaviorController::KDeparture /Vdesired.SqrLength();
	Vdesired *= constval;
	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();
	double radius = BehaviorController::gAgentRadius;

	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target

	actor -> setActiveBehaviorType(BehaviorType::ARRIVAL);
	Behavior * arrival { actor ->getActiveBehavior()};
	vec3 Vdesired {arrival -> calcDesiredVel( actor)};
	actor -> setActiveBehaviorType(BehaviorType::AVOID);	
	// Step 2. compute Lb
	double Lb { m_actorVel.Length() * BehaviorController::TAvoid};
	
	// Step 3 & 4 closest obstacle that could collide with actor
	AJoint& joint = actor -> getGuide();
	ATransform& worldToBody {joint.getLocal2Parent().Inverse()};
	double radiusSum;	
	double radius0;
	vec3 closestDistanceB;
	double closestLengthSquare = std::numeric_limits<double>::max();
	bool updated {false};
        for (const Obstacle& current : *mObstacles)
	{
		vec3 obstacleLoc { current.m_Center.getLocalTranslation()};
		vec3 distanceB { worldToBody.RotTrans(obstacleLoc)};
		// possible collision if condition is true if distance is 
		// positive and less than a critical value.
		double radiusSumC { radius + current.m_Radius};
		if ( distanceB[_Z] >= 0  && distanceB[_Z] <=  Lb + radiusSumC) {
			// will collide if no correction taken
			if (std::abs(distanceB[_X]) <= radiusSumC){ 
				double distSqr { distanceB.SqrLength()};
				if (distSqr < closestLengthSquare) {
					closestLengthSquare = distSqr;
					closestDistanceB = distanceB;
					radiusSum = radiusSumC;
					radius0 = current.m_Radius;
					updated = true;
				}
			}
		}
	}
	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	// above code transforms every position to body space to judge whether it is close enough along the direction of motion.
	// That is better than judging by distance alone because the most likely object to collide with may not be the closest.
	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	vec3 Vavoid;
	if (updated) {
		// avoid direction is in world space and is the direction of the x vector
		vec3 Vavoid = worldToBody.m_rotation.GetRow(_X);
		// neg sign to avoid the object
		double VavoidMag { (closestDistanceB[_X] >= 0)? -1.0: 1.0};
		/// vtransversM is always positive
		double vtransverseM { -closestDistanceB[_X] * VavoidMag};
		VavoidMag *= BehaviorController::KAvoid * ( radiusSum - vtransverseM)/radiusSum;
		Vavoid *= VavoidMag;
		Vdesired  += Vavoid; 
	}
	return Vdesired;
	
}

void Avoid::display( BehaviorController* actor)
{
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length()/BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;
	
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 actorVel = actor->getVelocity();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here
	std::uniform_real_distribution<double> d(-1.0, 1.0);
	vec3 randomNoise(d(dre), d(dre), d(dre));
	randomNoise.Normalize();

	// Step2. scale it with a noise factor
	//TODO: add your code here
	randomNoise *= BehaviorController::KNoise;


	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	std:uniform_real_distribution<double>  angle(-M_PI, M_PI);
	double currentAngle { angle(dre)};
	mat3 rotationM { mat3::Rotation3D(1, currentAngle)};
	vec3 Vwander { rotationM * m_Wander };
	// add the randomNoise to this vector
	Vwander += randomNoise;
	// normalize
	Vwander.Normalize();
	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	Vwander *= BehaviorController::KWander;
        vec3 Vdesired { actorVel + Vwander};
	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here




	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	
	
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 
	






	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 






	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vflocking
	// TODO: add your code here 

	




	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 4.0;  float CArrival = 2.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent







	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

