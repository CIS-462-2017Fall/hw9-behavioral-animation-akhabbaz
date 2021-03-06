#ifndef _BEHAVIOR_H
#define _BEHAVIOR_H


#include <string>
#include "aVector.h"
#include "aJoint.h"
#include "aSkeleton.h"
#include "aActor.h"

using namespace std;

class BehaviorController; 
class AActor;

// Behavior is an abstract base class for all behaviors
class Behavior
{
public:
	Behavior();
    Behavior( Behavior& orig);
    virtual ~Behavior() {}
    virtual  string& GetName() ;

    // Given an actor and behavior parameters, return a desired velocity in world coordinates
    virtual vec3 calcDesiredVel( BehaviorController* actor) = 0;
	virtual void display( BehaviorController* actor) {}
	virtual AJoint* getTarget() { return m_pTarget; }
	virtual void setTarget(AJoint* target) { m_pTarget = target; }

protected:
    Behavior( char* name);
    string m_name;
    vec3 m_actorPos;
    vec3 m_actorVel;
    AJoint* m_pTarget;
};

// Derived classes: Seek, Flee, Arrival, Departure, Avoid, Wander, Alignment, Separation, Cohesion, Flocking, Leader
///////////////////////////////////////////////////////////////////////////////
class Seek : public Behavior
{
public:
    Seek( AJoint* target);
    Seek( Seek& orig);
    virtual ~Seek();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};

///////////////////////////////////////////////////////////////////////////////
class Flee : public Behavior
{
public:
    Flee( AJoint* target);
    Flee( Flee& orig);
    virtual ~Flee();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
	
protected:

};

///////////////////////////////////////////////////////////////////////////////
class Arrival : public Behavior
{
public:
	Arrival( AJoint* target);
	Arrival( Arrival& orig);
	virtual ~Arrival();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};


///////////////////////////////////////////////////////////////////////////////
class Departure : public Behavior
{
public:
	Departure( AJoint* target);
	Departure( Departure& orig);
	virtual ~Departure();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};


///////////////////////////////////////////////////////////////////////////////
class Obstacle
{
public:
	Obstacle() {}
	virtual ~Obstacle() {}
	double m_Radius;
	AJoint m_Center;
};


///////////////////////////////////////////////////////////////////////////////
class Avoid : public Behavior
{
public:
    Avoid(AJoint* target, vector<Obstacle>* obstacles);
    Avoid( Avoid& orig);
    virtual ~Avoid();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
	virtual void display( BehaviorController* actor);

protected:

    vector<Obstacle>* mObstacles;
    //AJoint& m_Target;
	vec3 m_obstaclePos;
};

///////////////////////////////////////////////////////////////////////////////
class Wander : public Behavior
{
public:
	Wander();
	Wander( Wander& orig);
	virtual ~Wander();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

public:
	// the current direction
	vec3 m_Wander;
};

///////////////////////////////////////////////////////////////////////////////
//This class gets the next neigbor of an actor
class FlockBehavior : public Behavior
{
public:
	FlockBehavior();
	FlockBehavior(AJoint* target, vector<AActor>* agents);
	FlockBehavior( FlockBehavior& orig);
	FlockBehavior(string name, FlockBehavior& orig);
	virtual ~FlockBehavior();
	typedef vector<AActor>::iterator Neighbor;
	// first Possible neigbor
	Neighbor  first() { return start;}
	// returns the iterator to the next valid neighbor within neighborhood. 
	// input is the pointer to any Actor in the list  This will not 
	// return the self.  Distance is the distance from 
	// the current valid neighbor.  LengthSq is the length to the actor.
	// distance points from Actor to neighbor
	Neighbor  nextValid(Neighbor possibleNeigbor, BehaviorController * actor, 
			vec3 * toNeighbor, double* LengthSq);
	Neighbor  endNeighbor() {return endit;}

	virtual vec3 calcDesiredVel( BehaviorController* actor) = 0;

protected:
	vector<AActor>* m_pAgentList;
	Neighbor start;
	Neighbor endit;
	static vector<AActor> NoActors;
	// AJoint& m_Target;
};
///////////////////////////////////////////////////////////////////////////////
class Alignment : public FlockBehavior
{
public:
	Alignment(AJoint* target, vector<AActor>* agents);
	Alignment( Alignment& orig);
	virtual ~Alignment();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

	// AJoint& m_Target;
};

///////////////////////////////////////////////////////////////////////////////
class Separation : public FlockBehavior
{
public:
	Separation( AJoint* target,  vector<AActor>* agents);
    Separation( Separation& orig);
    virtual ~Separation();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

};

///////////////////////////////////////////////////////////////////////////////
class Cohesion : public FlockBehavior
{
public:
    Cohesion( vector<AActor>* agents);
    Cohesion( Cohesion& orig);
    virtual ~Cohesion();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

};

///////////////////////////////////////////////////////////////////////////////
class Flocking : public FlockBehavior
{
public:
    Flocking( AJoint* target,  vector<AActor>* agents);
    Flocking( Flocking& orig);
    virtual ~Flocking();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

    static double CSep;
    static double Calign;
    static double Ccoh;
};

///////////////////////////////////////////////////////////////////////////////
class Leader : public FlockBehavior
{
public:
    Leader( AJoint* target, vector<AActor>* agents);
    Leader( Leader& orig);
    virtual ~Leader();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
    static double CSeparation;
    static double Carrival;
};

///////////////////////////////////////////////////////////////////////////////
class GuidedAvoid: public FlockBehavior
{
public:
    GuidedAvoid( AJoint* target, vector<AActor>* agents);
    GuidedAvoid( GuidedAvoid& orig);
    virtual ~GuidedAvoid();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
    static double Cleader;
    static double CAvoid;
};
#endif
