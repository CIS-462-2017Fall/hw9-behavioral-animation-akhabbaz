#include "aIKController.h"
#include "GL/glut.h"

#include "aActor.h"

#pragma warning (disable : 4018)

int IKController::gIKmaxIterations = 5;
double IKController::gIKEpsilon = 0.1;

// AIKchain class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
AIKchain::AIKchain()
{
	mWeight0 = 0.1;
}

AIKchain::~AIKchain()
{

}

AJoint* AIKchain::getJoint(int index) 
{ 
	return mChain[index]; 
}
void AIKchain::setJoint(int index, AJoint* pJoint) 
{ 
	mChain[index] = pJoint; 
}

double AIKchain::getWeight(int index) 
{ 
	return mWeights[index]; 
}

void AIKchain::setWeight(int index, double weight) 
{ 
	mWeights[index] = weight; 
}

int AIKchain::getSize() 
{ 
	return mChain.size(); 
}

std::vector<AJoint*>& AIKchain::getChain() 
{ 
	return mChain; 
}

std::vector<double>& AIKchain::getWeights() 
{ 
	return mWeights; 
}

void AIKchain::setChain(std::vector<AJoint*> chain) 
{
	mChain = chain; 
}

void AIKchain::setWeights(std::vector<double> weights) 
{ 
	mWeights = weights; 
}

// AIKController class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

IKController::IKController()
{
	m_pActor = NULL;
	m_pSkeleton = NULL;
	mvalidLimbIKchains = false;
	mvalidCCDIKchains = false;

	// Limb IK
	m_pEndJoint = NULL;
	m_pMiddleJoint = NULL;
	m_pBaseJoint = NULL;
	m_rotationAxis = vec3(0.0, 1.0, 0.0);

	ATransform desiredTarget = ATransform();
	mTarget0.setLocal2Parent(desiredTarget);  // target associated with end joint
	mTarget1.setLocal2Parent(desiredTarget);  // optional target associated with middle joint - used to specify rotation of middle joint about end/base axis
	mTarget0.setLocal2Global(desiredTarget);
	mTarget1.setLocal2Global(desiredTarget);

	//CCD IK
	mWeight0 = 0.1;  // default joint rotation weight value

}

IKController::~IKController()
{
}

ASkeleton* IKController::getSkeleton()
{
	return m_pSkeleton;
}

const ASkeleton* IKController::getSkeleton() const
{
	return m_pSkeleton;
}

ASkeleton* IKController::getIKSkeleton()
{
	return &mIKSkeleton;
}

const ASkeleton* IKController::getIKSkeleton() const
{
	return &mIKSkeleton;
}

AActor* IKController::getActor()
{
	return m_pActor;
}

void IKController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}

//  given the end joint ID and the desired size (i.e. length) of the IK chain, 
// 1. add the corresponding skeleton joint pointers to the AIKChain "chain" vector data member starting with the end joint
// 2. also add weight values to the associated AIKChain "weights" vector data member for use in the CCD IK implemention
// Note: desiredChainSize = -1 should create an IK chain of maximum length (i.e. where the last chain joint is the joint before the root joint)
//  add code here to generate chain of desired size or terminate at the joint before root joint, so that root will not change during IK	
// also add weight values to corresponding weights vector  (default value = 0.1)
// Code added to to update the chain and weights
AIKchain IKController::createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton)
{
	bool getMaxSize = false;

	int EndJointID = endJointID;
	AIKchain result;
	// modified so chain and weights refer to the corresponding 
	// structures in result
	std::vector<AJoint*>& chain { result.getChain()};
	std::vector<double>& weights { result.getWeights()};
	// unnecessary clears..
	chain.clear();
	weights.clear();
	if (desiredChainSize == -1)
		getMaxSize = true;
	if (getMaxSize)  {
		desiredChainSize = pSkeleton -> getNumJoints();
	}
	if ((EndJointID >= 0) && (EndJointID < pSkeleton->getNumJoints()))
	{
		AJoint* pJoint = pSkeleton->getJointByID(endJointID);
		while ( pJoint != pSkeleton -> getRootNode() && desiredChainSize-- > 0){
			chain.push_back(pJoint);
			weights.push_back(0.1);
			pJoint = pJoint -> getParent();
		}
                

	}
	return result;
}



bool IKController::IKSolver_Limb(int endJointID, const ATarget& target)
{
	// Implements the analytic/geometric IK method assuming a three joint limb  

	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		//assert(mvalidLimbIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}



int IKController::createLimbIKchains()
{
	bool validChains = false;
	int desiredChainSize = 3;

	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);
	
	if (mLhandIKchain.getSize() == 3 && mRhandIKchain.getSize() == 3 && mLfootIKchain.getSize() == 3 && mRfootIKchain.getSize() == 3)
	{
		validChains = true;
		
		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}
//a, b and c are the lengths where c is opposite to the second angle solved for.
//  output.first  :  angle in radians that the base joint should rotate; "a" goes from
//      the base joint to the middle joint.
//  output.second :  angle in radians that the middle joint rotates. 0 means a, b are in a line
std::pair<double, double> AnglesOfTriangle(double a, double b, double c)
{
		std::pair<double, double> anglepair;
		double cosAngle;
		if (c >= a + b) { 
			cosAngle = -1.0;
		}
		else if ( c <= std::abs(a - b)) {
			cosAngle = 1.0;
		}
		else {
			cosAngle = (a*a + b*b - c * c) / (2 * a * b);
		}
		anglepair.second = std::acos(cosAngle);
		anglepair.second = M_PI - anglepair.second;
                anglepair.first = a * std::sin(anglepair.second)/c;
		anglepair.first = std::asin(anglepair.first);
		return anglepair;
}

// error in global Coordinates
vec3 computeError ( const ATarget& target,  AJoint const * endJoint) {
// 1. compute error vector between target and end joint
	vec3 actualEndPoint { endJoint -> getGlobalTranslation()};
	vec3 targetV { target.getGlobalTranslation()};
	return targetV - actualEndPoint;
}

// displacement = Position (endPoint)  - Position(currentJoint)
vec3 displacementVector(AJoint const * endJoint, AJoint const * currentJoint) 
{
	vec3  displacementEndJoint { endJoint -> getGlobalTranslation() };
	vec3  displacementCurrent { currentJoint -> getGlobalTranslation() };
	return displacementEndJoint - displacementCurrent;
}

int IKController::computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 midJointAxis, ASkeleton* pIKSkeleton)
{
	// TODO: Implement the analytic/geometric IK method assuming a three joint limb  
	// The actual position of the end joint should match the target position within some episilon error 
	// the variable "midJointAxis" contains the rotation axis for the middle joint

	
	bool result = false;
	int endJointID;
	mTarget0 = target;
		// 6. repeat same operations above for each joint in the IKchain from end to base joint

	if (IKchain.getSize() > 0)
		 endJointID = IKchain.getJoint(0)->getID();
	else endJointID = -1;

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		m_pEndJoint = IKchain.getJoint(0);
		m_pMiddleJoint = IKchain.getJoint(1);
		m_pBaseJoint = IKchain.getJoint(2);
		// 1. compute error vector between target and end joint
		vec3 actualEndPoint { m_pEndJoint -> getGlobalTranslation()};
		vec3 targetV { target.getGlobalTranslation()};
		vec3 error = targetV - actualEndPoint;
		//no substantial movement return
		if (error.SqrLength() < IKController::gIKEpsilon * 
				IKController::gIKEpsilon) {
			return false;
		}

		// 2. compute vector between end Joint and base joint
		vec3 baseJoint { m_pBaseJoint -> getGlobalTranslation()};
		vec3 EndPToBase {actualEndPoint - baseJoint};
	        double bL {EndPToBase.Length()};	
		vec3 EndPToBaseN {EndPToBase};
		EndPToBaseN.Normalize();

		// 3. compute vector between target and base joint
		vec3 targetToBase {targetV - baseJoint};
		double TL { targetToBase.Length()};
		targetToBase.Normalize();

		// 4. Compute desired angle for middle joint
		// l2 is the vector from mid to end in middle Joint local frame 
		double l2 {m_pEndJoint -> getLocalTranslation().Length() };
		double l1 {m_pMiddleJoint -> getLocalTranslation().Length() };
		std::pair<double, double> currentAngles { AnglesOfTriangle(l1, l2, bL) };
		std::pair<double, double>  nextAngles { AnglesOfTriangle(l1, l2, TL) };
		double anglediff1 { nextAngles.second - currentAngles.second};
                double anglediff2 { nextAngles.first - currentAngles.first};
		
		// 5. given desired angle and midJointAxis, compute new local middle joint rotation matrix and update joint transform
		quat midJointTransform;
		midJointTransform.FromAxisAngle(midJointAxis, nextAngles.second);
		//mat3 localRot { m_pMiddleJoint -> getLocalRotation()};
		//localRot = localRot * midJointTransform.ToRotation();
		m_pMiddleJoint -> setLocalRotation(midJointTransform.ToRotation());

		// update the base joint.  This rotates the base joint
		// So the limb is at the correct reach and is along the prior end - base axis
		//  but need to update this again below so skip this for now
		if (true) 
		{
			quat baseJointTransform;
			baseJointTransform.FromAxisAngle(midJointAxis, -anglediff2);
			// transform  from midjoint (1) to the basejoint (2)
			mat3 R2_1  { m_pMiddleJoint -> getLocalRotation()};
			// transform from the base (2) to the midjoint (1)
			mat3  R1_2 { R2_1.Inverse() };
			// rotate along joint 1 midaxis. Change basis to the 1 frame and then back
			mat3  BaseRotation =  R2_1 * baseJointTransform.ToRotation()  * R1_2;
			BaseRotation = m_pBaseJoint -> getLocalRotation() * BaseRotation;
			m_pBaseJoint -> setLocalRotation(BaseRotation);
			m_pBaseJoint -> updateTransform();
		}
		else{
			m_pMiddleJoint -> updateTransform();
		}

		
		// 6. compute vector between target and base joint

		actualEndPoint = m_pEndJoint -> getGlobalTranslation();
	        assert (m_pBaseJoint -> getGlobalTranslation() == baseJoint);
		EndPToBase = actualEndPoint - baseJoint;
		// normalize the endpoint to base
		// should match EndPToBaseN
		EndPToBase.Normalize();
		// should be the same as above
 
		// 7. Compute base joint rotation axis (in global coords) and desired angle
		vec3 targetAxis {EndPToBase.Cross(targetToBase)};
		targetAxis.Normalize();
		double rotationAngle { std::acos(Dot(targetToBase, EndPToBase))};

		// 8. transform base joint rotation axis to local coordinates
		mat3  R2_W { m_pBaseJoint -> getGlobalRotation().Inverse()};
		targetAxis = R2_W * targetAxis;
		quat baseJointTransform;
		baseJointTransform.FromAxisAngle(targetAxis, rotationAngle);

		// 9. given desired angle and local rotation axis, compute new local rotation matrix and update base joint transform
		mat3 BaseRotation {m_pBaseJoint -> getLocalRotation() };
		m_pBaseJoint -> setLocalRotation(BaseRotation * baseJointTransform.ToRotation());
		m_pBaseJoint -> updateTransform();
	
	}
	return result;

}

bool IKController::IKSolver_CCD(int endJointID, const ATarget& target)
{
	// Implements the CCD IK method assuming a three joint limb 

	bool validChains = false;

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		//assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computeCCDIK(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createCCDIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}



int IKController::computeCCDIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement CCD IK  
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		m_pMiddleJoint = m_pEndJoint;
		double initError {1E5};
		double finalError { 0};

		// 6. repeat same operations above for each joint in the IKchain from end to base joint
		do {

		  for (int i {1} ; i < IKchain.getSize(); ++i) {

			// 1. compute axis and angle for each joint in the IK chain (distal to proximal) in global coordinates
			m_pMiddleJoint = IKchain.getJoint(i);
			vec3 error { computeError( target, m_pEndJoint)};
			if (i == 1) {
				initError = error.SqrLength();
			}
			vec3 rV { displacementVector(m_pEndJoint, m_pMiddleJoint) };
			vec3 axis { rV.Cross(error)};
			double axisLength { axis.Length()};
			double rVLengthSquare  {rV.SqrLength()};
			double dot { Dot ( error, rV)};
			double angle { axisLength/(rVLengthSquare + dot)};
			axis /= axisLength;		
			// 2. once you have the desired axis and angle, convert axis to local joint coords 
			mat3 WorldToLocal { m_pMiddleJoint -> getGlobalRotation().Inverse()};
			axis = WorldToLocal * axis;
		        // 3. multiply angle by corresponding joint weight value
			angle *= IKchain.getWeight(i);

	        	// 4. compute new local joint rotation matrix
			quat midJointTransform;
			midJointTransform.FromAxisAngle(axis, angle);
			mat3 middleRotation {m_pMiddleJoint -> getLocalRotation() };
			m_pMiddleJoint -> setLocalRotation(middleRotation * 
					midJointTransform.ToRotation());
			

		        // 5. update joint transform
		        m_pMiddleJoint -> updateTransform();
			if ( i ==  IKchain.getSize()  - 1) {
                                     error = computeError( target, m_pEndJoint);
                                     finalError = error.SqrLength();
			}
		  }
		}
		while (  (initError - finalError) > IKController::gIKEpsilon * IKController::gIKEpsilon );
		result = true;
	}
	return result;
}


	// TODO: Implement Pseudo Inverse-based IK  
	// The actual position of the end joint should match the target position after the skeleton is updated with the new joint angles
bool IKController::IKSolver_PseudoInv(int endJointID, const ATarget& target)
{
	bool result = false;
//      implements PseudoInv
	bool validChains = false;

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		return true;
		//assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computePseudoInvIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computePseudoInvIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computePseudoInvIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computePseudoInvIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computePseudoInvIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computePseudoInvIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computePseudoInvIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computePseudoInvIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computePseudoInvIK(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}
int IKController::computePseudoInvIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement PseudoInvIK 
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		m_pMiddleJoint = m_pEndJoint;
		double initError {1E5};
		double finalError { 0};

		// 6. repeat same operations above for each joint in the IKchain from end to base joint
		do {

		  for (int i {1} ; i < IKchain.getSize(); ++i) {

			// 1. compute axis and angle for each joint in the IK chain (distal to proximal) in global coordinates
			m_pMiddleJoint = IKchain.getJoint(i);
			vec3 error { computeError( target, m_pEndJoint)};
			if (i == 1) {
				initError = error.SqrLength();
			}
			vec3 rV { displacementVector(m_pEndJoint, m_pMiddleJoint) };
			vec3 axis { rV.Cross(error)};
			double axisLength { axis.Length()};
			double rVLengthSquare  {rV.SqrLength()};
			double dot { Dot ( error, rV)};
			double angle { axisLength/(rVLengthSquare + dot)};
			axis /= axisLength;		
			// 2. once you have the desired axis and angle, convert axis to local joint coords 
			mat3 WorldToLocal { m_pMiddleJoint -> getGlobalRotation().Inverse()};
			axis = WorldToLocal * axis;
		        // 3. multiply angle by corresponding joint weight value
			angle *= IKchain.getWeight(i);

	        	// 4. compute new local joint rotation matrix
			quat midJointTransform;
			midJointTransform.FromAxisAngle(axis, angle);
			mat3 middleRotation {m_pMiddleJoint -> getLocalRotation() };
			m_pMiddleJoint -> setLocalRotation(middleRotation * 
					midJointTransform.ToRotation());
			

		        // 5. update joint transform
		        m_pMiddleJoint -> updateTransform();
			if ( i ==  IKchain.getSize()  - 1) {
                                     error = computeError( target, m_pEndJoint);
                                     finalError = error.SqrLength();
			}
		  }
		}
		while (  (initError - finalError) > IKController::gIKEpsilon * IKController::gIKEpsilon );
		result = true;
	}
	return result;
}

bool IKController::IKSolver_Other(int endJointID, const ATarget& target)
{
	
	bool result = false;
	
	// TODO: Put Optional IK implementation or enhancements here
	 
	return result;
}
