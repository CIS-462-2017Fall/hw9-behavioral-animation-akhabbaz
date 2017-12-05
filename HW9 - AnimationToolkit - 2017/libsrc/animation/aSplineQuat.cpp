#include "aSplineQuat.h"
#include <algorithm>
#pragma warning(disable:4018)

constexpr double aThird{ 1 / 3.0 };

ASplineQuat::ASplineQuat() : mDt(1.0 / 120.0), mLooping(true), mType(LINEAR)
{
}

ASplineQuat::~ASplineQuat()
{
}

void ASplineQuat::setInterpolationType(ASplineQuat::InterpolationType type)
{
    mType = type;
    cacheCurve();
}

ASplineQuat::InterpolationType ASplineQuat::getInterpolationType() const
{
    return mType;
}

void ASplineQuat::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineQuat::getLooping() const
{
    return mLooping;
}

void ASplineQuat::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double ASplineQuat::getFramerate() const
{
    return 1.0 / mDt;
}

int ASplineQuat::getCurveSegment(double time)
{
	int segment = 0;
	bool foundSegment = false;

	double t = time;
	if (t < 0.0)
		t = 0.0;

	int numKeys = mKeys.size();
	while (!foundSegment) {
		if (segment == numKeys - 1) {
			segment = numKeys - 2;
			foundSegment = true;
		}
		else {
			double keyTime0 = mKeys[segment].first;
			double keyTime1 = mKeys[segment + 1].first;
			if ((t >= keyTime0) && (t < keyTime1))
				 foundSegment = true;
			else segment++;
		}
	}
	return segment;

}


quat ASplineQuat::getCachedValue(double t)
{

	if (mCachedCurve.size() == 0) return quat();

	int numFrames = (int)(t / mDt);
	int i = numFrames % mCachedCurve.size();
	int inext = (i + 1) % mCachedCurve.size();
	if (!mLooping)
	inext = std::min<int>(inext, mCachedCurve.size() - 1);
	quat key1 = mCachedCurve[i];
	quat key2 = mCachedCurve[inext];
	double u = t - numFrames*mDt;
	return quat::Slerp(key1, key2, u);

}

void ASplineQuat::cacheCurve()
{
	int numKeys = mKeys.size();

	if (mType == LINEAR && numKeys >= 2)
		createSplineCurveLinear();

	if (mType == CUBIC && numKeys >= 2)
	{
		quat startQuat = mKeys[0].second;
		quat endQuat = mKeys[numKeys-1].second;

		computeControlPoints(startQuat, endQuat);
		createSplineCurveCubic();
	}
}
void ASplineQuat::computeControlPoints(quat& startQuat, quat& endQuat)
{
	// startQuat is a phantom point at the left-most side of the spline
	// endQuat is a phantom point at the left-most side of the spline

	mCtrlPoints.clear();
	int numKeys = mKeys.size();
	if (numKeys <= 1) return;

	quat b0, b1, b2, b3;
	quat q_1, q0, q1, q2;

	for (int segment = 0; segment < numKeys-1; segment++)
	{
	// TODO: student implementation goes here
	//  Given the quaternion keys q_1, q0, q1 and q2 associated with a curve segment, compute b0, b1, b2, b3 
	//  for each cubic quaternion curve, then store the results in mCntrlPoints in same the same way 
	//  as was used with the SplineVec implementation
	//  Hint: use the SDouble, SBisect and Slerp to compute b1 and b2
		b0 = mKeys[segment].second;
		b3 = mKeys[segment + 1].second;
		// here q0 is the prior quaternion used for calculating 
		// b1
		q0 = (segment == 0) ? startQuat : mKeys[segment - 1].second;
		// q1 is q(i+1)' reflect and scale q0 about b0
		q1 = quat::SDouble(q0, b0);
		// q1 is now q(i +1)*  mid quaternion between b3 and q(i+1)'
		q1 = quat::SBisect(q1, b3);
		b1 = quat::Slerp(b0, q1, aThird);
		// get the quaternion one past b3 or the endQuat
		q2 = (segment == numKeys - 2) ? endQuat : mKeys[segment + 2].second;
		//qi' reflect and scale q2 about b3
		q_1 = quat::SDouble(q2, b3);
		//qi* 
		q_1 = quat::SBisect(q_1, b0);
		b2  = quat::Slerp(b3, q_1, aThird);
		mCtrlPoints.push_back(b0);
		mCtrlPoints.push_back(b1);
		mCtrlPoints.push_back(b2);
		mCtrlPoints.push_back(b3);
	}
}

quat ASplineQuat::getLinearValue(double t)
{

	quat q;

	int segment = getCurveSegment(t);
	double difference{ mKeys[segment + 1].first - mKeys[segment].first };
	double numerator{ t - mKeys[segment].first };
	double u{ numerator / difference };
	return quat::Slerp(mKeys[segment].second, mKeys[segment + 1].second, u);
}

void ASplineQuat::createSplineCurveLinear()
{

	quat q;
	mCachedCurve.clear();
	int numKeys = mKeys.size(); 
	double startTime = mKeys[0].first;
	double endTime = mKeys[numKeys-1].first;

	for (double t = startTime; t <= endTime; t += mDt)
	{
		q = getLinearValue(t);
		mCachedCurve.push_back(q);
	}
}


quat ASplineQuat::getCubicValue(double t)
{
	quat q, b0, b1, b2, b3;

	int segment = getCurveSegment(t);
	double difference{ mKeys[segment + 1].first - mKeys[segment].first };
	double numerator{ t - mKeys[segment].first };
	double u{ numerator / difference };
	size_t start = segment * 4;
	b0 = mCtrlPoints[start++];
	b1 = mCtrlPoints[start++];
	b2 = mCtrlPoints[start++];
	b3 = mCtrlPoints[start++];
	return quat::Scubic(b0, b1, b2, b3, u);
}

void ASplineQuat::createSplineCurveCubic()
{
	quat q;
	mCachedCurve.clear();
	int numKeys = mKeys.size();
	double startTime = mKeys[0].first;
	double endTime = mKeys[numKeys - 1].first;

	for (double t = startTime; t <= endTime; t += mDt)
	{
		q = getCubicValue(t);
		mCachedCurve.push_back(q);
	}
}


void ASplineQuat::editKey(int keyID, const quat& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
}

void ASplineQuat::appendKey(const quat& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineQuat::appendKey(double t, const quat& value, bool updateCurve)
{
    mKeys.push_back(Key(t, value));
    if (updateCurve) cacheCurve();
}

void ASplineQuat::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID - 1);
}

quat ASplineQuat::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineQuat::getNumKeys() const
{
    return mKeys.size();
}

void ASplineQuat::clear()
{
    mKeys.clear();
}

double ASplineQuat::getDuration() const
{
    return mCachedCurve.size() * mDt;
}

double ASplineQuat::getNormalizedTime(double t) const
{
    double duration = getDuration();
    int rawi = (int)(t / duration);
    return t - rawi*duration;
}
