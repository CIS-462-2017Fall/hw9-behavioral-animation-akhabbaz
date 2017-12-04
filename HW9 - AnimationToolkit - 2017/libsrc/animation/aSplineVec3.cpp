#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen/Dense>
#include <iterator>

#pragma warning(disable:4018)
#pragma warning(disable:4244)
Eigen::Matrix4d loadBSpline();
static constexpr double aThird{ 1 / 3.0 };
static constexpr double aHalf{ 1 / 2.0 };
static constexpr double aSixth{ 1 / 6.0 };
static constexpr double twoThirds{ 2 * aThird };
static Eigen::Matrix4d bsplineM{ loadBSpline() };
/// Natural chooses between clamped and Natural endpoints
// with the Hermite Polynomial

static constexpr bool Natural{ false };

ASplineVec3::ASplineVec3() : mInterpolator(new ALinearInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    delete mInterpolator;
}

Eigen::Matrix4d loadBSpline()
{
	Eigen::Matrix4d m;
	m <<    aSixth, -aHalf, aHalf, -aSixth,
		 twoThirds,      0,    -1,    aHalf,
		    aSixth,  aHalf, aHalf,   -aHalf,
		         0,      0,     0,  aSixth;
	return m;
	
}
void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

    delete mInterpolator;
    switch (type)
    {
    case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
    case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
    case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break; 
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints();
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
        computeControlPoints();
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (mKeys.size() >= 2)
    {
        int totalPoints = mKeys.size();

        //If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
        //They lie on the tangent of the first and last interpolation points.
        vec3 tmp = mKeys[0].second - mKeys[1].second;
        double n = tmp.Length();
        mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

        tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
        n = tmp.Length();
        mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
    }

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
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

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size()+2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

vec3 ASplineVec3::getValue(double t)
{
    if (mCachedCurve.size() == 0) return vec3();

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    int i = rawi % mCachedCurve.size();
    double frac = t - rawi*dt;
    int inext = i + 1;
    if (!mLooping) inext = std::min<int>(inext, mCachedCurve.size() - 1);
    else inext = inext % mCachedCurve.size();

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints()
{
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0; 

	curve.clear();
	
	int numSegments = keys.size() - 1;
    for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
			//double range{ keys[segment + 1].first - keys[segment].first };
			double u = (t - keys[segment].first);

			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
            // u = 0.0 when t = keys[segment-1].first  
            // u = 1.0 when t = keys[segment].first

            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments-1, u);
		curve.push_back(val);
	}
	
    
}

vec3 lerpHelper(const vec3& left, const vec3& right, const double u) {
	return left * (1 - u) + right * u;
}
vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
   
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
    vec3 key1 = keys[segment+1].second;

    // TODO: 
	//Step 1: Create a Lerp helper function
	//Step 2: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
    
	return lerpHelper(key0, key1, u);
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	int baseIndex{ segment * 4 };
	vec3 b0{ ctrlPoints[baseIndex] };
	vec3 b1{ ctrlPoints[baseIndex + 1] };
	vec3 b2{ ctrlPoints[baseIndex + 2] };
	vec3 b3{ ctrlPoints[baseIndex + 3] };
	vec3 curveValue = b0 * std::pow(1 - t, 3) + 3 * b1 * t * (1 - t) * (1 - t) +
		b2 * 3 * t * t * (1 - t) + b3 * std::pow(t, 3);

    // TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
    
	return curveValue;

}

vec3  InterpolateCastelJau(const std::vector<vec3> invec, double t) {
	if (invec.size() == 1) {
		return invec[0];
	}
	std::vector<vec3> interpolatedValues;
	for (std::vector<vec3>::const_iterator it{ invec.begin() }; it != invec.end() - 1; ++it) {
		interpolatedValues.push_back(lerpHelper(*it, *(it + 1), t));
	}
	return InterpolateCastelJau(interpolatedValues, t);
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	ptrdiff_t baseIndex{ segment * 4 };
	std::vector<vec3> controlSet;
	std::vector<vec3>::const_iterator start{ ctrlPoints.begin() + baseIndex };
	std::copy(start, start + 4, std::back_inserter(controlSet));
	return InterpolateCastelJau(controlSet, t);
	
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	// U is the time polynomial vector
	Eigen::Vector4d U;
	U << 1.0, t, t * t, t * t * t;
	// M relates monomials to Bernstein polynomials
	Eigen::Matrix4d M;
	M << 1.0, -3.0, 3.0, -1.0,
		 0.0, 3.0, -6.0, 3.0,
		 0.0, 0.0, 3.0, -3.0,
		 0.0, 0.0, 0.0, 1.0;
	ptrdiff_t baseIndex{ segment * 4 };
	std::vector<vec3>::const_iterator 
		start{ ctrlPoints.begin() + baseIndex };
	vec3 b0{ *start++ };
	vec3 b1{ *start++ };
	vec3 b2{ *start++ };
	vec3 b3{ *start };
	// G holds the control points
	Eigen::Matrix<double, 3, 4> G;
	G << b0[0], b1[0], b2[0], b3[0],
	     b0[1], b1[1], b2[1], b3[1],
         b0[2], b1[2], b2[2], b3[2];
	Eigen::Vector3d f = G * M * U;
	vec3 curveValue(f[0], f[1], f[2]);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations

	return curveValue;
}

Eigen::Matrix<double, 3, 4> VectorColumns(const vec3& p0, const vec3& p1, const vec3& q0, 
	                                      const vec3& q1)
{
	
	Eigen::Matrix<double, 3, 4> G;
	G << p0[0], p1[0], q0[0], q1[0],
	     p0[1], p1[1], q0[1], q1[1],
         p0[2], p1[2], q0[2], q1[2];
	return G;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
    vec3 p0 = keys[segment].second;
    vec3 p1 = keys[segment + 1].second;
    vec3 q0 = ctrlPoints[segment]; // slope at p0
    vec3 q1 = ctrlPoints[segment + 1]; // slope at p1
	// U is the time polynomial vector
	Eigen::Vector4d U;
	U << 1.0, t, t * t, t * t * t;
	// M relates monomials to Hermite polynomials
	Eigen::Matrix4d M;
	M << 1.0, 0.0, -3.0, 2.0,
		 0.0, 0.0,  3.0,-2.0,
		 0.0, 1.0, -2.0, 1.0,
		 0.0, 0.0, -1.0, 1.0;
	// G holds the control points
	Eigen::Matrix<double, 3, 4> G;
	G << p0[0], p1[0], q0[0], q1[0],
	     p0[1], p1[1], q0[1], q1[1],
         p0[2], p1[2], q0[2], q1[2];
	Eigen::Vector3d f = G * M * U;
	vec3 curveValue(f[0], f[1], f[2]);

    // TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial  

    return curveValue;
}
// will compute the Bspline values in the order 
// b3, b2, b1, b0 or Nj-3, Nj-2, Nj-1, Nj
Eigen::Vector4d BSplineCoefficients(double t)
{

	Eigen::Vector4d U;
	U << 1.0, t, t * t, t * t * t;
	return bsplineM * U;
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	Eigen::Vector4d bcoeff{ BSplineCoefficients(t) };
	// for segment j (tj) the coefficients are from j -3, j -2, j -1 , j. 
	// j -3 at location j
	// the control point vector; segment 0 starts at controlpoint 0 to 4.
	Eigen::Vector3d pos{ VectorColumns(ctrlPoints[segment], ctrlPoints[segment + 1],
						   ctrlPoints[segment + 2], ctrlPoints[segment + 3])  * bcoeff};
	vec3 curveValue(pos[0], pos[1], pos[2]);
	
	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	// Step 2: compute the n nonzero Bspline Basis functions N given j
	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t


	
	return curveValue;
}
// size is the number of Data points
// size >=2.  clamped endpoint with supplied 
// clamped points to set the first derivative
Eigen::MatrixXd computeAClamped(int size) {
	Eigen::MatrixXd m(size, size);
	Eigen::RowVectorXd zeroR = Eigen::RowVectorXd::Zero( size - 1);
	m.row(0) << 1.0, zeroR;
	m.row(size - 1) << zeroR , 1.0;
	Eigen::RowVector3d middleMatrix;
	middleMatrix << 1, 4, 1;
	for (int i{ 1 }; i < size - 1; ++i) {
		zeroR = Eigen::RowVectorXd::Zero(i - 1);
		Eigen::RowVectorXd zeroR2 =
			Eigen::RowVectorXd::Zero(size - i - 2);
		m.row(i) << zeroR, middleMatrix, zeroR2;
	}
	return m;
	
//	= Eigen::MatrixXd::Zero(size, size);

}
// size is the number of Data points
// size >=2.  Natural endpoint with zero
// second derivative
Eigen::MatrixXd computeANatural(int size) {
	Eigen::MatrixXd m(size, size);
	Eigen::RowVector2d conditionRow;
	conditionRow << 2.0, 1.0;
	Eigen::RowVectorXd zeroR = Eigen::RowVectorXd::Zero( size - 2);
	m.row(0) << conditionRow, zeroR;
	conditionRow << 1.0, 2.0;
	m.row(size - 1) << zeroR , conditionRow;
	Eigen::RowVector3d middleMatrix;
	middleMatrix << 1, 4, 1;
	for (int i{ 1 }; i < size - 1; ++i) {
		zeroR = Eigen::RowVectorXd::Zero(i - 1);
		Eigen::RowVectorXd zeroR2 =
			Eigen::RowVectorXd::Zero(size - i - 2);
		m.row(i) << zeroR, middleMatrix, zeroR2;
	}
	return m;
	
//	= Eigen::MatrixXd::Zero(size, size);

}
// order is the max order of the Bslines
std::vector<double> knotVector(const std::vector<ASplineVec3::Key>& keys, int order)
{
   // up to cubic terms.
	double delta = keys[1].first - keys[0].first;
	std::vector<double> knots;
	double priortime = keys[0].first;
	for (int i{ 0 }; i < order; ++i) {
		priortime -= delta;
		knots.insert(knots.begin(), priortime);
	}
	for (const ASplineVec3::Key& key :keys) {
		knots.push_back(key.first);
	}
	delta = keys[keys.size() - 1].first - keys[keys.size() - 2].first;
	priortime = keys[keys.size() - 1].first;
	for (int i{ 0 }; i < order; ++i) {
		priortime += delta;
		knots.push_back(priortime);
	}
	return knots;
}
// 
double knots(const std::vector<double> knotVector, const int n, const int j, const double t)
{
	if (n == 0) {
		if (t >= knotVector[j] && t < knotVector[j + 1]) {
			return 1;
		}
		else {
			return 0;
		}
   }
	double delta1 = (t - knotVector[j]) / (knotVector[j + n] - knotVector[j]);
	double delta2 = (knotVector[j + n + 1] - t) / (knotVector[j + n + 1] - knotVector[j + 1]);
	double term1 = delta1 * knots(knotVector, n - 1, j, t);
	double term2 = delta2 * knots(knotVector, n - 1, j + 1, t);
	return term1 + term2;
}
double DKnots(const std::vector<double> knotVector, const int n, const int j, const double t, 
	          const int l)
{
	if (l == 0) {
		return knots(knotVector, n, j, t);
	}
	double delta1 = n / (knotVector[j + n] - knotVector[j]);
	double delta2 = n / (knotVector[j + n + 1] - knotVector[j + 1]);
	double term1 = delta1 * DKnots(knotVector, n - 1, j, t, l -1);
	double term2 = delta2 * DKnots(knotVector, n - 1, j + 1, t, l - 1);
	return term1 - term2;
}
// size is the number of Data points
// size >=2.  Natural endpoint with zero
// second derivative
Eigen::MatrixXd computeBSplineNatural(const std::vector<ASplineVec3::Key> keys) {
	// size is m + 1,  m  = size - 1; 
	// m + 3 by m + 3
	size_t m = keys.size() - 1;
	// B is the Bspline matrix
	Eigen::MatrixXd B(m + 3, m + 3);
	Eigen::RowVector4d conditionRow;
	std::vector<double> NV{ knotVector(keys, 3) };
	double n0{ DKnots(NV, 3, 0, keys[0].first, 2) };
	double n1{ DKnots(NV, 3, 1, keys[0].first, 2) };
	double n2{ DKnots(NV, 3, 2, keys[0].first, 2) };
	double n3{ DKnots(NV, 3, 3, keys[0].first, 2) };
	conditionRow << n0, n1, n2, n3;
	Eigen::RowVectorXd zeroR = Eigen::RowVectorXd::Zero(m - 1);
	B.row(0) << conditionRow, zeroR;
	n0 = DKnots(NV, 3, m -  1, keys[m].first, 2);
    n1 = DKnots(NV, 3,       m, keys[m].first, 2);
	n2 = DKnots(NV, 3,   m + 1, keys[m].first, 2);
	n3 = DKnots(NV, 3,   m + 2, keys[m].first, 2);
	conditionRow << n0, n1, n2, n3;
	B.row(m + 2) << zeroR, conditionRow;
	Eigen::RowVector4d middleMatrix;
	middleMatrix << BSplineCoefficients(0).transpose();
	for (int i{ 1 }; i < m + 1 ; ++i) {
		zeroR = Eigen::RowVectorXd::Zero(i - 1);
		Eigen::RowVectorXd zeroR2 =
			Eigen::RowVectorXd::Zero(m - i);
		B.row(i) << zeroR, middleMatrix, zeroR2;
	}
	middleMatrix  = BSplineCoefficients(1).transpose();
	B.row(m + 1) << zeroR, middleMatrix;
	return B;
}
// 1 times the difference between ctrl1 -ctrl2
Eigen::RowVector3d cntrlRow1(const vec3& ctrl1, const vec3& ctrl2)
{
	Eigen::RowVector3d row;
	row <<  (ctrl1[0] - ctrl2[0]),
		(ctrl1[1] - ctrl2[1]),
		(ctrl1[2] - ctrl2[2]);
	return row;

}
// 3 times the difference between ctrl1 -ctrl2
Eigen::RowVector3d cntrlRow(const vec3& ctrl1, const vec3& ctrl2)
{
	Eigen::RowVector3d row;
	row << 3 * (ctrl1[0] - ctrl2[0]),
		3 * (ctrl1[1] - ctrl2[1]),
		3 * (ctrl1[2] - ctrl2[2]);
	return row;

}


// compute the size * 3 matrix of D vectors one row for each ctrlPoint
// and one column for x, y, z.   computeDClamped.  Sets the slope at 
// both sides as the p0 - startPoint, endPoint - Pend.
Eigen::MatrixXd computeDClamped(const std::vector<ASplineVec3::Key>& ctrlP, 
	                            const vec3& startPoint, const vec3& endPoint)
{
	size_t size{ ctrlP.size() };
	Eigen::MatrixXd d(size, 3);
	d.row(0) << cntrlRow1(ctrlP[0].second, startPoint);
	d.row(size - 1) << cntrlRow1(endPoint, ctrlP[size - 1].second);
	for (size_t i{ 1 }; i < size - 1; ++i)
	{
		d.row(i) << cntrlRow(ctrlP[i + 1].second, ctrlP[i - 1].second);
	}
	return d;
}
// compute the size * 3 matrix of D vectors one row for each ctrlPoint

// compute the size * 3 matrix of D vectors one row for each ctrlPoint
// and one column for x, y, z.   computeDClamped.  Sets the slope at 
// both sides as the p0 - startPoint, endPoint - Pend.
Eigen::MatrixXd computeBSplinePoints(const std::vector<ASplineVec3::Key>& keys)
{
	size_t m = keys.size() - 1;
	Eigen::MatrixXd d(m + 3, 3);
	d.row(0) << 0, 0, 0;
	d.row(m + 2) << 0, 0, 0;
	for (size_t i{ 0 }; i <= m; ++i)
	{
		d.row(i + 1) << keys[i].second[0], keys[i].second[1], 
			            keys[i].second[2];
	}
	return d;
}
// and one column for x, y, z

Eigen::MatrixXd computeDNatural(const std::vector<ASplineVec3::Key>& ctrlP)
{
	size_t size{ ctrlP.size() };
	Eigen::MatrixXd d(size, 3);
	d.row(0) << cntrlRow(ctrlP[1].second, ctrlP[0].second);
	d.row(size - 1) << cntrlRow(ctrlP[size - 1].second, ctrlP[size - 2].second);
	for (size_t i{ 1 }; i < size - 1; ++i)
	{
		d.row(i) << cntrlRow(ctrlP[i + 1].second, ctrlP[i - 1].second);
	}
	return d;
}
//  get the control points;  The keys are one row per point and go in the order
// p0, p1, p2, .....pn, the matrix is one row per gradient; the output is
// p0', p1', p2', p3', p4', ....
std::vector<vec3> getControlPoints(
	Eigen::MatrixXd mat)
{
	std::vector<vec3> cntrlpoints;
	for (size_t i{ 0 }; i < mat.rows(); ++i)
	{
		cntrlpoints.push_back(vec3(mat(i, 0), mat(i, 1), mat(i, 2)));
	}
	return cntrlpoints;
}
void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;
    for (int i = 1; i < keys.size(); ++i)
    {
		vec3 b0{ keys[i - 1].second };
		vec3 b3{ keys[i].second };
		vec3 S0{ (i == 1) ? keys[i].second - keys[i - 1].second :
							(keys[i].second - keys[i - 2].second)/2 };
		vec3 S1{ (i + 1 == keys.size()) ? keys[i].second - keys[i - 1].second :
							(keys[i + 1].second - keys[i - 1].second)/2 };
		vec3 b1{ b0 + S0 * aThird };
		vec3 b2{ b3 - S1 * aThird };
        // TODO: compute b0, b1, b2, b3
        ctrlPoints.push_back(b0);
        ctrlPoints.push_back(b1);
        ctrlPoints.push_back(b2);
        ctrlPoints.push_back(b3);
    }
}
// control points are just the first derivatives at each point
// both methods of computing the control points are implemented.
// by now Clamped is running but by uncommenting and commenting,
// the natural solution would also work.
void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    int numKeys = keys.size();
	Eigen::MatrixXd m;
	Eigen::MatrixXd b;
	if (Natural) {
		m = computeANatural(keys.size());
		b = computeDNatural(keys);
	}
	else {
		m = computeAClamped(keys.size());
		b = computeDClamped(keys, startPoint, endPoint);
	}
	//std::cout << m << std::endl;
	// solve using partial pivots -- we know this is invertible;
	// because Gaussian elimination will preserve all rows
	Eigen::MatrixXd Pprime = m.partialPivLu().solve(b);
	std::cout << Pprime.rows() << std::endl;
	ctrlPoints = getControlPoints(Pprime);
	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints

}



void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;
	//testKnots();
	Eigen::MatrixXd  m = computeBSplineNatural(keys);
	//std::cout << m << std::endl;
	Eigen::MatrixXd  b = computeBSplinePoints(keys);
	Eigen::MatrixXd Pprime = m.partialPivLu().solve(b);
	std::cout << Pprime.rows();
	ctrlPoints = getControlPoints(Pprime);
    // TODO: c
    // Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C
	
    // 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)
  
	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	// Step 4: Solve AC=D for C 
	
	// Step 5: save control points in ctrlPoints
}


void testKnots() {
	std::vector<ASplineVec3::Key> keys;
	for (int i = 0; i < 7; ++i) {
		keys.push_back(std::pair<double, vec3>(i * 2, vec3(0)));
	}
	std::vector<double> nv{ knotVector(keys, 3) };
std::cout << knots(nv, 2, 4, 7) << std::endl;
std::cout << knots(nv, 3, 4, 7) << std::endl;
	std::cout << knots(nv, 3, 7, 12) << std::endl;

	Eigen::MatrixXd  m = computeBSplineNatural(keys);
	std::cout << m << std::endl;
}
