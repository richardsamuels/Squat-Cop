// SC_Analysis.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <list>
#include <algorithm>
#include <math.h>
#include <tuple>

using namespace std;

#pragma region Enums and constants
enum VideoType
{
	SIDE,
	FRONT
};
enum KneeError
{
	CAVING_IN,
	POINTING_OUT,
	KNEE_GOOD
};
enum ShoulderError{
	LEFT_HIGHER,
	RIGHT_HIGHER,
	SHOULDERS_GOOD
};
static const double FRACTION_GOOD = 0.9;
static const double RADIUS_FUDGE = 1.05;
static const double KNEE_THRESH = 0.1;
static const double SHOULDER_THRESH = 0.1;
#pragma endregion

#pragma region Point and Frame classes

#pragma region Point
//Forward declarations because I hate C++.
struct Point;
Point operator+(const Point & lhs, const Point & rhs);
//Point class
struct Point{
	double x, y, z;

	//Constructors
	Point(const double rectangle[]){
		x = rectangle[0] + rectangle[2] / 2;
		y = rectangle[1] + rectangle[3] / 2;
		z = 0;
	}
	Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z){}

	//Overloaded operators
	Point & operator=(const Point &rhs){
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}
	Point & operator+=(const Point &rhs){
		Point p = *this + rhs;
		x = p.x;
		y = p.y;
		z = p.z;
		return *this;
	}
	Point operator / (const double value){
		return Point(x / value, y / value, z / value);
	}
	Point operator * (const double value){
		return Point(x * value, y * value, z * value);
	}
	Point & operator *=(const double value){
		x *= value;
		y *= value;
		z *= value;
		return *this;
	}
	Point & operator/=(const double value){
		x /= value;
		y /= value;
		z /= value;
		return *this;
	}
	bool operator ==(const Point& rhs) const{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}
	bool operator !=(const Point& rhs) const{
		return !(*this == rhs);
	}
	friend ostream& operator<<(ostream& os, const Point& p);
	//Main operators
	double Dot(const Point& rhs) const{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}
	double Norm() const{
		return sqrt((*this).Dot(*this));
	}
	Point& Normalize(){
		*this /= Norm();
		return *this;
	}
	void Transform(const Point& xhat, const Point& yhat){
		double xnew = (*this).Dot(xhat);
		double ynew = (*this).Dot(yhat);
		x = xnew; y = ynew;
		//return *this;
	}
	Point CrossProd(const Point& rhs){
		double xVal, yVal, zVal;
		xVal = y * rhs.z - (z * rhs.y);
		yVal = -(x * rhs.z - (z * rhs.x));//Because determinant goes like 1 -1 1...
		zVal = x * rhs.y - (y * rhs.z);
		return Point(xVal, yVal, zVal);
	}
	void Round(){
		x = round(x * 1000) / 1000;
		y = round(y * 1000) / 1000;
		z = round(z * 1000) / 1000;
	}

};

//Non-member point operators
Point operator+(const Point& lhs, const Point& rhs){
	return Point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
Point operator-(const Point& lhs, const Point& rhs){
	return Point(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}
ostream& operator<<(ostream& os, const Point& p){
	if (p.z ==0)
		os << "(" << p.x << ", " << p.y << ")";
	else
		os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
	return os;
}
//Comparison functor
struct LHSYLess{
	bool operator()(const Point& lhs, const Point& rhs){
		if (lhs.z == rhs.z){
			if (lhs.y == rhs.y)
				return lhs.x < rhs.x;
			else
				return lhs.y < rhs.y;
		}
		else
			return lhs.z < rhs.z;
	}
}LHSLessObj;
#pragma endregion 

//A copy of frame contains the SAME data.
struct Frame{
public:
	vector<Point> pointList;
	int number;

	//Reads in line containing points
	Frame(const string &fromFile){
		int middleComma, lparen, rparen, start = 0;
		number = stoi(fromFile.substr(0, fromFile.find(',', 0)));
		while ((lparen = fromFile.find('(',start))!= string::npos){
			rparen = fromFile.find(')', start);
			middleComma = fromFile.find(',', lparen);
			pointList.push_back(Point(stof(fromFile.substr(lparen + 1, middleComma - (lparen + 1))), stof(fromFile.substr(middleComma + 1, rparen - (middleComma + 1)))));
			start = rparen + 1;
		}
	}

	template<typename Functor>
	void Organize(const Functor& f){
		sort(pointList.begin(), pointList.end(), f);
	}

	Point& operator[](int i) {
		if (i < 0){
			i += pointList.size();
		}
		return pointList[i];
	}

	const Point& operator[](int i) const {
		if (i < 0){
			i += pointList.size();
		}
		return pointList[i];
	}

	size_t size(){
		return pointList.size();
	}

	void push_back(const Point& p){
		pointList.push_back(p);
	}
};

#pragma endregion

#pragma region General helper functions

//Constructs two new basis vectors, where points from left-right foot, and the other points orthogonal to it, predominatly in the y direction.
void NewBasis(const Point& left, const Point& right, Point& xhat, Point& yhat, const bool yhatNegate = false){
	Point x = right - left;
	xhat = x / x.Norm();
	yhat.x = -xhat.y;
	yhat.y = xhat.x;
	if (yhatNegate)
		yhat *= -1;
}

//Reads csv of point data. There should be a trailing comma.
vector<Frame> ReadFile(const string &filename){
	vector<Frame> dirty = vector<Frame>();
	ifstream ifs(filename);
	string line;
	int count = 0;
	while (getline(ifs, line)){
		dirty.push_back(Frame(line));
	}
	ifs.close();
	return dirty;
}

#pragma endregion

#pragma region General structs (use inheritance to generalize later)
struct SetupStruct{
	//void Setup(const vector<Frame>& dirty, const int startFrame, const int numCalPts,
	//	double& footDist, double& rLeft, double& rRight, Point& lFootVec, Point& rFootVec)
	double footDist;
	double rLeft;
	double rRight;
	Point lFootVec;
	Point rFootVec;
	Point relRShoulder;
	SetupStruct() : footDist(0), rLeft(0), rRight(0){}//Double check this
};

struct FormErrors{
	KneeError leftKnee;
	KneeError rightKnee;
	ShoulderError shoulderBalance;
};
#pragma endregion

#pragma region Front analysis
//Finds the directional vector of the foot.
Point FootVector(const double dShoulder, const double dFeet, bool leftFoot){
	double sinThetaFoot = (dFeet / (2 * dShoulder));
	double cosThetaFoot = sqrt(1 - sinThetaFoot * sinThetaFoot);
	int xSign = 1;
	if (leftFoot)
		xSign = -1;
	return Point(xSign * sinThetaFoot, 0, -cosThetaFoot);
}

//Checks shoulder for tilt.
ShoulderError CheckShoulder(const Frame& normalizedFrame, const SetupStruct& s, const double shoulderThresh){
	Point relRCurrent = normalizedFrame[1] - normalizedFrame[0];//Right - left
	double sinTheta = (relRCurrent.CrossProd(s.relRShoulder) / (relRCurrent.Norm() * s.relRShoulder.Norm())).z;
	if (abs(sinTheta) > shoulderThresh){
		if (sinTheta > 0)
			return LEFT_HIGHER;
		else
			return RIGHT_HIGHER;
	}
	else
		return SHOULDERS_GOOD;

}

//Check if knees follow the path that they're supposed to follow
KneeError CheckKnee(Point calibratedKnee, const Point& plane, const double radius, const double kneeThresh){
	double z2d = radius * radius - calibratedKnee.Dot(calibratedKnee);
	if (z2d < 0)
		throw exception("Impossibly long shin.");
	calibratedKnee.z = sqrt(z2d);
	double perpVal = calibratedKnee.Dot(plane) / radius;
	//If the projection onto the perpendicular component (which points away from the body) is negative, the knees are pointing in. Otherwise, they're pointing out.
	if (abs(perpVal) > kneeThresh)
	{
		if (perpVal < 0)
			return CAVING_IN;
		else
			return POINTING_OUT;
	}
	else
		return KNEE_GOOD;
}
#pragma endregion 

#pragma region General execution methods

//Sets up the necessary vectors for calculation in Loop
SetupStruct Setup(const vector<Frame>& dirty, const int startFrame, const double numCalPts, const VideoType videoType){
	//Initialize data for use
	SetupStruct ret;
	if (videoType == FRONT){
		ret.rLeft = 0;
		ret.rRight = 0;
		Point leftKnee = Point();
		Point rightKnee = Point();
		Point leftFoot = Point();
		Point rightFoot = Point();
		Point leftShoulder = Point();
		Point rightShoulder = Point();
		int startFrameIndex = -1;
		for (size_t i = 0; i < dirty.size(); ++i){
			if (dirty[i].number == startFrame){
				startFrameIndex = i;
				break;
			}
		}
		if (startFrameIndex == -1)
			throw exception("Frame not found");
		//Sum up points for some period of time to get an average
		for (size_t i = startFrameIndex; i < dirty.size() && dirty[i].number < startFrame + numCalPts; ++i){
			leftFoot += dirty[i][-2];
			rightFoot += dirty[i][-1];
			leftKnee += dirty[i][-4];
			rightKnee += dirty[i][-3];
			leftShoulder += dirty[i][0];
			rightShoulder += dirty[i][1];
		}
		leftFoot /= numCalPts;
		rightFoot /= numCalPts;
		leftKnee /= numCalPts;
		rightKnee /= numCalPts;
		leftShoulder /= numCalPts;
		rightShoulder /= numCalPts;

		//Shoulder distance
		double dShoulder = 0;
		dShoulder = (leftShoulder - rightShoulder).Norm();

		//Find radii and foot width
		ret.rLeft = (leftFoot - leftKnee).Norm() * RADIUS_FUDGE;
		ret.rRight = (rightFoot - rightKnee).Norm() * RADIUS_FUDGE;
		ret.footDist = (leftFoot - rightFoot).Norm();

		//Calculate new basis vectors
		Point xhat = Point();
		Point yhat = Point();
		NewBasis(leftFoot, rightFoot, xhat, yhat);

		//Transform points so  that the left foot is the origin. Translate all points, then rotate.
		leftKnee = (leftKnee - leftFoot); leftKnee.Transform(xhat, yhat);
		rightKnee = (rightKnee - leftFoot); rightKnee.Transform(xhat, yhat);
		rightFoot = (rightFoot - leftFoot); rightFoot.Transform(xhat, yhat);
		leftShoulder = (leftShoulder - leftFoot); leftShoulder.Transform(xhat, yhat);
		rightShoulder = (rightShoulder - leftFoot); rightShoulder.Transform(xhat, yhat);
		leftFoot = Point(0, 0);

		//Transform right knee so its origin is at rightfoot.
		rightKnee = rightKnee - rightFoot;

		//Find vector to take CrossProd with, take CrossProds, and return these. These vectors should point AWAY from the body.
		Point shoulderMid = (leftShoulder + rightShoulder) / 2;//This helps to account for uneven feet/markers.
		Point temp = (FootVector(dShoulder, 2 * abs((leftFoot - shoulderMid).x), true)); // * 2 to assume that the feet are evenly spaced, even if they aren't.
		ret.lFootVec = leftKnee.CrossProd(temp).Normalize();//Pointing away from the body
		temp = (FootVector(dShoulder, 2 * abs((rightFoot - shoulderMid).x), false));
		ret.rFootVec = temp.CrossProd(rightKnee).Normalize();//Pointing away from the body

		//Find relative right shoulder
		ret.relRShoulder = rightShoulder - leftShoulder;
	}
	return ret;
}

//Normalizes a frame, requiring specification of origin, next point, and direction for orthogonal vector.
void NormalizeFrame(Frame& frame, const double footDist, const int originInd, const int otherInd, const bool yhatNegate = false){
	//Last point is leftfoot, second to last point is right (video's left/right)
	double curDist = (frame[otherInd]- frame[originInd]).Norm();
	double scaleFactor = footDist / curDist;//If curdist < originalDist, the points recorded are too close, so make them further apart.
	//Frame ret = Frame(frame);//Check this copy constructor works right.
	Point origin = Point(frame[originInd]) * scaleFactor;
	Point xhat = Point(), yhat = Point();
	NewBasis(frame[originInd], frame[otherInd], xhat, yhat, yhatNegate);
	for (size_t i = 0; i < frame.size(); ++i){
		//Scale, then subtract origin, transform into new space.
		Point newP = ((Point(frame[i]) *  scaleFactor) - origin); newP.Transform(xhat, yhat);
		frame[i] = newP;
	}
}

//Execute this function each cycle.
FormErrors Loop(Frame& frame, const SetupStruct s, const double thresh, const VideoType videoType){
	FormErrors ret;
	if (videoType == FRONT){
		//Frame now has left foot at (0, 0) and right foot at (footDist, 0)
		NormalizeFrame(frame, s.footDist, -2, -1);
		ret.leftKnee = CheckKnee(frame[-4], s.lFootVec, s.rLeft, thresh);
		ret.rightKnee = CheckKnee(frame[-3] - frame[-1], s.rFootVec, s.rRight, thresh);
		ret.shoulderBalance = CheckShoulder(frame, s, thresh);

	}
	return ret;
}

#pragma endregion

#pragma region Test Code
void TestPoint(){
	//Test Point class
		//Constructor
		//Dot/Cross/Transform
		//Overloaded operators
	cout << "Testing Point\n";
	Point p1 = Point();
	Point p2 = Point(0, 1, 0);
	Point p3 = Point(1, 0, 0);
	bool constructorTest = (p1 == Point(0, 0, 0)) && p1 != p2;
	cout << "\tConstructors: " << constructorTest << endl;
	bool crossTest = (p2.CrossProd(p2) == Point()) && (p3.CrossProd(p2) == Point(0, 0, 1));
	cout << "\tCross product: " << crossTest << endl;
	bool dotTest = p1.Dot(p2) == 0 && p2.Dot(p2) == 1;
	cout << "\tDot product: " << dotTest << endl;
	Point xhat = Point(-1, -1) / sqrtf(2), yhat = Point(-1, 1) / sqrtf(2);
	Point p4 = p3; p4.Transform(xhat, yhat);
	bool transTest = p4 == Point(1, 1) / -sqrtf(2);
	cout << "\tTrans: " << transTest << endl;
}

void TestFrame(){
	cout << "Testing Frame\n";

	string testString = "5,(5, 6),(3, 4),(5, 7),(1, 1)";
	Frame testFrame(testString);
	bool constructorTest = testFrame.number == 5 && testFrame[0] == Point(5, 6) && testFrame.size() == 4;
	cout << "\tConstructor: " << constructorTest << endl;

	testFrame.Organize(LHSLessObj);
	bool organizeTest = testFrame[0] == Point(1, 1) && testFrame[2] == Point(5, 6);
	cout << "\tOrganize: " << organizeTest << endl;

	//This test was passed.
	//const Frame constFrame(testString);
	//Point test = constFrame[0];

	//Test Frame class
		//Constructor
		//Organize
		//Overloaded operators
			//Test to see if the different [] operators act as expected

}

void TestFront(){
	cout << "TestFront\n";
	//Test ReadFile
	vector<Frame> readFileVec =ReadFile("side_test.txt");
	bool readFileTest = readFileVec[0].number == 0 && (readFileVec[0][3] - readFileVec[1][3]).x == -5;
	cout << "\tReadFile: " << readFileTest << endl;
	//Test NewBasis
	Point middle, right(5, 5), left(-1, 1), xhat, yhat;
	NewBasis(middle, right, xhat, yhat);
	left.Transform(xhat, yhat);
	right.Transform(xhat, yhat);
	bool newBasisTest = right * sqrt(50) == Point(50, 0);//left * sqrt(2)== Point(0, 2) fails due to roundoff error
	cout << "\tNewBasis: " << newBasisTest << endl;

	//Test FootVector
	double angle = 30;
	Point footVector = FootVector(1, 1, false);
	footVector += Point(sin(30 * 3.14159265 / 180), 0, -cos(30 * 3.14159265 / 180)) * -1; footVector.Round();
	bool footVectorTest = footVector == Point();
	cout << "\tFootVector : " << footVectorTest << endl;

	//Test NormalizeFrame
	string testString = "0,(0, 2),(1, 1),(2, 2)";
	double footDist = 2;
	Frame testFrame(testString);
	NormalizeFrame(testFrame, footDist, -2, -1);
	testFrame[-2].Round(); testFrame[-1].Round();
	bool normalizeFrameTest = testFrame[-2] == Point(0, 0) && testFrame[-1] == Point(footDist, 0);
	cout << "\tNormalizeFrame: " << normalizeFrameTest << endl;
	
	//Test Setup
	testString = "0,(1, 20),(21, 20),(1, 10),(20, 10),(-2, 0),(22, 0)";
	vector<Frame> vec;
	vec.push_back(Frame(testString));
	testString = "1, (1, 20), (21, 20), (1, 10), (21, 10), (-2, 0), (22, 0)";
	vec.push_back(Frame(testString));
	SetupStruct s = Setup(vec, 0, 2, FRONT);
	//Inspected by eye.
	
	//Test CheckShoulder
	s.relRShoulder = Point(10, 1);
	testFrame[0] = Point(10, 2);
	ShoulderError shoulderError = CheckShoulder(testFrame, s, 0);
	bool checkShoulderTest = (RIGHT_HIGHER == shoulderError);
	cout << "\tCheckShoulder: " << checkShoulderTest << endl;

	//Test CheckKnee
	Point plane(1, 0, 0), knee(-1, 2);
	KneeError kneeError = CheckKnee(knee, plane, 6, 0);
	bool checkKneeTest = kneeError == CAVING_IN;
	cout << "\tCheckKnee: " << checkKneeTest << endl;
}
#pragma endregion

int _tmain(int argc, char* argv[])
{
	//Test ReadFile
	TestPoint();
	TestFrame();
	TestFront();
	vector<Frame> side, front;
	for (int i = 0; i < -argc - 1; ++i){
		if (argv[i] == "-side"){
			side = ReadFile(argv[i + 1]);
		}
		else if (argv[i] == "-front"){
			front = ReadFile(argv[i + 1]);
		}
	}
	cout << "Finished. Press ENTER to continue." << flush;
	cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
	return 0;
}