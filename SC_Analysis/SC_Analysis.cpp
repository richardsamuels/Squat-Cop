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
static const float FRACTION_GOOD = 0.9f;

#pragma region Point and Frame classes

//Forward declarations because I hate C++.
struct Point;
Point operator+(const Point & lhs, const Point & rhs);

//Point class
struct Point{
	float x, y, z;

	Point(const float rectangle[]){
		x = rectangle[0] + rectangle[2] / 2;
		y = rectangle[1] + rectangle[3] / 2;
		z = 0;
	}
	Point(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z){}

	float Dot(const Point& rhs) const{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}
	float Norm() const{
		return sqrt((*this).Dot(*this));
	}

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
	Point operator / (const float value){
		return Point(x / value, y / value, z / value);
	}
	Point operator * (const float value){
		return Point(x * value, y * value, z * value);
	}
	Point & operator/=(const float value){
		x /= value;
		y /= value;
		z /= value;
		return *this;
	}
	Point & Transform(const Point& xhat, const Point& yhat){
		//Fill this
		x = (*this).Dot(xhat);
		y = (*this).Dot(yhat);
		return *this;
	}
	Point CrossProd(const Point& rhs){
		float xVal, yVal, zVal;
		xVal = y * rhs.z - (z * rhs.y);
		yVal = -(x * rhs.z - (z * rhs.x));//Because determinant goes like 1 -1 1...
		zVal = x * rhs.y - (y * rhs.z);
		return Point(xVal, yVal, zVal);
	}
};

//Non-member point operators
Point operator+(const Point& lhs, const Point& rhs){
	return Point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
Point operator-(const Point& lhs, const Point& rhs){
	return Point(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}
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


//Frame class
struct Frame{
	vector<Point> pointList;
	int number;

	//Reads in line containing points
	Frame(const string &fromFile){
		int middleComma, lparen, rparen, start = 0;
		number = stof(fromFile.substr(0, fromFile.find(',', 0)));
		while ((lparen = fromFile.find('(',start))!= string::npos){
			rparen = fromFile.find(',', start);
			middleComma = fromFile.find(',', lparen);
			pointList.push_back(Point(stof(fromFile.substr(lparen + 1, middleComma - (lparen + 1))), stof(fromFile.substr(middleComma + 1, rparen - (middleComma + 1)))));
			start = rparen + 1;
		}
	}
	Frame(int number, int size = 2) :pointList(vector<Point>(size)), number(number){}
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
};

#pragma endregion

struct SetupStruct{
	//void Setup(const vector<Frame>& dirty, const int startFrame, const int numCalPts,
	//	float& footDist, float& rLeft, float& rRight, Point& lFootVec, Point& rFootVec)
	float footDist;
	float  rLeft;
	float rRight;
	Point lFootVec;
	Point rFootVec;
	Point relRShoulder;
	SetupStruct(): footDist(0), rLeft(0), rRight(0){}//Double check this
};
struct FormErrors{
	KneeError leftKnee;
	KneeError rightKnee;
	float shoulderBalance;
};

#pragma region Front 
//Setup:
//Save average distance ( few points ) for each leg.
//These points are radii for the sphere
//Using average shoulder distance, calculate the foot angles
//Find CrossProd of vector from foot to knee (foot normalized to origin) and outgoing angles along z/x axis. z component should be positive for both feet/
//	x should be positive for right and not for left (double check this)
//Loop:
//Transform points (foot transform)
//For both foot:
//z = sqrt(r^2 - (x^ + y^2))
//Take DP of [x y z] vector and vector perp to plane of circle.
//Calculate the magnitude of this vector, then subtract this from the whole mag.
//Flag bad form if z distances are not the same within some threshold
//Flag bad form if |magnitude in plane of circle - r| < some threshold 

//Sets up the necessary vectors for calculation in FontLoop
SetupStruct Setup(vector<Frame>& dirty, const int startFrame, const int numCalPts){
	SetupStruct ret;
	ret.rLeft = 0;
	ret.rRight = 0;
	Point leftKnee = Point();
	Point rightKnee = Point();
	Point xhat = Point();
	Point yhat = Point();
	Point leftFoot = Point();
	Point rightFoot = Point();
	Point leftShoulder = Point();
	Point rightShoulder = Point();
	float dShoulder = 0;
	//Sum up points for some period of time to get an average
	for (int i = startFrame; i < startFrame + numCalPts; ++i){
		leftFoot += dirty[i][-2];
		rightFoot += dirty[i][-1];
		leftKnee += (dirty[i][-4]);
		rightKnee += (dirty[i][-3]);
		leftShoulder += dirty[i][0];
		rightShoulder += dirty[i][1];
	}
	leftFoot /= numCalPts;
	rightFoot /= numCalPts;
	leftKnee /= numCalPts;
	rightKnee /= numCalPts;
	dShoulder = ((leftShoulder - rightShoulder) / numCalPts).Norm;
	//Find radii and foot width
	ret.rLeft = (leftFoot - leftKnee).Norm();
	ret.rRight = (rightFoot - rightKnee).Norm();
	ret.footDist = (leftFoot - rightFoot).Norm();
	
	//Calculate new basis vectors
	Point xhat = Point();
	Point yhat = Point();
	NewBasis(leftFoot, rightFoot, xhat, yhat);
	
	//Transform points so  that the left foot is the origin. Translate all points, then rotate.
	leftKnee = (leftKnee - leftFoot).Transform(xhat, yhat);
	rightKnee = (rightKnee - leftFoot).Transform(xhat, yhat);
	rightFoot = (rightFoot - leftFoot).Transform(xhat, yhat);
	leftShoulder = (leftShoulder - leftFoot).Transform(xhat, yhat);
	rightShoulder = (rightShoulder - leftFoot).Transform(xhat, yhat);
	leftFoot = Point(0 , 0);
	
	//Transform right knee so its origin is at rightfoot.
	rightKnee = rightKnee - rightFoot;
	//Find vector to take CrossProd with, take CrossProds, and return these. These vectors should point AWAY from the body.
	ret.lFootVec = FootVector(dShoulder, ret.footDist, true).CrossProd(leftKnee);
	ret.rFootVec = FootVector(dShoulder, ret.footDist, false).CrossProd(rightKnee);
	
	//Find relative right shoulder
	ret.relRShoulder = leftShoulder - rightShoulder;
	return ret;
}

//Finds the directional vector of the foot.
Point FootVector(const float dShoulder, const float dFeet, bool leftFoot){
	float sinThetaFoot = (dFeet / (2 * dShoulder));
	float cosThetaFoot = sqrt(1 - sinThetaFoot * sinThetaFoot);
	int xSign = -1 * leftFoot;
	return Point(xSign * sinThetaFoot, 0, -cosThetaFoot);
}

//Constructs two new basis vectors, where points from left-right foot, and the other points orthogonal to it, predominatly in the y direction.
void NewBasis(const Point& leftFoot, const Point& rightFoot, Point& xhat, Point& yhat){
	Point x = rightFoot - leftFoot;
	xhat = x / x.Norm();
	yhat.x = -xhat.y;
	yhat.y = xhat.x;
}

//Normalizes a frame, making the origin the left foot, and making the right foot at (0, dFoot)
void NormalizeFrame(Frame& frame, const float dFoot){
	//Last point is leftfoot, second to last point is right (video's left/right)
	float curDist = (frame[-1]- frame[-2]).Norm();
	float scaleFactor = dFoot / curDist;//If curdist < originalDist, the points recorded are too close, so make them further apart.
	//Frame ret = Frame(frame);//Check this copy constructor works right.
	Point origin = Point(frame[-2]) * scaleFactor;
	Point xhat = Point(), yhat = Point();
	NewBasis(frame[-2], frame[-1], xhat, yhat);
	for (int i = 0; i < frame.pointList.size(); ++i){
		//Scale, then subtract origin, transform into new space.
		Point newP = ((Point(frame[i]) *  scaleFactor) - origin).Transform(xhat, yhat);
		frame[i] = newP;
	}
}

//Checks shoulder for tilt.
ShoulderError CheckShoulder(Frame& normalizedFrame, const SetupStruct& s, const float thresh){
	Point relRCurrent = normalizedFrame[1] - normalizedFrame[0];
	float sinTheta = (relRCurrent.CrossProd(s.relRShoulder) / (relRCurrent.Norm() * s.relRShoulder.Norm())).z;
	if (abs(sinTheta) > thresh){
		if (sinTheta < 0)
			return LEFT_HIGHER;
		else
			return RIGHT_HIGHER;
	}
	else
		return SHOULDERS_GOOD;

}

//Check if knees follow the path that they're supposed to follow
KneeError CheckKnee(const Point& calibratedKnee, const Point& plane, const float radius, const float thresh){
	Point fullPoint = Point(calibratedKnee);
	fullPoint.z = sqrt(radius * radius - fullPoint.Dot(fullPoint));
	float perpVal = fullPoint.Dot(plane);
	if (abs(perpVal) > thresh)
	{
		if (perpVal < 0)
			return CAVING_IN;
		else
			return POINTING_OUT;
	}
	else
		return KNEE_GOOD;
}

//Execute this function each cycle.
FormErrors& FrontLoop(Frame& frame, const SetupStruct s, const float thresh){
	FormErrors ret;
	//Frame now has left foot at (0, 0) and right foot at (dFoot, 0)
	NormalizeFrame(frame, s.footDist);
	ret.leftKnee = CheckKnee(frame[-4], s.lFootVec, s.rLeft, thresh);
	ret.rightKnee = CheckKnee(frame[-3] - frame[-1], s.rFootVec, s.rRight, thresh);
	ret.shoulderBalance = CheckShoulder(frame, s);
	return ret;
}

#pragma endregion 

#pragma region Both
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

int _tmain(int argc, char* argv[])
{
	vector<Frame> side, front;
	side = ReadFile("side_test.txt");
	for (int i = 0; i < argc - 1; ++i){
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