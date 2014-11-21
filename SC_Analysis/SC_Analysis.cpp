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

//Remove up to start
//Normalize frames
//Find reps
//Scale for each rep
//Check each rep

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
//Form X_Y_FUDGE
//X->person is going in; Y-> direction going
const double TOP_DOWN_FUDGE = 0.8;
const double TOP_UP_FUDGE = 0.9;
const double BOTTOM_UP_FUDGE = 1.3;
const double BOTTOM_DOWN_FUDGE = 1.15;
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

#pragma region General

#pragma region Structs (currently only for front)

struct SetupStruct{

	double footDist;
	double rLeft;
	double rRight;
	double lTheta;
	double rTheta;
	Point lFootVec;
	Point leftKnee;
	Point rightKnee;
	Point rFootVec;
	Point relRShoulder;
	SetupStruct() : footDist(0), rLeft(0), rRight(0){}//Double check this
};

struct FormErrors{
	double lthetaDiff;
	bool lwobble;
	double rthetaDiff;
	bool rwobble;
	ShoulderError shoulderBalance;
};
#pragma endregion

#pragma region Helper Functions
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

//Normalizes a frame, requiring specification of origin, next point, and direction for orthogonal vector.
void NormalizeFrame(Frame& frame, const int originInd, const int otherInd, const bool yhatNegate = false){
	//Last point is leftfoot, second to last point is right (video's left/right)
	double curDist = (frame[otherInd] - frame[originInd]).Norm();
	Point origin = Point(frame[originInd]);
	Point xhat = Point(), yhat = Point();
	NewBasis(frame[originInd], frame[otherInd], xhat, yhat, yhatNegate);
	for (size_t i = 0; i < frame.size(); ++i){
		//Scale, then subtract origin, transform into new space.
		Point newP = ((Point(frame[i])) - origin); newP.Transform(xhat, yhat);
		frame[i] = newP;
	}
}

//Remove all frames before startFrame
void RemoveUpToStart(vector<Frame>& frames, const int startFrame){
	int startFrameIndex = -1;
	for (size_t i = 0; i < frames.size(); ++i){
		if (frames[i].number == startFrame){
			startFrameIndex = i;
			break;
		}
	}
	if (startFrameIndex == -1)
		throw exception("Frame not found");
	frames.erase(frames.begin(), frames.begin() + startFrameIndex + 1);
}

void ScaleFrames(vector<Frame>& frames, const float base, const int relIndR, const int relIndL){

	for (int i = 0; i < frames.size(); ++i){
		float scale = base / (frames[i][relIndR] - frames[i][relIndL]).Norm();
		for (int j = 0; j < frames[i].size(); ++j){
			frames[i][j] *= scale;
		}
	}
}

#pragma endregion

#pragma region Execution methods

//Sets up the necessary vectors for calculation in Loop
SetupStruct Setup(const vector<Frame>& frames, const int startIndex, const double numCalPts, const VideoType videoType){
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
		//Sum up points for some period of time to get an average
		for (size_t i = startIndex; i < frames.size() && frames[i].number < frames[startIndex].number + numCalPts; ++i){
			leftFoot += frames[i][-2];
			rightFoot += frames[i][-1];
			leftKnee += frames[i][-4];
			rightKnee += frames[i][-3];
			leftShoulder += frames[i][0];
			rightShoulder += frames[i][1];
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
		ret.lTheta = atan(temp.x / temp.z);
		ret.leftKnee = leftKnee;

		temp = (FootVector(dShoulder, 2 * abs((rightFoot - shoulderMid).x), false));
		ret.rFootVec = temp.CrossProd(rightKnee).Normalize();//Pointing away from the body
		ret.rTheta = atan(temp.x / temp.z);
		ret.rightKnee = rightKnee;

		//Find relative right shoulder
		ret.relRShoulder = rightShoulder - leftShoulder;
	}
	return ret;
}

//Find first time after current frame when person is likely at botom of squat
int CriticalPointIndex(const int startFrame, const vector<Frame>& calibratedFrames, const bool bottom, const int referencePoint){
	//Person is erect, or mostly erect, at the beginning. Loop through until person is obviously not erect, then erect.
	double initialHeight = abs(calibratedFrames[startFrame][0].y);//Arbitrarily choose one of the shoulders to follow
	bool midSquat = false;
	int indA = -1, indB = -1;
	for (int i = startFrame; i < calibratedFrames.size(); ++i){
		if (bottom){
			if (abs(calibratedFrames[i][referencePoint].y) > initialHeight * BOTTOM_UP_FUDGE){
				midSquat = true;
				indA = i;
			}
			if (midSquat && abs(calibratedFrames[i][referencePoint].y) > initialHeight * BOTTOM_DOWN_FUDGE){
				indB = i;
				continue;
			}
		}
		else{
			if (abs(calibratedFrames[i][referencePoint].y) < initialHeight * TOP_DOWN_FUDGE){
				midSquat = true;
				indA = i;
			}
			if (midSquat && abs(calibratedFrames[i][referencePoint].y) < initialHeight * TOP_UP_FUDGE){
				indB = i;
				continue;
			}
		}
	}
	if (indA != -1){
		if (indB == -1)
			throw exception("Video stopped mid squat, or person is dead.");
		else{
			//Find the minimum value
			float minValue = INT_MAX;//Some large number
			int minIndex = -1;
			for (int i = indA; i < indB; ++i){
				float temp = abs(calibratedFrames[i][0].y);
				if (temp < minValue){
					minValue = temp;
					minIndex = i;
				}
			}
			if (minIndex == -1)
				throw exception("This shouldn't be executed.");
			return minIndex;
		}

	}
	else{
		if (indB == -1)
			return -1;//Flag that the video has finished.
		else
			throw exception("This should never execute.");
	}
}

#pragma endregion

#pragma endregion
/*
To Test:
NormalizeFrame
RemoveUpToStart
ScaleFrames

Setup
CriticalPointIndex

ReadPythonOut
CheckRep
*/
#pragma region Front 

//Finds the directional vector of the foot.
Point FootVector(const double dShoulder, const double dFeet, bool leftFoot){
	double sinThetaFoot = (dFeet / (2 * dShoulder));
	double cosThetaFoot = sqrt(1 - sinThetaFoot * sinThetaFoot);
	int xSign = 1;
	if (leftFoot)
		xSign = -1;
	return Point(xSign * sinThetaFoot, 0, -cosThetaFoot);
}

//Read output from python function that generates foot vectors
double ReadPythonOut(const string& line, Point& point){
	int afterComma = 0, nextComma = line.find(',');
	double ret = stod(line.substr(0, nextComma));
	afterComma = nextComma + 1;
	nextComma = line.find(afterComma, ',');
	point.x = stod(line.substr(afterComma, nextComma));
	afterComma = nextComma + 1;
	nextComma = line.find(afterComma, ',');
	point.y = stod(line.substr(afterComma, nextComma));
	afterComma = nextComma + 1;
	nextComma = line.find(afterComma, ',');
	point.z = stod(line.substr(afterComma, nextComma));
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

//Given a start and end of a rep, calculate if at any point there was shoulder tilt and if the knees followed the correct pattern, as described by values in FormErrors
FormErrors CheckRep(const int startInd, const int endInd, vector<Frame>& frames, const float shoulderThresh, const float r2Thresh, const int numCalPoints, const VideoType videoType){
	FormErrors ret;
	//Scale
	SetupStruct setupStruct = Setup(frames, startInd, numCalPoints, videoType);
	ScaleFrames(frames, setupStruct.footDist, -2, -1);
	if (videoType == FRONT){
		ret.shoulderBalance = SHOULDERS_GOOD;
		ofstream ofs("temp.csv");
		float lFootr2 = setupStruct.rLeft * setupStruct.rLeft;
		float rFootr2 = setupStruct.rRight * setupStruct.rRight;
		for (int i = startInd; i < endInd; ++i){
			ShoulderError temp = CheckShoulder(frames[i], setupStruct, shoulderThresh);
			if (temp != SHOULDERS_GOOD){
				ret.shoulderBalance = temp;
			}
			//Left foot
			Point lKnee = (frames[i][2] - frames[i][-2]);
			double z2d = lFootr2 - lKnee.Dot(lKnee);
			if (z2d < 0)
				throw exception("Impossibly long shin.");
			lKnee.z = -sqrt(z2d);//Z always negative?
			//Right foot
			Point rKnee = (frames[i][3] - frames[i][-1]);
			double z2d = rFootr2 - rKnee.Dot(lKnee);
			if (z2d < 0)
				throw exception("Impossibly long shin.");
			lKnee.z = -sqrt(z2d);//Z always negative?
			ofs << lKnee.x << ", " << lKnee.y << ", " << lKnee.z << "|" << rKnee.x << ", " << rKnee.y << ", " << rKnee.z << endl;
		}
		ofs.close();
		//Close file before writing to it again.
		system("lsq.py");
		ifstream ifs("temp.csv");
		//Read fitted plane data and use it to construct and compare to theoretical normal vector
		string input;
		getline(ifs, input);
		Point lPlane;
		ret.lwobble = ReadPythonOut(input, lPlane) > r2Thresh;//R squared
		lPlane = lPlane.CrossProd(setupStruct.leftKnee);
		getline(ifs, input);
		Point rPlane;
		ret.rwobble = ReadPythonOut(input, rPlane) > r2Thresh;//R squared
		rPlane = rPlane.CrossProd(setupStruct.rightKnee);
		//Take absolute value to force angles to be in correct quadrant.
		ret.lthetaDiff = setupStruct.lTheta- atan(abs(lPlane.x / lPlane.z));
		ret.rthetaDiff = setupStruct.rTheta - atan(abs(rPlane.x / rPlane.z));
	}
}

//Check if knees follow the path that they're supposed to follow
//DONT USE THIS!!
KneeError CheckKnee(Point calibratedKnee, const Point& plane, const double radius, const double kneeThresh){
	throw exception("Don't run this code!");
	double z2d = radius * radius - calibratedKnee.Dot(calibratedKnee);
	if (z2d < 0)
		throw exception("Impossibly long shin.");
	calibratedKnee.z = -sqrt(z2d);
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

//Given a vector of frames that start at the proper time, find all the reps and determine if they are good or not, returning a formerrors struct for each rep
vector<FormErrors> FrontAnalysis(vector<Frame>& frames, const float shoulderThresh){
	vector<FormErrors> formErrors;
	//Normalize frames
	for (int i = 0; i < frames.size(); ++i){
		NormalizeFrame(frames[i], -2, -1);
	}
	//Find all maxima and minima
	vector<int> maxima;
	vector<int> minima;
	int startFrame = 0;
	int next = 0;
	bool bottom = false;
	while ((next = CriticalPointIndex(startFrame, frames, bottom, 0)) != -1){
		if (bottom)//Start at bottom of squat
			maxima.push_back(next);
		else
			minima.push_back(next);
		//Run calculations for every rep.
		if (!bottom && minima.size() > 0){
			formErrors.push_back(CheckRep(maxima[maxima.size() - 2], maxima[maxima.size() - 1], frames));
		}
		bottom = !bottom;
	}
	return formErrors;
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
	NormalizeFrame(testFrame, -2, -1);
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