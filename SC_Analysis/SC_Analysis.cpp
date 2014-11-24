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
#include <cmath>
#include <tuple>

#include "Frame.hpp"
#include "helpers.hpp"

using namespace std;

//Remove up to start
//Normalize frames
//Find reps
//Scale for each rep
//Check each rep

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

#pragma region Test Code
//void TestPoint(){
//	//Test Point class
//		//Constructor
//		//Dot/Cross/Transform
//		//Overloaded operators
//	cout << "Testing Point\n";
//	Point p1 = Point();
//	Point p2 = Point(0, 1, 0);
//	Point p3 = Point(1, 0, 0);
//	bool constructorTest = (p1 == Point(0, 0, 0)) && p1 != p2;
//	cout << "\tConstructors: " << constructorTest << endl;
//	bool crossTest = (p2.CrossProd(p2) == Point()) && (p3.CrossProd(p2) == Point(0, 0, 1));
//	cout << "\tCross product: " << crossTest << endl;
//	bool dotTest = p1.Dot(p2) == 0 && p2.Dot(p2) == 1;
//	cout << "\tDot product: " << dotTest << endl;
//	Point xhat = Point(-1, -1) / sqrtf(2), yhat = Point(-1, 1) / sqrtf(2);
//	Point p4 = p3; p4.Transform(xhat, yhat);
//	bool transTest = p4 == Point(1, 1) / -sqrtf(2);
//	cout << "\tTrans: " << transTest << endl;
//}
//
//void TestFrame(){
//	cout << "Testing Frame\n";
//
//	string testString = "5,(5, 6),(3, 4),(5, 7),(1, 1)";
//	Frame testFrame(testString);
//	bool constructorTest = testFrame.number == 5 && testFrame[0] == Point(5, 6) && testFrame.size() == 4;
//	cout << "\tConstructor: " << constructorTest << endl;
//
//	testFrame.Organize(LHSLessObj);
//	bool organizeTest = testFrame[0] == Point(1, 1) && testFrame[2] == Point(5, 6);
//	cout << "\tOrganize: " << organizeTest << endl;
//
//	//This test was passed.
//	//const Frame constFrame(testString);
//	//Point test = constFrame[0];
//
//	//Test Frame class
//		//Constructor
//		//Organize
//		//Overloaded operators
//			//Test to see if the different [] operators act as expected
//
//}
//
//void TestFront(){
//	cout << "TestFront\n";
//	//Test ReadFile
//	vector<Frame> readFileVec =ReadFile("side_test.txt");
//	bool readFileTest = readFileVec[0].number == 0 && (readFileVec[0][3] - readFileVec[1][3]).x == -5;
//	cout << "\tReadFile: " << readFileTest << endl;
//	//Test NewBasis
//	Point middle, right(5, 5), left(-1, 1), xhat, yhat;
//	NewBasis(middle, right, xhat, yhat);
//	left.Transform(xhat, yhat);
//	right.Transform(xhat, yhat);
//	bool newBasisTest = right * sqrt(50) == Point(50, 0);//left * sqrt(2)== Point(0, 2) fails due to roundoff error
//	cout << "\tNewBasis: " << newBasisTest << endl;
//
//	//Test FootVector
//	double angle = 30;
//	Point footVector = FootVector(1, 1, false);
//	footVector += Point(sin(30 * 3.14159265 / 180), 0, -cos(30 * 3.14159265 / 180)) * -1; footVector.Round();
//	bool footVectorTest = footVector == Point();
//	cout << "\tFootVector : " << footVectorTest << endl;
//
//	//Test NormalizeFrame
//	string testString = "0,(0, 2),(1, 1),(2, 2)";
//	double footDist = 2;
//	Frame testFrame(testString);
//	NormalizeFrame(testFrame, -2, -1);
//	testFrame[-2].Round(); testFrame[-1].Round();
//	bool normalizeFrameTest = testFrame[-2] == Point(0, 0) && testFrame[-1] == Point(footDist, 0);
//	cout << "\tNormalizeFrame: " << normalizeFrameTest << endl;
//	
//	//Test Setup
//	testString = "0,(1, 20),(21, 20),(1, 10),(20, 10),(-2, 0),(22, 0)";
//	vector<Frame> vec;
//	vec.push_back(Frame(testString));
//	testString = "1, (1, 20), (21, 20), (1, 10), (21, 10), (-2, 0), (22, 0)";
//	vec.push_back(Frame(testString));
//	SetupStruct s = Setup(vec, 0, 2, FRONT);
//	//Inspected by eye.
//	
//	//Test CheckShoulder
//	s.relRShoulder = Point(10, 1);
//	testFrame[0] = Point(10, 2);
//	ShoulderError shoulderError = CheckShoulder(testFrame, s, 0);
//	bool checkShoulderTest = (RIGHT_HIGHER == shoulderError);
//	cout << "\tCheckShoulder: " << checkShoulderTest << endl;
//
//	//Test CheckKnee
//	Point plane(1, 0, 0), knee(-1, 2);
//	KneeError kneeError = CheckKnee(knee, plane, 6, 0);
//	bool checkKneeTest = kneeError == CAVING_IN;
//	cout << "\tCheckKnee: " << checkKneeTest << endl;
//}
#pragma endregion

int _tmain(int argc, char* argv[])
{
	//Test ReadFile
	/*TestPoint();
	TestFrame();
	TestFront();*/
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