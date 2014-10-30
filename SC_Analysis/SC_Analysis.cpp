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

using namespace std;

enum VideoType
{
	SIDE,
	FRONT
};
static const float FRACTION_GOOD = 0.9f;

#pragma region Point and Frame classes

//Forward declarations because I hate C++.
struct Point;
Point operator+(const Point & lhs, const Point & rhs);

//Point class
struct Point{
	float x, y;

	Point(const float rectangle[]){
		x = rectangle[0] + rectangle[2] / 2;
		y = rectangle[1] + rectangle[3] / 2;
	}
	Point(float x = 0, float y = 0) : x(0), y(0){}

	float Dot(const Point& rhs){
		return x * rhs.x + y * rhs.y;
	}
	float Norm(){
		return sqrt((*this).Dot(*this));
	}

	Point & operator=(const Point &rhs){
		x = rhs.x;
		y = rhs.y;
		return *this;
	}
	Point & operator+=(const Point &rhs){
		Point p = *this + rhs;
		x = p.x;
		y = p.y;
		return *this;
	}
	Point operator / (const float value){
		return Point(x / value, y / value);
	}
	Point & operator/=(const float value){
		x /= value;
		y /= value;
		return *this;
	}
};

//Non-member point operators
Point operator+(const Point& lhs, const Point& rhs){
	return Point(lhs.x + rhs.x, lhs.y + rhs.y);
}
Point operator-(const Point& lhs, const Point& rhs){
	return Point(lhs.x - rhs.x, lhs.y - rhs.y);
}
struct LHSYLess{
	bool operator()(const Point& lhs, const Point& rhs){
		if (lhs.y == rhs.y)
			return lhs.x < rhs.x;
		else
			return lhs.y < rhs.y;
	}
}LHSLessObj;


//Frame class
struct Frame{
	vector<Point> pointList;
	int number;

	//Reads in line containing points
	Frame(int number, const string &fromFile):number(number){
		size_t start = 0, end = 0;
		short count = 0;
		float temp[2];
		while ((end = fromFile.find(',',start))!= fromFile.size() - 1){
			temp[count++] = stof(fromFile.substr(start, end));
			if (count == 2){
				pointList.push_back(Point(temp));
				count = 0;
			}
			start = end + 1;
		}
	}
	Frame(int number, int size = 2) :pointList(vector<Point>(size)), number(number){}
	template<typename Functor>
	void Organize(const Functor& f){
		sort(pointList.begin(), pointList.end(), f);
	}
};

#pragma endregion

#pragma region Front 
//Using a dot product, find the cosine of the angle of rotation of a pair of points from their "equilibrium" state.
float CosAngle(const Point& actL, const Point& actR, const Point& avgL, const Point& avgR){
	Point actC = (actL + actR) / 2, avgC = (avgL + avgR) / 2;
	Point actRPrime = actR - actC, avgRPrime = avgR - avgC;
	return (actRPrime).Dot(avgRPrime) / (actRPrime.Norm() * avgRPrime.Norm());
}

//Calculate the fractional distance difference between reference and current points.
float FracDistDiff(const Point& actL, const Point& actR, const Point& avgL, const Point& avgR){
	float actLen = (actL - actR).Norm(), avgLen = (avgL - avgR).Norm();
	return abs(actLen - avgLen) / avgLen;
}
#pragma endregion 

#pragma region Both
//Calculates the average of each point within a certain window. Can be used to calculate calibration values, and to find "smoothed" locations when person is moving(?).
//	Modification to make: Find the mean AND the variance of the points, to enable avoiding using an arbitrary threshold value.
vector<Point> AvgVals(const vector<Frame>& input, const int startFrame, const int numFrames){
	vector<Point> ret = vector<Point>();
	vector<Point> avgPoints = vector<Point>(6);
	int length = input[0].pointList.size();
	for (int i = startFrame; i < startFrame + numFrames; ++i){
		for (int j = 0; j < length; ++j){
			avgPoints[j] += input[i].pointList[j];
		}
	}
	for (int i = 0; i < length; ++i){
		avgPoints[i] /= (float)numFrames;//Cast for happy compiler.
		ret.push_back(avgPoints[i]);
	}
	return ret;
}

//Reads csv of point data. There should be a trailing comma.
vector<Frame> ReadFile(const string &filename){
	vector<Frame> dirty = vector<Frame>();
	ifstream ifs(filename);
	string line;
	int count = 0;
	while (getline(ifs, line)){
		dirty.push_back(Frame(count++, line));
	}
	ifs.close();
	return dirty;
}
//nan-ifies invalid entries and makes all frames maintain the same order of points
	//int threshFrames-max number of frames to consider when determining the initial point locations
vector<Frame> SanitizeInput(vector<Frame>& dirty, const int threshFrames, const VideoType type){
	vector<Frame> ret = vector<Frame>();
	if (type == SIDE){
		int numPts = 5;//Bar, spine-mid, hip, knee, ankle.
		int index = 0;
		//Ignore all points that don't have all 5 points.
		while (dirty[index].pointList.size() < numPts)
			++index;
		//By some magic, find the first good frame starting with index.
		while (index != dirty.size()){
			index = FindFirstGoodFrame(dirty, index, threshFrames, numPts);
			//Start adding points, add the closest point to each slot, and fill with NaN if not possible.
			index = FillAllPossilbe(dirty, ret, index, threshFrames, numPts);
		}
	}
	return ret;
}
int FindFirstGoodFrame(vector<Frame>& dirty, int index, const int threshFrames, const int numPts){
	//Assume that if a number of consecutive frames (with some error) have the correct number of frames, the points detected are the right points.
	int numGood = 0;
	//Find the mean of each point in a frame, then calculate the variance. If small, then we have our joints.
	vector<Point> meanPts = vector<Point>(numPts);
	vector<Point> varPts = vector<Point>(numPts);
	vector<Frame> useFrames = vector<Frame>();
	while (numGood < threshFrames){
		if (dirty[index].pointList.size() == numPts){
			dirty[index].Organize(LHSLessObj);
			useFrames.push_back(dirty[index]);
			for (int i = 0; i < numPts; ++i){
				meanPts[i] += dirty[index].pointList[i];
			}
		}
		++index;//At end of loop, point after last used will be index.
	}
	for (int i = 0; i < numPts; ++i){
		meanPts[i] /= numPts;
	}
	//Calculate variance. See if the variance is much more than the width of the SC Logo (save the mean widths for each point)
	//If the variance is bad, keep removing frames from the front and updating the mean/variance until the variance is small enough.
	//Once good points are found, keep adding frames, adjusting them so the points are in order and only points relatively close are considered the same. 
	//If points are lost for some period of time, rerun the code to find the points and flag the frame that there is a discontinuity since the last frame.
}

int FillAllPossilbe(vector<Frame>& dirty, vector<Frame>& ret, int index, const int threshFrames, const int numPts){
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