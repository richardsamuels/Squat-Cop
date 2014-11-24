#ifndef SC_HELPERS
#define SC_HELPERS

#include <vector>

#include "Point.hpp"
#include "Frame.hpp"
#include "frontstructs.hpp"

void NewBasis(const Point& left, const Point& right, Point& xhat, Point& yhat, const bool yhatNegate = false);

//Reads csv of point data. There should be a trailing comma.
std::vector<Frame> ReadFile(const string& filename);

//Normalizes a frame, requiring specification of origin, next point, and direction for orthogonal vector.
void NormalizeFrame(Frame& frame, const int originInd, const int otherInd, const bool yhatNegate = false);

//Remove all frames before startFrame
void RemoveUpToStart(std::vector<Frame>& frames, const int startFrame);


void ScaleFrames(std::vector<Frame>& frames, const float base, const int relIndR, const int relIndL);


//Sets up the necessary vectors for calculation in Loop
SetupStruct Setup(const std::vector<Frame>& frames, const int startIndex, const double numCalPts, const VideoType videoType);

//Find first time after current frame when person is likely at botom of squat
int CriticalPointIndex(const int startFrame, const std::vector<Frame>& calibratedFrames, const bool bottom, const int referencePoint);


//Read output from python function that generates foot vectors
double ReadPythonOut(const string&, Point&);
#endif SC_HELPERS