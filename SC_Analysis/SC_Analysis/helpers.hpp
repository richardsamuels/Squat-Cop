#ifndef SC_HELPERS
#define SC_HELPERS

#include <cstdlib>
#include <string>
#include <vector>

#include "Point.hpp"
#include "Frame.hpp"
#include "frontstructs.hpp"

void NewBasis(const Point&, const Point&, Point&, Point&, const bool = false);

//Reads csv of point data. There should be a trailing comma.
std::vector<Frame> ReadFile(const std::string&);

//Normalizes a frame, requiring specification of origin, next point, and direction for orthogonal vector.
void NormalizeFrame(Frame&, const int, const int, const bool = false);

//Remove all frames before startFrame
void RemoveUpToStart(std::vector<Frame>&, const int);


void ScaleFrames(std::vector<Frame>&, const float, const int, const int );


//Sets up the necessary vectors for calculation in Loop
SetupStruct Setup(const std::vector<Frame>&, const int, const double, const VideoType);

//Find first time after current frame when person is likely at botom of squat
int CriticalPointIndex(const int, const std::vector<Frame>&, const bool, const int);


//Read output from python function that generates foot vectors
double ReadPythonOut(const std::string&, Point&);
#endif SC_HELPERS