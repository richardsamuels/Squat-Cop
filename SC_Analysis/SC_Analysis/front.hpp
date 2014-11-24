#ifndef SC_FRONT
#define SC_FRONT

#include <fstream>

#include "constants.hpp"
#include "frontstructs.hpp"
#include "helpers.hpp"
#include "Frame.hpp"
#include "Point.hpp"

using namespace std;

//Finds the directional vector of the foot.
Point FootVector(const double, const double, bool);


//Checks shoulder for tilt.
ShoulderError CheckShoulder(const Frame&, const SetupStruct&, const double);

//Given a start and end of a rep, calculate if at any point there was shoulder tilt and if the knees followed the correct pattern, as described by values in FormErrors
FormErrors CheckRep(const int, const int, vector<Frame>&, const float, const float, const int, const VideoType);

//Check if knees follow the path that they're supposed to follow
//DONT USE THIS!!
KneeError CheckKnee(Point, const Point&, const double, const double );

//Given a vector of frames that start at the proper time, find all the reps and determine if they are good or not, returning a formerrors struct for each rep
vector<FormErrors> FrontAnalysis(vector<Frame>&, const double, const double, const int);

#endif