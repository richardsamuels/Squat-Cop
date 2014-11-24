#pragma once

#ifndef SC_CONSTANTS_MAIN
#define SC_CONSTANTS_MAIN

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

#endif //SC_CONSTANTS_MAIN