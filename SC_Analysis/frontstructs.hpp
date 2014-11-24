#ifndef SC_STRUCTS_FRONT
#define SC_STRUCTS_FRONT

#include "Point.hpp"
#include "constants.hpp"

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

#endif //SC_STRUCTS_FRONT