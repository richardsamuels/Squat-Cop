#include "front.hpp"

using namespace std;

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
			ofs << lKnee.x << ", " << lKnee.y << ", " << lKnee.z << "," << rKnee.x << ", " << rKnee.y << ", " << rKnee.z << endl;
		}
		//Close file before writing to it again.
		ofs.close();
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
		ret.lthetaDiff = setupStruct.lTheta - atan(abs(lPlane.x / lPlane.z));
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
vector<FormErrors> FrontAnalysis(vector<Frame>& frames, const double shoulderThresh, const double  r2Thresh, const int numCalPoints){
	vector<FormErrors> formErrors;
	//Normalize frames
	for (int i = 0; i < frames.size(); ++i){
		NormalizeFrame(frames[i], -2, -1);
	}//SCALE FRAMES
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
			formErrors.push_back(CheckRep(maxima[maxima.size() - 2], maxima[maxima.size() - 1], frames, shoulderThresh, r2Thresh, numCalPoints, FRONT));
		}
		bottom = !bottom;
	}
	return formErrors;
}