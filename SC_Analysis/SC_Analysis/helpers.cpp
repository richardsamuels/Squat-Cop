#include "helpers.hpp"
#include "constants.hpp"
#include "frontstructs.hpp"
#include "front.hpp"
#include "Point.hpp"

using namespace std;

//Constructs two new basis vectors, where points from left-right foot, and the other points orthogonal to it, predominatly in the y direction.
void NewBasis(const Point& left, const Point& right, Point& xhat, Point& yhat, const bool yhatNegate){
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
void NormalizeFrame(Frame& frame, const int originInd, const int otherInd, const bool yhatNegate){
	//Last point is leftfoot, second to last point is right (video's left/right)
	double curDist = (frame[otherInd] - frame[originInd]).Norm();
	Point origin = Point(frame[originInd]);
	Point xhat = Point(), yhat = Point();
	NewBasis(frame[originInd], frame[otherInd], xhat, yhat, yhatNegate);
	for (size_t i = 0; i < frame.size(); ++i){
		//Subtract origin and transform into new space.
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

	for (size_t i = 0; i < frames.size(); ++i){
		double scale = base / (frames[i][relIndR] - frames[i][relIndL]).Norm();
		for (size_t j = 0; j < frames[i].size(); ++j){
			frames[i][j] *= scale;
		}
	}
}


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
	for (size_t i = startFrame; i < calibratedFrames.size(); ++i){
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
			float minValue = FLT_MAX;//Some large number
			int minIndex = -1;
			for (int i = indA; i < indB; ++i){
				double temp = abs(calibratedFrames[i][0].y);
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
	return ret;
}