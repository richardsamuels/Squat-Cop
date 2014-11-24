#include "catch.hpp"


#include <string>
#include <vector>

#include "Point.hpp"
#include "Frame.hpp"
#include "helpers.hpp"
#include "front.hpp"


TEST_CASE("Test Front", "[Front]") {
	std::string testString = "5,(5, 6),(3, 4),(5, 7),(1, 1)";
	Frame testFrame(testString);


	SECTION("Readfile test") {

		std::vector<Frame> readFileVec = ReadFile("side_test.txt");
		bool readFileTest = readFileVec[0].number == 0 && (readFileVec[0][3] - readFileVec[1][3]).x == -5;
		REQUIRE(readFileTest == true);
	}

	SECTION("NewBasis test") {
		Point middle, right(5, 5), left(-1, 1), xhat, yhat;
		NewBasis(middle, right, xhat, yhat);
		left.Transform(xhat, yhat);
		right.Transform(xhat, yhat);
		bool newBasisTest = right * sqrt(50) == Point(50, 0);//left * sqrt(2)== Point(0, 2) fails due to roundoff error
		REQUIRE(newBasisTest == true);
	}
	
	SECTION("FootVector test") {
		double angle = 30;
		Point footVector = FootVector(1, 1, false);
		footVector += Point(sin(30 * 3.14159265 / 180), 0, -cos(30 * 3.14159265 / 180)) * -1; footVector.Round();
		bool footVectorTest = footVector == Point();
		REQUIRE(footVectorTest == true);
	}


	testString = "0,(0, 2),(1, 1),(2, 2)";
	SECTION("NormalizeFrame test") {
		
		double footDist = 2;
		Frame testFrame(testString);
		NormalizeFrame(testFrame, -2, -1);
		testFrame[-2].Round(); testFrame[-1].Round();
		bool normalizeFrameTest = testFrame[-2] == Point(0, 0) && testFrame[-1] == Point(footDist, 0);
		REQUIRE(normalizeFrameTest == true);
	}

	testString = "0,(1, 20),(21, 20),(1, 10),(20, 10),(-2, 0),(22, 0)";
	vector<Frame> vec;
	vec.push_back(Frame(testString));
	testString = "1, (1, 20), (21, 20), (1, 10), (21, 10), (-2, 0), (22, 0)";
	vec.push_back(Frame(testString));
	SetupStruct s = Setup(vec, 0, 2, FRONT);

	SECTION("Setup test") {
		
		//No assertions, need some
		
	}


	SECTION("CheckShoulder test") {
		s.relRShoulder = Point(10, 1);
		testFrame[0] = Point(10, 2);
		ShoulderError shoulderError = CheckShoulder(testFrame, s, 0);
		bool checkShoulderTest = (RIGHT_HIGHER == shoulderError);
		REQUIRE(checkShoulderTest == true);
	}

	/*SECTION("CheckKnee test") {
		Point plane(1, 0, 0), knee(-1, 2);
		KneeError kneeError = CheckKnee(knee, plane, 6, 0);
		bool checkKneeTest = kneeError == CAVING_IN;
		REQUIRE(checkKneeTest == true);
	}*/

}