#ifndef SC_TEST_FRAME
#define SC_TEST_FRAME

#include "catch.hpp"


#include <string>

#include "Point.hpp"
#include "Frame.hpp"

TEST_CASE("Test Frame", "[Frame]") {
	std::string testString = "5,(5, 6),(3, 4),(5, 7),(1, 1)";
	Frame testFrame(testString);


	SECTION("Constructor test") {
		
		bool constructorTest = testFrame.number == 5 && testFrame[0] == Point(5, 6) && testFrame.size() == 4;
		REQUIRE(constructorTest == true);
	}

	SECTION("Organiser test") {
		testFrame.Organize(LHSYLess());
		bool organizeTest = testFrame[0] == Point(1, 1) && testFrame[2] == Point(5, 6);
		REQUIRE(organizeTest == true);
	}
		
	//This test was passed.
	//const Frame constFrame(testString);
	//Point test = constFrame[0];

	//Test Frame class
	//Constructor
	//Organize
	//Overloaded operators
	//Test to see if the different [] operators act as expected
}

#endif //SC_TEST_FRAME