#include "catch.hpp"
#include "Point.hpp"

TEST_CASE("Test Point", "[Point]") {
	Point p1 = Point();
	Point p2 = Point(0, 1, 0);
	Point p3 = Point(1, 0, 0);

	SECTION("Constructor test") {
		bool constructorTest = (p1 == Point(0, 0, 0)) && p1 != p2;
		REQUIRE(constructorTest == true);
	}

	SECTION("Cross-product test") {
		bool crossTest = (p2.CrossProd(p2) == Point()) && (p3.CrossProd(p2) == Point(0, 0, 1));
		REQUIRE(crossTest == true);
	}

	SECTION("Dot-product test") {
		bool dotTest = p1.Dot(p2) == 0 && p2.Dot(p2) == 1;
		REQUIRE(dotTest == true);
	}

	SECTION("Translation test") {
		Point xhat = Point(-1, -1) / sqrtf(2), yhat = Point(-1, 1) / sqrtf(2);
		Point p4 = p3; p4.Transform(xhat, yhat);
		bool transTest = p4 == Point(1, 1) / -sqrtf(2);
		REQUIRE(transTest == true);
	}


}