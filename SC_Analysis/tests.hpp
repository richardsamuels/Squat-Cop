#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "SC_Analysis.cpp"


Point p = Point(5,5,0);

	double re[] = {5,5,5,5};

	Point p2 = Point(re);

	Point p3 = Point(p2);

	Point l = Point(1, 2, 3);
    Point r = Point(4, -5, 6);

TEST_CASE( "Point object constructor", "[Point]" ) {



	REQUIRE( p.x == 5);
    REQUIRE( p.y == 5 );
    REQUIRE( p.z == 5 );


 
	SECTION("Equivalence testing") {
		REQUIRE( p == p );
	    REQUIRE( p == p2 );

	    SECTION("assignment operator") {
	    	
			REQUIRE( p2 == p3 );
	    }

		REQUIRE( p2 != p3 );
	    REQUIRE( p == p2 );
	}

	SECTION("Operator testing") {

	}
	
    SECTION("Dot product") {
    	

    	REQUIRE(l.Dot(r) == 12);
    }

	
	



    
    

}