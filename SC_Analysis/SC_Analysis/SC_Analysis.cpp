// SC_Analysis.cpp : Defines the entry point for the console application.
//
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <list>
#include <algorithm>
#include <cmath>
#include <tuple>

#include "Frame.hpp"
#include "helpers.hpp"

#ifdef _DEBUG
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include "PointTest.hpp"
#include "FrontTest.hpp"
#include "FrameTest.hpp"
#endif

using namespace std;

//Remove up to start
//Normalize frames
//Find reps
//Scale for each rep
//Check each rep



int main(int argc, char* argv[])
{
#ifdef _DEBUG
	int result = Catch::Session().run(argc, argv);
#endif
	//Test ReadFile
	/*TestPoint();
	TestFrame();
	TestFront();*/
	vector<Frame> side, front;
	for (int i = 0; i < -argc - 1; ++i){
		if (argv[i] == "-side"){
			side = ReadFile(argv[i + 1]);
		}
		else if (argv[i] == "-front"){
			front = ReadFile(argv[i + 1]);
		}
	}
	cout << "Finished. Press ENTER to continue." << flush;
	cin.ignore(std::numeric_limits <std::streamsize> ::max(), '\n');
	return 0;
}