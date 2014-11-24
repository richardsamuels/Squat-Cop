#pragma once

#ifndef SC_FRAME
#define SC_FRAME

#include <cstdlib>
#include <vector>
#include <string>

#include "Point.hpp"

//A copy of frame contains the SAME data.
struct Frame{
	std::vector<Point> pointList;
	int number;

	Frame(const std::string&);

	template<typename Functor>
	void Organize(const Functor&);

	Point& operator[](int);

	const Point& operator[](int) const;

	size_t size();

	void push_back(const Point&);
};

#endif //SC_FRAME