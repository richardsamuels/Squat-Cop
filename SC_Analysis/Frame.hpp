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

	Frame(const std::string &fromFile);

	template<typename Functor>
	void Organize(const Functor& f);

	Point& operator[](int i);

	const Point& operator[](int i) const;

	size_t size();

	void push_back(const Point& p);
};

#endif //SC_FRAME