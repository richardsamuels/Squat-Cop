#pragma once

#ifndef SC_POINT
#define SC_POINT

#include <iostream>
#include <fstream>
#include <string>

struct Point{
	double x, y, z;

	//Constructors
	Point(const double rectangle[]);
	Point(double x = 0, double y = 0, double z = 0);

	//Overloaded operators
	Point & operator=(const Point &rhs);
	Point & operator+=(const Point &rhs);
	Point operator / (const double value);
	Point operator * (const double value);
	Point & operator *=(const double value);
	Point & operator/=(const double value);
	bool operator ==(const Point& rhs) const;
	bool operator !=(const Point& rhs) const;
	friend std::ostream& operator<<(std::ostream& os, const Point& p);
	//Main operators
	double Dot(const Point& rhs) const;
	double Norm() const;
	Point& Normalize();
	void Transform(const Point& xhat, const Point& yhat);
	Point CrossProd(const Point& rhs);
	void Round();

};

//Comparison functor
struct LHSYLess {
	bool operator()(const Point& lhs, const Point& rhs);
};

Point operator+(const Point & lhs, const Point & rhs);

std::ostream& operator<<(std::ostream& os, const Point& p);

#endif //SC_POINT