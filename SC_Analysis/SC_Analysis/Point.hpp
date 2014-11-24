#pragma once

#ifndef SC_POINT
#define SC_POINT

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

struct Point{
	double x, y, z;

	//Constructors
	Point(const double[]);
	Point(double = 0, double = 0, double = 0);

	//Overloaded operators
	Point & operator=(const Point&);
	Point & operator+=(const Point&);
	Point operator / (const double);
	Point operator * (const double);
	Point & operator *=(const double);
	Point & operator/=(const double);
	bool operator ==(const Point&) const;
	bool operator !=(const Point&) const;
	friend std::ostream& operator<<(std::ostream&, const Point&);
	//Main operators
	double Dot(const Point&) const;
	double Norm() const;
	Point& Normalize();
	void Transform(const Point&, const Point&);
	Point CrossProd(const Point&);
	void Round();

};

//Comparison functor
struct LHSYLess {
	bool operator()(const Point& lhs, const Point& rhs){
		if (lhs.z == rhs.z){
			if (lhs.y == rhs.y)
				return lhs.x < rhs.x;
			else
				return lhs.y < rhs.y;
		}
		else
			return lhs.z < rhs.z;
	}
};

Point operator+(const Point&, const Point& );
Point operator-(const Point&, const Point&);

std::ostream& operator<<(std::ostream&, const Point&);

#endif //SC_POINT