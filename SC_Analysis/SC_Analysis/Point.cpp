#include "Point.hpp"

using namespace std;

//Point class
Point::Point(const double rectangle[]){
	x = rectangle[0] + rectangle[2] / 2;
	y = rectangle[1] + rectangle[3] / 2;
	z = 0;
}

Point::Point(double x, double y, double z) : x(x), y(y), z(z){}

//Overloaded operators
Point& Point::operator=(const Point &rhs){
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	return *this;
}
Point& Point::operator+=(const Point &rhs){
	Point p = *this + rhs;
	x = p.x;
	y = p.y;
	z = p.z;
	return *this;
}
Point Point::operator / (const double value){
	return Point(x / value, y / value, z / value);
}
Point Point::operator * (const double value){
	return Point(x * value, y * value, z * value);
}
Point& Point::operator *=(const double value){
	x *= value;
	y *= value;
	z *= value;
	return *this;
}
Point& Point::operator/=(const double value){
	x /= value;
	y /= value;
	z /= value;
	return *this;
}
bool Point::operator==(const Point& rhs) const{
	return x == rhs.x && y == rhs.y && z == rhs.z;
}
bool Point::operator!=(const Point& rhs) const{
	return !(*this == rhs);
}

//Main operators
double Point::Dot(const Point& rhs) const{
	return x * rhs.x + y * rhs.y + z * rhs.z;
}
double Point::Norm() const{
	return sqrt((*this).Dot(*this));
}
Point& Point::Normalize(){
	*this /= Norm();
	return *this;
}
void Point::Transform(const Point& xhat, const Point& yhat){
	double xnew = (*this).Dot(xhat);
	double ynew = (*this).Dot(yhat);
	x = xnew; y = ynew;
	//return *this;
}
Point Point::CrossProd(const Point& rhs){
	double xVal, yVal, zVal;
	xVal = y * rhs.z - (z * rhs.y);
	yVal = -(x * rhs.z - (z * rhs.x));//Because determinant goes like 1 -1 1...
	zVal = x * rhs.y - (y * rhs.z);
	return Point(xVal, yVal, zVal);
}
void Point::Round(){
	x = round(x * 1000) / 1000;
	y = round(y * 1000) / 1000;
	z = round(z * 1000) / 1000;
}

//Non-member point operators
Point operator+(const Point& lhs, const Point& rhs){
	return Point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
Point operator-(const Point& lhs, const Point& rhs){
	return Point(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}
ostream& operator<<(ostream& os, const Point& p){
	if (p.z == 0)
		os << "(" << p.x << ", " << p.y << ")";
	else
		os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
	return os;
}

//Comparison functor
bool PointLHSComparator::operator()(const Point& lhs, const Point& rhs){
	if (lhs.z == rhs.z){
		if (lhs.y == rhs.y)
			return lhs.x < rhs.x;
		else
			return lhs.y < rhs.y;
	}
	else
		return lhs.z < rhs.z;
}