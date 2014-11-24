#include "Frame.hpp"

using namespace std;

//Reads in line containing points
Frame::Frame(const string &fromFile){
	int middleComma, lparen, rparen, start = 0;
	number = stoi(fromFile.substr(0, fromFile.find(',', 0)));
	while ((lparen = fromFile.find('(', start)) != string::npos){
		rparen = fromFile.find(')', start);
		middleComma = fromFile.find(',', lparen);
		pointList.push_back(Point(stof(fromFile.substr(lparen + 1, middleComma - (lparen + 1))), stof(fromFile.substr(middleComma + 1, rparen - (middleComma + 1)))));
		start = rparen + 1;
	}
}

Point& Frame::operator[](int i) {
	if (i < 0){
		i += pointList.size();
	}
	return pointList[i];
}

const Point& Frame::operator[](int i) const {
	if (i < 0){
		i += pointList.size();
	}
	return pointList[i];
}

size_t Frame::size(){
	return pointList.size();
}

void Frame::push_back(const Point& p){
	pointList.push_back(p);
}