// Shapes.h : header
//

#pragma once

#include "stdafx.h"


class Shape {
public:
	Shape(int cx, int cy, int nsides) : nsides(nsides), centerx(cx), centery(cy)
	{
	}
	Shape(int cx, int cy, int nsides, int length) : nsides(nsides), centerx(cx), centery(cy), length(length)
	{
	}
	virtual ~Shape() {};
	virtual void drawOn(Mat &img) = 0;

public:
	int nsides;
	int length;
	int centerx, centery;
};


class Circle : public Shape {
public:
	Circle(int cx, int cy, int radius) : Shape(cx, cy, 0), radius(radius) {};

	virtual ~Circle() {};
public:
	int radius;
};


class Triangle : public Shape {
public:
	Triangle(int cx, int cy, int len, float votescore) : Shape(cx, cy, 3, len), score(votescore) {};
	Triangle(Point center, Point a, Point b, Point c) : Shape(center.x, center.y, 3), A(a), B(b), C(c) {};
	virtual ~Triangle() {};
	virtual void drawOn(Mat &img) {
		/*if (debug_lv1){
			char text[20] = { 0 };
			sprintf(text, "%d/%.1f/%.1f", length, total_votes, single_votes/ * ///%.1f/%.1f, total_votes, single_votes* /);
			putText(img, text, Point(centerx - 6, centery + R + 8), 0, 0.35, Scalar(100, 200, 250), 1, 8, false);
		}*/
		line(img, A, B, Scalar(0, 0, 255), 1);
		line(img, B, C, Scalar(0, 0, 255), 1);
		line(img, C, A, Scalar(0, 0, 255), 1);
	}
public:
	float score;
	Point A, B, C;
	bool up_down;					// true : up; false : down.
};


class Square : public Shape {

};


void drawShapes(const std::vector<Triangle*> &shapes, Mat &img);