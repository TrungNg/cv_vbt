// Detections.h : implementation file
//

#pragma once


#include "stdafx.h"
#include "Shapes.h"

enum cl_prior_t{
	NONE,
	RED,
	BLUE
};

std::vector<Triangle*> VBT_detect_uptriangle(Mat grad_x, Mat grad_y, Mat grad);
void colorTransform(Mat input, Mat &output, cl_prior_t type);

class Detector {
public:
	Detector(Mat &image) : r(image.rows), c(image.cols) {
		img = Mat(image.size(), CV_8UC1);
		colorTransform(image, img, RED);
	};

	virtual ~Detector() {};

	void preproc(double threshold);
	//std::vector<Shape*> findShape(int nsides, int maxSize, int minSize);
	std::vector<Triangle*> findShape(int nsides, int maxSize, int minSize);
	//std::vector<Triangle*> findTriangles(const std::vector<int> &sizes);
	/*std::vector<trEdge_t> ShapeFinder::findEdges(std::vector<Triangle*> shapes);*/
	const Mat &getScalarGradient() const { return mag; }
	const Mat &getV() const { return V; }

/*
protected:
	void computeGradients(double threshold);*/

private:
	int r, c;
	Mat img;
	Mat gx, gy, mag;
	Mat V;
};
