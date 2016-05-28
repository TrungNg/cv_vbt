// Shapes.cpp : header
//

#include "stdafx.h"
#include "Shapes.h"

void drawShapes(const std::vector<Triangle*> &shapes, Mat &img)
{
	for (std::vector<Triangle*>::const_iterator it = shapes.begin(); it != shapes.end(); it++)
		(*it)->drawOn(img);
}