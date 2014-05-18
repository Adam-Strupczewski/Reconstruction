#ifndef STRUCTS_H
#define STRUCTS_H

#include "stdafx.h"

#include <opencv2/core/core.hpp>

struct Point2Di{
	int x;
	int y;
};

struct Point2Df{
	float x;
	float y;
};

struct Point3Df{
	float x;
	float y;
	float z;
};

struct CloudPoint {
	cv::Point3d pt;
	std::vector<int> imgpt_for_img;
	double reprojection_error;
};

#endif