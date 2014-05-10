#ifndef COMMON_H
#define COMMON_H

#include "structs.h"

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::vector<cv::Point3d> cloudPointsToCvPoints(const std::vector<CloudPoint> cpts) {
	std::vector<cv::Point3d> out;
	for (unsigned int i=0; i<cpts.size(); i++) {
		out.push_back(cpts[i].pt);
	}
	return out;
}

#endif