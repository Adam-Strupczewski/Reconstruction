#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#define EPSILON 0.0001

#include "common.h"
#include "logging.h"
#include "scenemodel.h"

double triangulatePoints(const std::vector<cv::KeyPoint>& pt_set1, 
					const std::vector<cv::KeyPoint>& pt_set2, 
					const cv::Mat& K,
					const cv::Mat& Kinv,
					const cv::Mat& distcoeff,
					const cv::Matx34d& P0,
					const cv::Matx34d& P1,
					std::vector<CloudPoint>& reconstructedPts);

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,	//homogenous image point (u,v,1)
												cv::Matx34d P,			//camera 1 matrix
												cv::Point3d u1,			//homogenous image point in 2nd camera
												cv::Matx34d P1			//camera 2 matrix
												);
/*
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Mat_<double> linearLSTriangulation(cv::Point3d u,		//homogenous image point (u,v,1)
										cv::Matx34d P,		//camera 1 matrix
										cv::Point3d u1,		//homogenous image point in 2nd camera
										cv::Matx34d P1		//camera 2 matrix
										);

// Checks if 3D points are in front of the camera
bool testTriangulation(const std::vector<CloudPoint>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status);

#endif