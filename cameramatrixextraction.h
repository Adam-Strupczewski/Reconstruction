#ifndef CAMERAMATRIXEXTRACTION_H
#define CAMERAMATRIXEXTRACTION_H

#include "scenemodel.h"

// Finds camera matrices and returns reconstructed 3d points from triangulation
bool findCameraMatrices(const cv::Mat& K, 
					const cv::Mat& Kinv, 
					const cv::Mat& distcoeff,
					const std::vector<cv::KeyPoint>& keypoints1,
					const std::vector<cv::KeyPoint>& keypoints2,
					std::vector<cv::KeyPoint>& keypoints1_refined,
					std::vector<cv::KeyPoint>& keypoints2_refined,
					cv::Matx34d& P0,
					cv::Matx34d& P1,
					std::vector<cv::DMatch>& matches,
					std::vector<CloudPoint>& reconstructedCloud);

// Not called directly from reconstruction loop
cv::Mat findFundamentalMatrix(const std::vector<cv::KeyPoint>& keypoints1,
						const std::vector<cv::KeyPoint>& keypoints2,
						std::vector<cv::KeyPoint>& keypoints1_refined,
						std::vector<cv::KeyPoint>& keypoints2_refined,
						std::vector<cv::DMatch>& matches);
	
bool decomposeEtoRandT(cv::Mat_<double>& E,
						cv::Mat_<double>& R1,
						cv::Mat_<double>& R2,
						cv::Mat_<double>& t1,
						cv::Mat_<double>& t2);

void takeSVDOfE(cv::Mat_<double>& E, 
				cv::Mat& svd_u, 
				cv::Mat& svd_vt, 
				cv::Mat& svd_w);

void DecomposeEssentialUsingHorn90(double _E[9], 
									double _R1[9], 
									double _R2[9], 
									double _t1[3], 
									double _t2[3]);

bool checkCoherentRotation(cv::Mat_<double>& R);

#endif