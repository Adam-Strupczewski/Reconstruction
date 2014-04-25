#ifndef FEATUREHANDLER_H
#define FEATUREHANDLER_H

#include "scenemodel.h"

class FeatureHandler
{

public:
    explicit FeatureHandler();
    virtual ~FeatureHandler();

	bool extractKeypoints(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints);

	bool extractDescriptors(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

	bool findMatches(int idx1, int idx2, std::vector<cv::KeyPoint> &keypoints1, cv::Mat &descriptors1, 
	std::vector<cv::KeyPoint> &keypoints2, cv::Mat &descriptors2, std::vector< cv::DMatch > &matchesOut);

};

#endif