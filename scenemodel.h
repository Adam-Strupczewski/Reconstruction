#ifndef SCENEMODEL_H
#define SCENEMODEL_H

#include "structs.h"

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

class SceneModel
{

public:
    explicit SceneModel();
    virtual ~SceneModel();

	void addNewFramePoints(std::vector<cv::KeyPoint> point);
	void addNewFrameDescriptors(cv::Mat descriptors);

	std::vector<cv::KeyPoint> getNextFramePoints();
	cv::Mat getNextFrameDescriptors();
	int getFrameCount();

private:
	// Vector of vectors with keypoints for each image
	std::vector<std::vector<cv::KeyPoint>> keypointDatabase;

	// Vector of Mts with descriptors for each image
	std::vector<cv::Mat> descriptorDatabase;

	// Current scene model - all computed 3D scene feature points
	std::vector<Point3Df> allPoints;

	int counter;
	QMutex mutex;
};

#endif