#ifndef SCENEMODEL_H
#define SCENEMODEL_H

#include "settings.h"
#include "structs.h"

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef std::pair<int, int> MatchKey;

class SceneModel
{

public:
    explicit SceneModel();
    virtual ~SceneModel();

	void addNewFramePoints(std::vector<cv::KeyPoint> point);
	void addNewFrameDescriptors(cv::Mat descriptors);
	void addMatches(int i, int j, std::vector< cv::DMatch > macthes);

	std::vector<cv::KeyPoint> getKeypoints(int frame);
	cv::Mat getDescriptors(int frame);
	int getFrameCount();

	// TEMPORARY FOR VISUALIZATION
	std::vector<cv::Mat> frames;

private:
	// Vector of vectors with keypoints for each image
	std::vector<std::vector<cv::KeyPoint>> keypointDatabase;

	// Vector of Mts with descriptors for each image
	std::vector<cv::Mat> descriptorDatabase;

	// Vector with matches assigned to image pairs
	std::map<std::pair<int, int>, std::vector< cv::DMatch >> matchMap;

	// Current scene model - all computed 3D scene feature points
	std::vector<Point3Df> allPoints;

	int counter;
	QMutex mutex;
};

#endif