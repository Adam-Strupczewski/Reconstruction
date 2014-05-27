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

	void initialize(int w, int h);

	// Camera parameters
	cv::Mat K;
	cv::Mat_<double> Kinv;
	cv::Mat distortionCoefficients;

	void addNewFramePoints(std::vector<cv::KeyPoint> point);
	void addNewFrameDescriptors(cv::Mat descriptors);
	void addMatches(int i, int j, std::vector< cv::DMatch > macthes);

	std::vector<cv::KeyPoint> getKeypoints(int frame);
	std::vector<std::vector<cv::KeyPoint> > getKeypoints(){
		return keypointDatabase;
	}
	cv::Mat getDescriptors(int frame);
	std::map<std::pair<int, int>, std::vector< cv::DMatch > > getMatches(){
		return matchMap;
	}
	std::vector< cv::DMatch > getMatches(int i, int j);

	std::vector<cv::Matx34d> getCameras() { 
		// Need to copy map to vector
		std::vector<cv::Matx34d> v; 
		for(std::map<int ,cv::Matx34d>::const_iterator it = poseMats.begin(); it != poseMats.end(); ++it ) {
			v.push_back( it->second );
		}
		return v;
    }

	int getFrameCount();

	int getNumberOfObservations();

	QString folderPath;
	QStringList imageFileList;
	QStringList keypointFileList;
	int imageLimit;

	// TEMPORARY FOR VISUALIZATION
	std::vector<cv::Mat> frames;
	std::vector<cv::Mat_<cv::Vec3b> > framesRGB;

	std::map<int,cv::Matx34d> poseMats;
	//std::map<std::pair<int,int> ,std::vector<cv::DMatch> > matches_matrix;

	// Structure with all the reconstructed 3D points
	std::vector<CloudPoint> reconstructedPts;

	// Keypoints refined with F matrix
	//std::vector<std::vector<cv::KeyPoint> > keypointsGood;

	// For multiview reconstruction
	std::set<int> doneViews;
	std::set<int> goodViews;

private:	// TODO clean-up public / private vars 
			// Some are public some are private - should be made consistent

	// Vector of vectors with keypoints for each image
	std::vector<std::vector<cv::KeyPoint> > keypointDatabase;

	// Vector of Mts with descriptors for each image
	std::vector<cv::Mat> descriptorDatabase;

	// Vector with matches assigned to image pairs
	std::map<std::pair<int, int>, std::vector< cv::DMatch > > matchMap;

	int counter;
	QMutex mutex;
};

#endif