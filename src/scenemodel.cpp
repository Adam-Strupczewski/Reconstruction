#include "stdafx.h"

#include "scenemodel.h"	
#include "logging.h"	

SceneModel::SceneModel()
{	
	counter = 0;
	distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F);
}

SceneModel::~SceneModel(){

}

void SceneModel::initialize(int w, int h){

	// Initialize camera parameters
	K = cv::Mat::eye(3, 3, CV_64F);
	K.at<double>(0,0) = qMax(w,h);
	K.at<double>(1,1) = qMax(w,h);
	K.at<double>(0,2) = w/2;
	K.at<double>(1,2) = h/2;

	LOG(Debug, "Camera internal matrix: ");
	LOG(Debug, K);
	
	// Get inverse of camera matrix
	invert(K, Kinv); 

}

void SceneModel::addNewFramePoints(std::vector<cv::KeyPoint> point){
	mutex.lock();
	keypointDatabase.push_back(point);
	mutex.unlock();
}

void SceneModel::addNewFrameDescriptors(cv::Mat descriptors){
	mutex.lock();
	descriptorDatabase.push_back(descriptors);
	mutex.unlock();
}

void SceneModel::addMatches(int i, int j, std::vector< cv::DMatch > matches){
	MatchKey key = std::make_pair(i,j);
	matchMap.insert(std::make_pair(key, matches));
}

std::vector<cv::KeyPoint> SceneModel::getKeypoints(int frame){

	std::vector<cv::KeyPoint> pts;

	mutex.lock();
	pts = keypointDatabase.at(frame);
	mutex.unlock();
	return pts;
}

cv::Mat SceneModel::getDescriptors(int frame){
	
	cv::Mat descriptors;

	mutex.lock();
	descriptors = descriptorDatabase.at(frame);
	mutex.unlock();
	return descriptors;
}

std::vector< cv::DMatch > SceneModel::getMatches(int i, int j){
	return matchMap.at(std::make_pair(i,j));
}

int SceneModel::getFrameCount(){
	return keypointDatabase.size();
}

int SceneModel::getNumberOfObservations(){
	int n=0;
	for (int i=0; i<reconstructedPts.size(); ++i){
		for (int j=0; j<reconstructedPts[0].imgpt_for_img.size(); ++j){
			if (reconstructedPts[i].imgpt_for_img[j]!=-1)
				n++;
		}
	}
	return n;
}
