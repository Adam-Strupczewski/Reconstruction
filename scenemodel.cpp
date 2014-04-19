#include "stdafx.h"

#include "scenemodel.h"	

SceneModel::SceneModel(){
	counter = 0;
}

SceneModel::~SceneModel(){

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

int SceneModel::getFrameCount(){
	return keypointDatabase.size();
}