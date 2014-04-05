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

std::vector<cv::KeyPoint> SceneModel::getNextFramePoints(){

	std::vector<cv::KeyPoint> pts;

	mutex.lock();
	if (counter < keypointDatabase.size()){
		pts = keypointDatabase.at(counter++);
	}else{
		pts = keypointDatabase.at(counter-1);
	}
	mutex.unlock();
	return pts;
}

cv::Mat SceneModel::getNextFrameDescriptors(){
	
	cv::Mat descriptors;

	mutex.lock();
	if (counter < keypointDatabase.size()){
		descriptors = descriptorDatabase.at(counter++);
	}else{
		descriptors = descriptorDatabase.at(counter-1);
	}
	mutex.unlock();
	return descriptors;
}

int SceneModel::getFrameCount(){
	return keypointDatabase.size();
}