#include "stdafx.h"

#include "featurehandler.h"
#include "logging.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/nonfree/features2d.hpp>

FeatureHandler::FeatureHandler(){
}

FeatureHandler::~FeatureHandler(){
	delete detector;
	delete extractor;
}

void FeatureHandler::initialize()
{
	int minHessian = 100;
	detector = new cv::SURF(minHessian);
	//detector = new cv::BRISK();
	//detector = new cv::MSER();


	//extractor = new cv::BRISK();
	extractor = new cv::SIFT();
	//extractor = new cv::SURF();
	//extractor = new cv::BRISK();
	//extractor = new cv::BriefDescriptorExtractor();
	//extractor = new cv::FREAK();
	//extractor = new cv::ORB();
}

bool FeatureHandler::extractKeypoints(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints)
{
	double time = cv::getTickCount();

    detector->detect(img, keypoints);
    //delete detector;
	if(keypoints.size()==0){
		LOG(Debug, "No keypoints found!");
		return false;
	}

	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	LOG(Debug, "Finished extracting keypoints in time: ", time);
	return true;
}

bool FeatureHandler::extractDescriptors(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
	double time = cv::getTickCount();

    if(keypoints.size())
    {
        extractor->compute(img, keypoints, descriptors);

        if((int)keypoints.size() != descriptors.rows)
        {
            LOG(Debug, "Error extracting descriptors!");
        }else{
			LOG(Debug, "Descriptors extracted successfully");
		}
    }
    else
    {
        LOG(Debug, "WARNING: no features detected");
		return false;
    }

	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	LOG(Debug, "Finished calculating descriptors in time: ", time);
	return true;
}

bool FeatureHandler::findMatches(int idx1, int idx2, std::vector<cv::KeyPoint> &keypoints1, cv::Mat &descriptors1, 
	std::vector<cv::KeyPoint> &keypoints2, cv::Mat &descriptors2, std::vector< cv::DMatch > &matchesOut)
{
	double time = cv::getTickCount();
	LOG(Debug, "Calculating matches...");
			
	LOG(Info, "Image 1 number of keypoints: ", (int)keypoints1.size());
	LOG(Info, "Image 1 number of descriptors: ", (int)descriptors1.size().height);
	LOG(Info, "Image 2 number of keypoints: ", (int)keypoints2.size());
	LOG(Info, "Image 2 number of descriptors: ", (int)descriptors2.size().height);

	if (descriptors1.empty() || descriptors2.empty()){
		LOG(Warn, "No descriptors for the previous image");
		return false;
	}

	// Find matches
	//cv::FlannBasedMatcher matcher;
	cv::BFMatcher matcher( cv::NORM_L1, true ); // Using cross-checking as an alternative to ratio checking
	std::vector< cv::DMatch > matches;

	////////////////////////////////////////////
	
	// First option - simple matching
	//matcher.match( descriptors1, descriptors2, matches );
		
	////////////////////////////////////////////
	
	// Second option - more sophisticated matching
	std::vector<double> dists;
	if (matches.size() == 0) {
		std::vector<std::vector<cv::DMatch> > nn_matches;
		// If cross-checking is used, last parameter of knnmatch has to be 1!
		matcher.knnMatch(descriptors1,descriptors2,nn_matches,1);
		matches.clear();
		for(int i=0;i<nn_matches.size();i++) {
			if(nn_matches[i].size()>0) {
				// Take best match only if it is sufficiently better than the second best match - 
				// Alternatively use cross-checking
				//if(nn_matches[i].size()==1 || (nn_matches[i][0].distance) < 0.6*(nn_matches[i][1].distance))
				//{
					matches.push_back(nn_matches[i][0]);
					// Get the distance of this match
					double dist = matches.back().distance;
					// Insert distance to distance vector
					if(fabs(dist) > 10000) dist = 1.0;
					matches.back().distance = dist;
					dists.push_back(dist);
				//}
			}
		}
	}

	////////////////////////////////////////////

	LOG(Debug, "Calculated coarse matches");
	LOG(Info, "Number of coarse matches: ", (int)matches.size());
	
	double max_dist = 0; double min_dist = 0.0;
	cv::minMaxIdx(dists,&min_dist,&max_dist);
	
	LOG(Info, "-- Max dist : ", max_dist);
	LOG(Info, "-- Min dist : ", min_dist);
	
	if (min_dist < 0.02) {
		min_dist = 0.02;
	}
	
	// Select only good matches
	std::vector< cv::DMatch > goodMatches;

	// Eliminate any re-matching of training points (multiple queries to one training)
	// Usual threshold is 3*min_dist...
	double cutoff = 6.0*min_dist;
	std::set<int> existing_trainIdx;
	for(unsigned int i = 0; i < matches.size(); i++ )
	{ 
		//"normalize" matching: somtimes imgIdx is the one holding the trainIdx
		if (matches[i].trainIdx <= 0) {
			matches[i].trainIdx = matches[i].imgIdx;
			LOG(Info, "Weird match - rematching");
		}
		
		int tidx = matches[i].trainIdx;
		if(matches[i].distance > 0.0 && matches[i].distance < cutoff) {
			if( existing_trainIdx.find(tidx) == existing_trainIdx.end() && 
			   tidx >= 0 && tidx < (int)(keypoints2.size()) ) 
			{
				goodMatches.push_back( matches[i]);
				existing_trainIdx.insert(tidx);
			}
		}
	}

	LOG(Info, "Number of matches after refinement: ", (int)goodMatches.size());

	// Optional - filter matches using F matrix with RANSAC
	// Currently this is being done later...
	/*
    std::vector<uchar> inliers(points1.size(),0);
    cv::Mat fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2),inliers,CV_FM_RANSAC,distance,confidence); // confidence probability
    // extract the surviving (inliers) matches
    std::vector<uchar>::const_iterator
    itIn= inliers.begin();
    std::vector<cv::DMatch>::const_iterator
    itM= matches.begin();
    // for all matches
    for ( ;itIn!= inliers.end(); ++itIn, ++itM)
    {
        if (*itIn)
        { // it is a valid match
            goodMatches.push_back(*itM);
        }
    }
	*/

	// Check if there are sufficient good matches
	if (goodMatches.size() < 8){
		LOG(Debug, "Insufficient good matches to find homography");
		return false;
	}

	matchesOut = goodMatches;
	
	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	LOG(Debug, "Finished finding matches in time: ", time);
	return true;
}