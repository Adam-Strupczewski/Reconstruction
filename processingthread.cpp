#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QDebug>
#include <QTextStream>

#include <opencv2/nonfree/features2d.hpp>

static const int QUEUE_MAX_LENGTH = 5;
static const int THREAD_SLEEP_MS = 25;

ProcessingThread::ProcessingThread(QObject *parent) :
    QThread(parent), stopped(false), queueMaxLength(QUEUE_MAX_LENGTH)
{
}

ProcessingThread::~ProcessingThread()
{
    stop();
}

void ProcessingThread::initialize(SceneModel *sceneModel)
{
	this->sceneModel = sceneModel;
}

void ProcessingThread::stop()
{
    stopped = true;
}

void ProcessingThread::addFrameToProcessingQueue(QImage frame)
{
    if (queue.length() < queueMaxLength) {
        QImage threadCopy = frame.copy();
        queue.enqueue(threadCopy);
    } else {
        LOG(Debug, "Queue is full");
        emit queueFull();
    }
}

void ProcessingThread::run()
{
    // Process until stop() called
    while (!stopped)
    {
        if (!queue.isEmpty())
        {
            currentFrame = queue.dequeue();

            LOG(Debug, "Processing image");

            // Here you can do whatever processing you need on the frame
            //currentFrame.save("E:/PROGRAMOWANIE/workspace-qt/img.png");

            /***************************************************************/
            // Process last frame
            IplImage iplImage = *(cvtQImage2IplImage(currentFrame));

			// TODO this will be removed in future, now for visualization
			sceneModel->frames.push_back(cv::Mat(&iplImage));

            // Convert image to grayscale
            IplImage * imageGrayScale = 0;
            if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
            {
                LOG(Debug, "Creating grascale...");
                imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
                cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
            }

			// Create Mat for further processing
			LOG(Debug, "Creating mat...");
            cv::Mat img;
            if(imageGrayScale)
            {
                img = cv::Mat(imageGrayScale);
            }
            else
            {
                img =  cv::Mat(&iplImage);
            }

            // Extract keypoints
			std::vector<cv::KeyPoint> keypoints;
			extractKeypoints(img, keypoints);

			// Extract descriptors
            cv::Mat descriptors;
			extractDescriptors(img, keypoints, descriptors);

			if(imageGrayScale)
            {
                cvReleaseImage(&imageGrayScale);
            }

			// Add extracted keypoints and descriptors to global model
			// TODO Add only if viewpoint has changed sufficiently
			sceneModel->addNewFramePoints(keypoints);
			sceneModel->addNewFrameDescriptors(descriptors);

			// cv::drawKeypoints requires color image!
			cv::Mat imgWithKeypoints;
			cv::drawKeypoints( cv::Mat(&iplImage)/*img*/, keypoints, imgWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
			currentFrameWithKeypoints = cvtCvMat2QImage(imgWithKeypoints);
			//cv::imshow("Test.jpg", imgWithKeypoints);
			//cv::waitKey(0);

			// Here we want to process the already added keypoints and create / refine a stereo model
			if (sceneModel->getFrameCount()>1){

				int idx1 = sceneModel->getFrameCount()-2;
				int idx2 = sceneModel->getFrameCount()-1;
				/* 
				For each incoming frame, once there are at least two frames stored
				create a 3D stereo models from the last two frames. This will be changed
				to a single stereo initialisation, online model expansion and periodic 
				bundle adjustment refinements.
				*/

				std::vector< cv::DMatch > goodMatches;
				findMatches(idx1, idx2, goodMatches);

				// Store good matches in model
				sceneModel->addMatches(idx1,idx2,goodMatches);
				LOG(Debug, "Stored matches");

				/*
				// Calculate homography
				std::vector<cv::Point2f> pts1;
				std::vector<cv::Point2f> pts2;

				for( int i = 0; i < goodMatches.size(); i++ )
				{
					//-- Get the keypoints from the good matches
					pts1.push_back( keypoints[ goodMatches[i].queryIdx ].pt );
					pts2.push_back( keypointsPrevious[ goodMatches[i].trainIdx ].pt );
				}

				cv::Mat H = cv::findHomography( pts1, pts2, CV_RANSAC );
				LOG(Debug, "Calculated homography");
				*/

				// Assuming we have two images appropriate for initialization - calculate stereo
				// For this we need the 5-point algorithm and triangulation
				// TODO Move this to the correct place

				// End of stereo initialization
				LOG(Debug, "Initialized stereo model");

				
				// DEBUG - Drawing matches
				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[idx1], sceneModel->getKeypoints(idx1), 
								sceneModel->frames[idx2], sceneModel->getKeypoints(idx2),
								goodMatches, img_matches);
				imshow( "Good Matches & Object detection", img_matches );
				cv::waitKey(0);
				
			}

			
			/* 
			Create online map:
			1. Find keypoints in each image
			2. Match keypoints from each image to previous keypoints
			3. Initialize 3D scene from stereo pair using 5-point algorithm & select scale
			4. Perform local and global bundle adjustment sequentially - for this save all previous descriptors
			5. Iterate, gradually creating larger map
			*/

			/*
			 Bundler:
			 Compute matches for each image pair using ANN (Arya, et al. [1998])
			 Robustly estimate fundamental matrix for each pair using Ransac
			 Find matching images
			 Perform alignment - find homographies, starting from best matches
			 Update global model, refine using bundle adjustment
			 */
            emit frameProcessed();
        }
        else
        {
            // No frames in queue, sleep for a short while
            msleep(THREAD_SLEEP_MS);
        }
    }
}

bool ProcessingThread::extractKeypoints(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints)
{
	int minHessian = 100;
	cv::SurfFeatureDetector detector(minHessian);
    //cv::FeatureDetector * detector = new cv::BRISK();
    detector.detect(img, keypoints);
    //delete detector;
	if(keypoints.size()==0){
		LOG(Debug, "No keypoints found!");
		return false;
	}

	LOG(Debug, "Finished extracting keypoints");
	return true;
}

bool ProcessingThread::extractDescriptors(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    if(keypoints.size())
    {
		cv::SurfDescriptorExtractor extractor;
        //cv::DescriptorExtractor * extractor = new cv::BRISK();
		//cv::DescriptorExtractor * extractor = new cv::SIFT();
		//cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
		//cv::DescriptorExtractor * extractor = new cv::FREAK();
		//cv::DescriptorExtractor * extractor = new cv::ORB();
        extractor.compute(img, keypoints, descriptors);
        //delete extractor;

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

	LOG(Debug, "Finished calculating descriptors");
	return true;
}

bool ProcessingThread::findMatches(int idx1, int idx2, std::vector< cv::DMatch > &matchesOut)
{
	LOG(Debug, "Calculating matches...");

	// Get keypoints & descriptors
	std::vector<cv::KeyPoint> keypoints1 = sceneModel->getKeypoints(idx1);
	cv::Mat descriptors1 = sceneModel->getDescriptors(idx1);
	std::vector<cv::KeyPoint> keypoints2 = sceneModel->getKeypoints(idx2);
	cv::Mat descriptors2 = sceneModel->getDescriptors(idx2);
			
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
	cv::BFMatcher matcher( cv::NORM_L1, true );
	std::vector< cv::DMatch > matches;


	////////////////////////////////////////////

	//matcher.match( descriptors1, descriptors2, matches );
		
	////////////////////////////////////////////
	
	std::vector<double> dists;
	if (matches.size() == 0) {
		std::vector<std::vector<cv::DMatch> > nn_matches;
		matcher.knnMatch(descriptors1,descriptors2,nn_matches,1);
		matches.clear();
		for(int i=0;i<nn_matches.size();i++) {
			if(nn_matches[i].size()>0) {
				// If there are matches, push back the best match
				matches.push_back(nn_matches[i][0]);
				// Get the distance of this match
				double dist = matches.back().distance;
				// Insert distance to distance vector
				if(fabs(dist) > 10000) dist = 1.0;
				matches.back().distance = dist;
				dists.push_back(dist);
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
	double cutoff = 3.0*min_dist;
	std::set<int> existing_trainIdx;
	for(unsigned int i = 0; i < matches.size(); i++ )
	{ 
		//"normalize" matching: somtimes imgIdx is the one holding the trainIdx
		if (matches[i].trainIdx <= 0) {
			matches[i].trainIdx = matches[i].imgIdx;
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

	// Check if there are sufficient good matches
	if (goodMatches.size() < 8){
		LOG(Debug, "Insufficient good matches to find homography");
		return false;
	}

	matchesOut = goodMatches;
	
	LOG(Debug, "Finished finding matches");
	return true;
}
