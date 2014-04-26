#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include "cameramatrixextraction.h"

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
	delete featureHandler;
}

void ProcessingThread::initialize(SceneModel *sceneModel)
{
	this->sceneModel = sceneModel;
	this->featureHandler = new FeatureHandler();
	this->featureHandler->initialize();
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

            LOG(Debug, "Starting to process new image...");

            // Process next frame
            IplImage iplImage = *(cvtQImage2IplImage(currentFrame));

			// TODO this will be removed in future, now for visualization
			sceneModel->frames.push_back(cv::Mat(&iplImage));

            // Convert image to grayscale
            IplImage * imageGrayScale = 0;
            if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
            {
                LOG(Debug, "Creating grascale from new image...");
                imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
                cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
            }

			// Create Mat for further processing
			LOG(Debug, "Creating mat from new image...");
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
			featureHandler->extractKeypoints(img, keypoints);

			// Extract descriptors
            cv::Mat descriptors;
			featureHandler->extractDescriptors(img, keypoints, descriptors);

			if(imageGrayScale)
            {
                cvReleaseImage(&imageGrayScale);
            }

			// Add extracted keypoints and descriptors to global model
			// TODO Add only if viewpoint has changed sufficiently
			sceneModel->addNewFramePoints(keypoints);
			sceneModel->addNewFrameDescriptors(descriptors);

			// Attention! cv::drawKeypoints requires color image!
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
				
				// Get keypoints & descriptors
				std::vector<cv::KeyPoint> keypoints1 = sceneModel->getKeypoints(idx1);
				cv::Mat descriptors1 = sceneModel->getDescriptors(idx1);
				std::vector<cv::KeyPoint> keypoints2 = sceneModel->getKeypoints(idx2);
				cv::Mat descriptors2 = sceneModel->getDescriptors(idx2);
				featureHandler->findMatches(idx1, idx2, keypoints1, descriptors1, keypoints2, descriptors2, goodMatches);

				// Store good matches in model
				sceneModel->addMatches(idx1,idx2,goodMatches);
				LOG(Debug, "Stored matches");
		
#ifdef __DEBUG__DISPLAY__
				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[idx1], sceneModel->getKeypoints(idx1), 
								sceneModel->frames[idx2], sceneModel->getKeypoints(idx2),
								goodMatches, img_matches);
				imshow( "Good Matches", img_matches );
				cv::waitKey(0);
#endif

				// Set camera parameters - necessary for reconstruction
				cv::Matx34d P0,P1;
				cv::Mat K;
				cv::Mat_<double> Kinv;
				cv::Mat distortion_coeff;
				std::vector<CloudPoint> reconstructedPts;

				P0 = cv::Matx34d(1,0,0,0,
								0,1,0,0,
								0,0,1,0);
				P1 = cv::Matx34d(1,0,0,50,
								0,1,0,0,
								0,0,1,0);

				K = cv::Mat::eye(3, 3, CV_64F);
				K.at<double>(0,0) = 115;
				K.at<double>(1,1) = 115;
				K.at<double>(0,2) = 320;
				K.at<double>(1,2) = 240;

				LOG(Debug, "Camera internal matrix: ");
				LOG(Debug, K);

				distortion_coeff = cv::Mat::zeros(8, 1, CV_64F);
				invert(K, Kinv); //get inverse of camera matrix

				// Calculate camera matrices from fundamental matrix
				std::vector<cv::KeyPoint> keypoints1Refined;
				std::vector<cv::KeyPoint> keypoints2Refined;

				findCameraMatrices(K, Kinv, distortion_coeff, 
									sceneModel->getKeypoints(idx1), sceneModel->getKeypoints(idx1), 
									keypoints1Refined, keypoints2Refined, P0, P1, goodMatches, reconstructedPts);
								
				// TODO
				// We have the reconstructed points from 2 views here
				// We want to store them and refine them with future images
				// In the end, we want to add bundle adjustment

#ifdef __DEBUG__DISPLAY__
				// DEBUG - Drawing matches that survived the fundamental matrix
				cv::drawMatches(sceneModel->frames[idx1], sceneModel->getKeypoints(idx1), 
								sceneModel->frames[idx2], sceneModel->getKeypoints(idx2),
								goodMatches, img_matches, cv::Scalar(0,0,255));
				imshow( "Good Matches After F Refinement", img_matches );
				cv::waitKey(0);
#endif						

				// End of stereo initialization
				LOG(Debug, "Initialized stereo model");


			}
		
			/* 
			PTAM:
			1. Find keypoints in each image
			2. Match keypoints from each image to previous keypoints
			3. Initialize 3D scene from stereo pair using 5-point algorithm & select scale
			4. Perform local and global bundle adjustment sequentially - for this save all previous descriptors
			5. Iterate, gradually creating larger map
			*/

			/*
			Bundler:
			1. Compute matches for each image pair using ANN (Arya, et al. [1998])
			2. Robustly estimate fundamental matrix for each pair using Ransac
			3. Find matching images
			4. Perform alignment - find homographies, starting from best matches
			5. Update global model, refine using bundle adjustment
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