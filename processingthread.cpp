#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QDebug>

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
			LOG(Debug, "Extracting keypoints...");
			int minHessian = 400;
			cv::SurfFeatureDetector detector(minHessian);
            //cv::FeatureDetector * detector = new cv::BRISK();
            std::vector<cv::KeyPoint> keypoints;
            detector.detect(img, keypoints);
            //delete detector;

			// Extract descriptors
            cv::Mat descriptors;
            if(keypoints.size())
            {
				cv::SurfDescriptorExtractor extractor;
                //cv::DescriptorExtractor * extractor = new cv::BRISK();
                extractor.compute(img, keypoints, descriptors);
                //delete extractor;
                if((int)keypoints.size() != descriptors.rows)
                {
                    LOG(Debug, "Error extracting descriptors");
                }else{
					LOG(Debug, "Descriptors extracted successfully");
				}
                if(imageGrayScale)
                {
                    cvReleaseImage(&imageGrayScale);
                }
            }
            else
            {
                LOG(Debug, "WARNING: no features detected");
            }

			// Add extracted keypoints and descriptors to global model
			sceneModel->addNewFramePoints(keypoints);
			sceneModel->addNewFrameDescriptors(descriptors);

			// Display projected model here...
			// cv::drawKeypoints requires color image!
			cv::Mat imgWithKeypoints;
			cv::drawKeypoints( cv::Mat(&iplImage)/*img*/, keypoints, imgWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
			currentFrameWithKeypoints = cvtCvMat2QImage(imgWithKeypoints);
			//cv::imshow("Test.jpg", imgWithKeypoints);
			//cv::waitKey(0);

			// Here we want to process the already added keypoints and create a stereo model
			// Later will add bundle adjustment etc
			if (sceneModel->getFrameCount()==2){

				// PURELY TEST CODE - one-time model update
				std::vector<cv::KeyPoint> keypointsPrevious = sceneModel->getKeypoints(0);
				cv::Mat descriptorsPrevious = sceneModel->getDescriptors(0);

/////////////////////////////////////////////////////////////////////////////////////////
				
				if (descriptors.empty() || descriptorsPrevious.empty()){
					LOG(Warn, "No descriptors for the last two images");
					return;
				}

				// Find matches
				//cv::FlannBasedMatcher matcher;
				cv::BFMatcher matcher;
				std::vector< cv::DMatch > matches;
				matcher.match( descriptors, descriptorsPrevious, matches );

				LOG(Debug, "Calculated matches with Flann");
			
				// Calculate min/max distance
				double max_dist = 0; double min_dist = 100;
				
				for( int i = 0; i < descriptors.rows; i++ )
				{ 
					double dist = matches[i].distance;
					if( dist < min_dist ) min_dist = dist;
					if( dist > max_dist ) max_dist = dist;
				}

				LOG(Info, "-- Max dist : ", max_dist);
				LOG(Info, "-- Min dist : ", min_dist);

				// Select only "good" matches (i.e. whose distance is less than 3*min_dist )
				std::vector< cv::DMatch > goodMatches;

				for( int i = 0; i < descriptors.rows; i++ )
				{ if( matches[i].distance < 3*min_dist )
					{ goodMatches.push_back( matches[i]); }
				}

				// Check if there are sufficient good matches
				if (goodMatches.size() < 30){
					LOG(Debug, "Insufficient good matches to find homography");
					return;
				}

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

				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[0], keypointsPrevious, sceneModel->frames[1], keypoints,
					goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
					std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

				//-- Show detected matches
				imshow( "Good Matches & Object detection", img_matches );
				cv::waitKey(0);

/////////////////////////////////////////////////////////////////////////////////////////

				//computeStereo(keypoints, keypointsPrevious);
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
