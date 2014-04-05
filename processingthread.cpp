#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QDebug>

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
            cv::FeatureDetector * detector = new cv::BRISK();
            std::vector<cv::KeyPoint> keypoints;
            detector->detect(img, keypoints);
            delete detector;

			// Extract descriptors
            cv::Mat descriptors;
            if(keypoints.size())
            {
                cv::DescriptorExtractor * extractor = new cv::BRISK();
                extractor->compute(img, keypoints, descriptors);
                delete extractor;
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
				std::vector<cv::KeyPoint> keypointsPrevious = sceneModel->getNextFramePoints();

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
