#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QDebug>
#include <QTextStream>

#include <opencv2/nonfree/features2d.hpp>

// TODO For real-time reconstrcution queue has to be more limited!
static const int QUEUE_MAX_LENGTH = 20;
static const int THREAD_SLEEP_MS = 25;

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

ProcessingThread::ProcessingThread(QObject *parent) :
    QThread(parent), stopped(false), queueMaxLength(QUEUE_MAX_LENGTH)
{
}

ProcessingThread::~ProcessingThread()
{
    stop();
	delete reconstructionHandler;
}

void ProcessingThread::initialize(SceneModel *sceneModel)
{
	this->sceneModel = sceneModel;
	this->reconstructionHandler = new ReconstructionHandler();
	this->reconstructionHandler->initialize(sceneModel);
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
	int imageInitializedCnt = 0;
	int imageReconstructedCnt = 0;

#ifdef KEYPOINTS_FROM_FILE
	// Read all matches
	//reconstructionHandler->readMatchesFromFile();
	reconstructionHandler->readMatchesFromFileAfterRansac();
#endif

    while (!stopped)
    {

		// Wait for image reading thread to read all images
		msleep(2000);

        if (!queue.isEmpty())
        {
			imageInitializedCnt++;
            currentFrame = queue.dequeue();

            LOG(Debug, "Starting to process new image...");

            // Process next frame
            IplImage iplImage = *(cvtQImage2IplImage(currentFrame));

			// TODO this will be removed in future, now for visualization
			sceneModel->frames.push_back(cv::Mat(&iplImage));
			// TODO this is necessary for getting color for 3D reconstruction, should be made more efficient
			sceneModel->framesRGB.push_back(cv::Mat_<cv::Vec3b>());
			cv::Mat cvMat = cvtQImage2CvMat(currentFrame);
			cvMat.copyTo(sceneModel->framesRGB[imageInitializedCnt-1]);

            // Convert image to grayscale TODO inefficient
            IplImage * imageGrayScale = 0;
            if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
            {
                LOG(Info, "Creating grascale from new image...");
                imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
                cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
            }

			// Create Mat for further processing TODO inefficient
			LOG(Info, "Creating mat from new image...");
            cv::Mat img;
            if(imageGrayScale)
            {
                img = cv::Mat(imageGrayScale);
            }
            else
            {
                img =  cv::Mat(&iplImage);
            }

#ifdef KEYPOINTS_FROM_FILE
			reconstructionHandler->readKeypointsFromFile(imageInitializedCnt-1);
#else
			// Will calculate keypoints, descriptors and matches
			reconstructionHandler->initializeImage(img, imageInitializedCnt);
#endif
			// Attention! cv::drawKeypoints requires color image!
			cv::Mat imgWithKeypoints;
			cv::drawKeypoints( cv::Mat(&iplImage)/*img*/, sceneModel->getKeypoints(imageInitializedCnt-1), imgWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
			currentFrameWithKeypoints = cvtCvMat2QImage(imgWithKeypoints);
			//cv::imshow("Test.jpg", imgWithKeypoints);
			//cv::waitKey(0);

			if(imageGrayScale)
			{
				cvReleaseImage(&imageGrayScale);
			}

            emit frameProcessed();
        }
        else
        {
			// In this set-up, if there are no waiting frames, choose consecutive best frames and reconstruct...
			if (imageReconstructedCnt < imageInitializedCnt){
				imageReconstructedCnt++;
				// If there remain frames with calculated features to be processed 

				// If this is the beginning - initialize stereo model
				if (imageReconstructedCnt == 1){

					if (imageInitializedCnt<2){
						// Only one image is available - cannot initialize
						LOG(Critical, "Only one image available - waiting...");
						imageReconstructedCnt--;
						break;
					}
					
					imageReconstructedCnt++;
#ifdef __DEBUG__DISPLAY__
					cv::Mat img_matches;
					cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
									sceneModel->frames[1], sceneModel->getKeypoints(1),
									sceneModel->getMatches(0,1), img_matches);
					imshow( "Good Matches", img_matches );
					cv::waitKey(0);
#endif

					std::vector<cv::Point3d> points;
					std::vector<cv::Vec3b> pointsRGB;
					bool b = reconstructionHandler->initializeStereoModel(points, pointsRGB, imageInitializedCnt);
					
					if (!b){ 
						LOG(Critical, "Exiting...");
						exit(0);
					}

					update(points, pointsRGB,sceneModel->getCameras());

#ifdef __DEBUG__DISPLAY__
					// DEBUG - Drawing matches that survived the fundamental matrix
					cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
									sceneModel->frames[1], sceneModel->getKeypoints(1),
									sceneModel->getMatches(0,1), img_matches, cv::Scalar(0,0,255));
					imshow( "Good Matches After F Refinement", img_matches );
					cv::waitKey(0);
#endif		


				}else if(imageReconstructedCnt > 2){
					
					// If this is the 3rd or next image, use it to refine stereo model

					std::vector<cv::Point3d> points;
					std::vector<cv::Vec3b> pointsRGB;
					bool b = reconstructionHandler->addNextViewToStereoModel(points, pointsRGB, imageInitializedCnt);
					
					if (!b) continue;

					update(points, pointsRGB,sceneModel->getCameras());				
				}
								
			}else{
				// No frames in queue, sleep for a short while
				msleep(THREAD_SLEEP_MS);
			}
        }
    }
}