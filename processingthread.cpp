#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include "cameramatrixextraction.h"
#include "bundleadjustment.h"

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
	int imageCnt = -1;

    while (!stopped)
    {
        if (!queue.isEmpty())
        {
			imageCnt++;
            currentFrame = queue.dequeue();

            LOG(Debug, "Starting to process new image...");

            // Process next frame
            IplImage iplImage = *(cvtQImage2IplImage(currentFrame));

			// TODO this will be removed in future, now for visualization
			sceneModel->frames.push_back(cv::Mat(&iplImage));
			// TODO this is necessary for getting color for 3D reconstruction, should be made more efficient
			sceneModel->framesRGB.push_back(cv::Mat_<cv::Vec3b>());
			cv::Mat cvMat = cvtQImage2CvMat(currentFrame);
			cvMat.copyTo(sceneModel->framesRGB[imageCnt]);

            // Convert image to grayscale TODO inefficient
            IplImage * imageGrayScale = 0;
            if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
            {
                LOG(Debug, "Creating grascale from new image...");
                imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
                cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
            }

			// Create Mat for further processing TODO inefficient
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

			// Now the incoming image keypoints need to be matched to all existing images
			// TODO - maybe not to all? Snavely takes initial images according to matches & homography inliers...
			int idx2 = imageCnt;
			for (int idx1=0; idx1<idx2; idx1++){

				std::vector< cv::DMatch > goodMatches;

				// Get keypoints & descriptors - TODO get rid of copying
				std::vector<cv::KeyPoint> keypoints1 = sceneModel->getKeypoints(idx1);
				cv::Mat descriptors1 = sceneModel->getDescriptors(idx1);
				std::vector<cv::KeyPoint> keypoints2 = sceneModel->getKeypoints(idx2);
				cv::Mat descriptors2 = sceneModel->getDescriptors(idx2);
				featureHandler->findMatches(idx1, idx2, keypoints1, descriptors1, keypoints2, descriptors2, goodMatches);

				std::vector<cv::DMatch> goodMatchesFlipped = featureHandler->flipMatches(goodMatches);

				// Store good matches in model - good matches are already refined with F matrix
				sceneModel->addMatches(idx1,idx2,goodMatches);
				sceneModel->addMatches(idx2,idx1,goodMatchesFlipped);
			}
			LOG(Debug, "Stored all matches of the new image to previous images");

			// If the current image is the second image - initialize stereo model
			if (imageCnt+1 == 2){
		
#ifdef __DEBUG__DISPLAY__
				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
								sceneModel->frames[1], sceneModel->getKeypoints(1),
								sceneModel->getMatches(0,1), img_matches);
				imshow( "Good Matches", img_matches );
				cv::waitKey(0);
#endif

				// Set initial camera parameters - necessary for reconstruction
				cv::Matx34d P0,P1;

				// Set arbitrary parameters - will be refined
				P0 = cv::Matx34d(1,0,0,0,
								0,1,0,0,
								0,0,1,0);
				P1 = cv::Matx34d(1,0,0,0,
								0,1,0,0,
								0,0,1,0);

				// Attempt to find baseline traingulation
				LOG(Debug, "Trying to find baseline triangulation...");

				std::vector<cv::KeyPoint> keypoints1Refined;
				std::vector<cv::KeyPoint> keypoints2Refined;
				bool b = findCameraMatrices(sceneModel->K, sceneModel->Kinv, sceneModel->distortionCoefficients, 
									sceneModel->getKeypoints(0), sceneModel->getKeypoints(1), 
									keypoints1Refined, keypoints2Refined, P0, P1, sceneModel->getMatches(0,1), 
									sceneModel->reconstructedPts);
				
				if (b){
					LOG (Debug, "Baseline triangulation successful!");
				}else{
					LOG (Debug, "Baseline triangulation failed!");
					exit(0);
				}

				// Add found matrices to pose matrices - TODO: move
				sceneModel->poseMats[0] = P0;
				sceneModel->poseMats[1] = P1;

				// Adjust bundle for everything so far before display
				cv::Mat temporaryCameraMatrix = sceneModel->K;
				BundleAdjustment BA;
				BA.adjustBundle(sceneModel->reconstructedPts,temporaryCameraMatrix,sceneModel->getKeypoints(),sceneModel->poseMats);

				// Pass reconstructed points to 3D display		
				std::vector<cv::Point3d> points;
				// TODO unnecessary copying
				for (int i = 0; i < sceneModel->reconstructedPts.size(); ++i) {
					points.push_back(sceneModel->reconstructedPts[i].pt);
				}

				std::vector<cv::Vec3b> pointsRGB;
				getRGBForPointCloud(sceneModel->reconstructedPts, pointsRGB);
				update(points, pointsRGB);

#ifdef __DEBUG__DISPLAY__
				// DEBUG - Drawing matches that survived the fundamental matrix
				cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
								sceneModel->frames[1], sceneModel->getKeypoints(1),
								sceneModel->getMatches(0,1), img_matches, cv::Scalar(0,0,255));
				imshow( "Good Matches After F Refinement", img_matches );
				cv::waitKey(0);
#endif						

				// End of stereo initialization
				LOG(Debug, "Initialized stereo model");


			}else if(0/*imageCnt-1 == 2*/){
				// If this is the 3rd or next image, use it to refine stereo model
				
				// TODO - complete
				// Find correspondences from this image to already reconstructed 3D points
				// Find pose of camera for new image based on these correspondences
				// Triangulate current view with all previous views to reconstruct more 3D points
				// Bundle adjustment
				
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

void ProcessingThread::getRGBForPointCloud(
	const std::vector<struct CloudPoint>& pcloud,
	std::vector<cv::Vec3b>& RGBCloud) 
{
	RGBCloud.resize(pcloud.size());

	// For every point
	for (unsigned int i=0; i<pcloud.size(); i++) {
		
		// Find all views in which the point was seen
		unsigned int good_view = 0;
		std::vector<cv::Vec3b> point_colors;
		for(; good_view < sceneModel->frames.size(); good_view++) {
			if(pcloud[i].imgpt_for_img[good_view] != -1) {
				int pt_idx = pcloud[i].imgpt_for_img[good_view];
				if(pt_idx >= sceneModel->getKeypoints(good_view).size()) {
					std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << sceneModel->getKeypoints(good_view).size()  ;
					continue;
				}
				cv::Point _pt = sceneModel->getKeypoints(good_view)[pt_idx].pt;
				assert(good_view < sceneModel->frames.size() && _pt.x < sceneModel->framesRGB[good_view].cols && _pt.y < sceneModel->framesRGB[good_view].rows);
				
				point_colors.push_back(sceneModel->framesRGB[good_view].at<cv::Vec3b>(_pt));
			}
		}

		cv::Scalar res_color = cv::mean(point_colors);
		RGBCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
		if(good_view == sceneModel->frames.size()) //nothing found.. put red dot
			RGBCloud.push_back(cv::Vec3b(255,0,0));
	}
}