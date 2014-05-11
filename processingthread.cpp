#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include "cameramatrixextraction.h"
#include "bundleadjustment.h"
#include "triangulation.h"

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
	int imageCnt = 0;

	// Camera parameters - necessary for reconstruction
	cv::Matx34d P0,P1;
	cv::Mat_<double> t;
	cv::Mat_<double> R;
	cv::Mat_<double> rvec(1,3); 

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
			cvMat.copyTo(sceneModel->framesRGB[imageCnt-1]);

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
			int idx2 = imageCnt-1;
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
			if (imageCnt == 2){
		
#ifdef __DEBUG__DISPLAY__
				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
								sceneModel->frames[1], sceneModel->getKeypoints(1),
								sceneModel->getMatches(0,1), img_matches);
				imshow( "Good Matches", img_matches );
				cv::waitKey(0);
#endif

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
					LOG (Warn, "Baseline triangulation failed!");
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

				cv::Matx34d P1 = sceneModel->poseMats[1];
				t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
				R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2), 
																P1(1,0), P1(1,1), P1(1,2), 
																P1(2,0), P1(2,1), P1(2,2));
				Rodrigues(R, rvec);

				// Update sets which track what was processed 
				sceneModel->doneViews.clear();
				sceneModel->goodViews.clear();
				sceneModel->doneViews.insert(0);
				sceneModel->doneViews.insert(1);
				sceneModel->goodViews.insert(0);
				sceneModel->goodViews.insert(1);

			}else if(imageCnt > 2){
				// If this is the 3rd or next image, use it to refine stereo model

				// Find correspondences from this image to already reconstructed 3D points
				// We don't need to find image with highest correspondences [Snavely07 4.2], 
				// as we are processing images consecutively as they appear
				std::vector<cv::Point3f> _3dPoints; std::vector<cv::Point2f> _2dPoints;
				LOG(Debug, "Finding 2D-3D correpondences for image ", imageCnt);
				find2D3DCorrespondences(imageCnt-1,_3dPoints,_2dPoints);
				sceneModel->doneViews.insert(imageCnt);

				// Find pose of camera for new image based on these correspondences
				bool poseEstimated = findPoseEstimation(imageCnt-1,rvec,t,R,_3dPoints,_2dPoints);
				if(!poseEstimated){
					LOG (Warn, "Pose estimation for view ", imageCnt , "failed. Continuing.");
					continue;
				}

				// Store estimated pose	
				sceneModel->poseMats[imageCnt-1] = cv::Matx34d	(R(0,0),R(0,1),R(0,2),t(0),
																R(1,0),R(1,1),R(1,2),t(1),
																R(2,0),R(2,1),R(2,2),t(2));

				// Triangulate current view with all previous good views to reconstruct more 3D points
				// Good views are those views for which camera poses were calculated successfully
				//...
				for (std::set<int>::iterator goodView = sceneModel->goodViews.begin(); goodView != sceneModel->goodViews.end(); ++goodView) 
				{
					int view = *goodView;
					if( view == imageCnt-1 ) continue; // Skip current, this should never be reached

					LOG(Debug, "Triangulating current view with view ", view);
			
					std::vector<CloudPoint> newTriangulated;
					std::vector<int> addToCloud;
					bool goodTriangulation = triangulatePointsBetweenViews(imageCnt-1,view,newTriangulated,addToCloud,imageCnt-1);
					if(!goodTriangulation){
						LOG(Warn, "Triangulating current view with view ", view, " has failed! Continuing.");
						continue;
					}

					LOG(Info, "Number of points before triangulation ", (int)sceneModel->reconstructedPts.size());
					for (int j=0; j<addToCloud.size(); j++) {
						if(addToCloud[j] == 1)
							sceneModel->reconstructedPts.push_back(newTriangulated[j]);
					}
					LOG(Info, "Number of points after triangulation ", (int)sceneModel->reconstructedPts.size());;
				}
				sceneModel->goodViews.insert(imageCnt);

				// Bundle adjustment for all views & points reconstructed so far
				cv::Mat temporaryCameraMatrix = sceneModel->K;
				BundleAdjustment BA;
				BA.adjustBundle(sceneModel->reconstructedPts,temporaryCameraMatrix,sceneModel->getKeypoints(),sceneModel->poseMats);
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

void ProcessingThread::find2D3DCorrespondences(int currentView, 
	std::vector<cv::Point3f>& ppcloud, 
	std::vector<cv::Point2f>& imgPoints) 
{
	ppcloud.clear(); imgPoints.clear();

	// Initialize status of all reconstructed points as not matched with current view
	std::vector<int> pcloud_status(sceneModel->reconstructedPts.size(),0);

	// For all views that were already processed up till now
	for (std::set<int>::iterator goodView = sceneModel->goodViews.begin(); goodView != sceneModel->goodViews.end(); ++goodView) 
	{
		int oldView = *goodView;
		// Check for matches between the already processed view and the current view
		std::vector<cv::DMatch> matches_from_old_to_working = sceneModel->getMatches()[std::make_pair(oldView,currentView)];

		// For every match check if the 2D point from already processed view is reconstructed
		for (unsigned int match_from_old_view=0; match_from_old_view < matches_from_old_to_working.size(); match_from_old_view++) {
			
			// The index of the matching point in <oldView>
			int idx_in_old_view = matches_from_old_to_working[match_from_old_view].queryIdx;

			// Scan the existing cloud (pcloud) to see if this point from <oldView> exists
			for (unsigned int reconstructedPt=0; reconstructedPt<sceneModel->reconstructedPts.size(); reconstructedPt++) {
				
				// See if corresponding point was found in this point
				if (idx_in_old_view == sceneModel->reconstructedPts[reconstructedPt].imgpt_for_img[oldView] && pcloud_status[reconstructedPt] == 0) //prevent duplicates
				{
					// Add 3d point to 3d correspondence vector
					ppcloud.push_back(sceneModel->reconstructedPts[reconstructedPt].pt);
					// Add 2d point in image i to 2d correspondence vector
					imgPoints.push_back((sceneModel->getKeypoints())[currentView][matches_from_old_to_working[match_from_old_view].trainIdx].pt);

					pcloud_status[reconstructedPt] = 1;
					// Once a match is found - break, no more matches can be found
					break;
				}
			}
		}
	}
	LOG(Debug, "Found ", ppcloud.size(), " 3d-2d point correspondences");
}

bool ProcessingThread::findPoseEstimation(
	int working_view,
	cv::Mat_<double>& rvec,
	cv::Mat_<double>& t,
	cv::Mat_<double>& R,
	std::vector<cv::Point3f> ppcloud,
	std::vector<cv::Point2f> imgPoints
	) 
{
	if(ppcloud.size() <= 7 || imgPoints.size() <= 7 || ppcloud.size() != imgPoints.size()) { 
		//something went wrong aligning 3D to 2D points..
		LOG(Warn, "Couldn't find [enough] corresponding cloud points... (only ", ppcloud.size(), ") points found");
		return false;
	}

	std::vector<int> inliers;
	if(true/*!use_gpu*/) {
		//use CPU
		double minVal,maxVal; 
		cv::minMaxIdx(imgPoints,&minVal,&maxVal);
		// TODO is this the best way to find camera pose?
		cv::solvePnPRansac(ppcloud, imgPoints, sceneModel->K, sceneModel->distortionCoefficients, 
						rvec, t, true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);
	} else {

#ifdef HAVE_OPENCV_GPU
		//use GPU ransac
		//make sure datatstructures are cv::gpu compatible
		cv::Mat ppcloud_m(ppcloud); ppcloud_m = ppcloud_m.t();
		cv::Mat imgPoints_m(imgPoints); imgPoints_m = imgPoints_m.t();
		cv::Mat rvec_,t_;

		cv::gpu::solvePnPRansac(ppcloud_m,imgPoints_m,K_32f,distcoeff_32f,rvec_,t_,false);

		rvec_.convertTo(rvec,CV_64FC1);
		t_.convertTo(t,CV_64FC1);
#endif
	}

	std::vector<cv::Point2f> projected3D;
	cv::projectPoints(ppcloud, rvec, t, sceneModel->K, sceneModel->distortionCoefficients, projected3D);

	// Find inliers by measuring reprojection error
	if(inliers.size()==0) {
		for(int i=0;i<projected3D.size();i++) {
			if(norm(projected3D[i]-imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}
	}

#if 0
	//display reprojected points and matches
	cv::Mat reprojected; imgs_orig[working_view].copyTo(reprojected);
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::line(reprojected,imgPoints[ppt],projected3D[ppt],cv::Scalar(0,0,255),1);
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::line(reprojected,imgPoints[inliers[ppt]],projected3D[inliers[ppt]],cv::Scalar(0,0,255),1);
	}
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::circle(reprojected, imgPoints[ppt], 2, cv::Scalar(255,0,0), CV_FILLED);
		cv::circle(reprojected, projected3D[ppt], 2, cv::Scalar(0,255,0), CV_FILLED);			
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::circle(reprojected, imgPoints[inliers[ppt]], 2, cv::Scalar(255,255,0), CV_FILLED);
	}
	stringstream ss; ss << "inliers " << inliers.size() << " / " << projected3D.size();
	putText(reprojected, ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255), 2);

	cv::imshow("__tmp", reprojected);
	cv::waitKey(0);
	cv::destroyWindow("__tmp");
#endif

	if(inliers.size() < (double)(imgPoints.size())/5.0) {
		LOG(Warn, "Not enough inliers to consider a good pose - ", inliers.size(), "/", imgPoints.size());
		return false;
	}

	if(cv::norm(t) > 200.0) {
		LOG(Warn, "Estimated camera movement is too big, skip this camera");
		return false;
	}

	cv::Rodrigues(rvec, R);
	if(!checkCoherentRotation(R)) {
		LOG(Warn, "Rotation is incoherent. We should try a different base view...");
		return false;
	}

	LOG(Info, "Found t = ");
	LOG(Info, t);
	LOG(Info, "Found R = ");
	LOG(Info, R);

	return true;
}

bool ProcessingThread::triangulatePointsBetweenViews(
	int workingView, 
	int olderView,
	std::vector<struct CloudPoint>& newTriangulated,
	std::vector<int>& addToCloud,
	int numViews) 
{
	LOG (Debug, "Triangulating image ", workingView, " and ", olderView);
	cv::Matx34d P0 = sceneModel->poseMats[olderView];
	cv::Matx34d P1 = sceneModel->poseMats[workingView];

	// Get aligned matched 2D points from the two views
	std::vector<cv::KeyPoint> pt_set1,pt_set2;
	std::vector<cv::DMatch> matches = sceneModel->getMatches()[std::make_pair(olderView,workingView)];
	for (unsigned int i=0; i<matches.size(); i++) {
		pt_set1.push_back((sceneModel->getKeypoints())[olderView][matches[i].queryIdx]);
		pt_set2.push_back((sceneModel->getKeypoints())[workingView][matches[i].trainIdx]);
	}

	// Adding more triangulated points to general cloud
	// Why not 4 triangulations as in stereo initialization?
	double reprojectionError = triangulatePoints(pt_set1, pt_set2, sceneModel->K, sceneModel->Kinv, sceneModel->distortionCoefficients, P0, P1, newTriangulated);
	LOG(Info, "Triangulation reprojection error ", reprojectionError);

	std::vector<uchar> trig_status;
	if(!testTriangulation(newTriangulated, P0, trig_status) || 
		!testTriangulation(newTriangulated, P1, trig_status)) {
		LOG(Warn, "Triangulation did not succeed");
		return false;
	}
	if(reprojectionError> 50.0) {
		LOG(Warn, "Reprojection error too high - triangulation failed.");
		return false;
	}

	// Filter out outlier points with high reprojection
	std::vector<double> reprojectionErrors;
	for(int i=0;i<newTriangulated.size();i++) {
		reprojectionErrors.push_back(newTriangulated[i].reprojection_error); 
	}
	std::sort(reprojectionErrors.begin(),reprojectionErrors.end());
	// Get the 80% precentile
	double reprj_err_cutoff = reprojectionErrors[4 * reprojectionErrors.size() / 5] * 2.4; //threshold from Snavely07 4.2
	
	std::vector<CloudPoint> newTriangulatedFiltered;
	std::vector<cv::DMatch> newMatches;
	for(int i=0;i<newTriangulated.size();i++) {
		if(trig_status[i] == 0)
			continue; //point was not in front of either camera
		if(newTriangulated[i].reprojection_error > 16.0) {
			continue; //reject point as it is inaccurate
		} 
		if(newTriangulated[i].reprojection_error < 4.0 ||
			newTriangulated[i].reprojection_error < reprj_err_cutoff) 
		{
			newTriangulatedFiltered.push_back(newTriangulated[i]);
			newMatches.push_back(matches[i]);
		} 
		else 
		{
			continue;
		}
	}

	LOG(Info, " Filtered out ", (newTriangulated.size() - newTriangulatedFiltered.size()), " high-error points");

	// Check if all points are filtered out
	if(newTriangulatedFiltered.size() <= 0){
		LOG(Warn, "All points have been filtered out.");
		return false;
	}

	// Use filtered points now
	newTriangulated.clear();
	newTriangulated.insert(newTriangulated.begin(), newTriangulatedFiltered.begin(), newTriangulatedFiltered.end());	
	// Use filtered matches
	matches = newMatches;
	
	// Update the matches storage
	sceneModel->getMatches()[std::make_pair(olderView,workingView)] = newMatches; //just to make sure, remove if unneccesary
	sceneModel->getMatches()[std::make_pair(workingView,olderView)] = featureHandler->flipMatches(newMatches);
	
	// Now determine which points should be added to the cloud based on which already exist
	
	addToCloud.clear();
	addToCloud.resize(newTriangulated.size(),1);
	int foundOtherViewsCount = 0;

	// Scan new triangulated points, if they were already triangulated before - strengthen cloud
	// If not, mark them to be added
	for (int j = 0; j<newTriangulated.size(); j++) {
		newTriangulated[j].imgpt_for_img.resize(numViews,-1);

		//matches[j] corresponds to new_triangulated[j]
		//matches[j].queryIdx = point in <olderView>
		//matches[j].trainIdx = point in <workingView>
		newTriangulated[j].imgpt_for_img[olderView] = matches[j].queryIdx;		//2D reference to <olderView>
		newTriangulated[j].imgpt_for_img[workingView] = matches[j].trainIdx;	//2D reference to <workingView>
		
		bool foundInOtherView = false;
		for (unsigned int view = 0; view < numViews; view++) {
			if(view != olderView) {
				// Look for points in <view> that match to points in <workingView>
				std::vector<cv::DMatch> submatches = sceneModel->getMatches()[std::make_pair(view,workingView)];
				for (unsigned int ii = 0; ii < submatches.size(); ii++) {
					if (submatches[ii].trainIdx == matches[j].trainIdx && !foundInOtherView) 
					{
						// Point was already found in <view> - strengthen it in the known cloud, if it exists there
						for (unsigned int pt3d=0; pt3d<sceneModel->reconstructedPts.size(); pt3d++) {
							if (sceneModel->reconstructedPts[pt3d].imgpt_for_img[view] == submatches[ii].queryIdx) 
							{
								//sceneModel->reconstructedPts[pt3d] - a point that has 2d reference in <view>
								{
									// As this point has already been reconstructed in <view>, update 2D match references
									sceneModel->reconstructedPts[pt3d].imgpt_for_img[workingView] = matches[j].trainIdx;
									sceneModel->reconstructedPts[pt3d].imgpt_for_img[olderView] = matches[j].queryIdx;
									foundInOtherView = true;
									addToCloud[j] = 0;
								}
							}
						}
					}
				}
			}
		}

		{
			if (foundInOtherView) {
				// If point was found in any of the previous views
				foundOtherViewsCount++;
			} else {
				addToCloud[j] = 1;
			}
		}
	}
	LOG(Info, "Number of points found in other views: ", foundOtherViewsCount, "/", newTriangulated.size());
	LOG(Debug, "Adding new points from triangulation: ", cv::countNonZero(addToCloud));
	return true;
}