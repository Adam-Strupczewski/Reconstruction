#include "stdafx.h"

#include "reconstructionhandler.h"

#include "qtcvconversion.h"
#include "logging.h"

#include "cameramatrixextraction.h"
#include "bundleadjustment.h"
#include "triangulation.h"

#include <QDebug>
#include <QTextStream>

#include <opencv2/nonfree/features2d.hpp>

bool sortByFirst(std::pair<int,std::pair<int,int> > a, std::pair<int,std::pair<int,int> > b) 
{ return a.first < b.first; }

ReconstructionHandler::ReconstructionHandler()
{
}

ReconstructionHandler::~ReconstructionHandler()
{
	delete featureHandler;
}

void ReconstructionHandler::initialize(SceneModel *sceneModel)
{
	this->sceneModel = sceneModel;
	this->featureHandler = new FeatureHandler();
	this->featureHandler->initialize();
}

void ReconstructionHandler::initializeImage(cv::Mat img, int imageInitializedCnt){
	
	// Extract keypoints
	std::vector<cv::KeyPoint> keypoints;
	featureHandler->extractKeypoints(img, keypoints);

	// Extract descriptors
	cv::Mat descriptors;
	featureHandler->extractDescriptors(img, keypoints, descriptors);

	// Add extracted keypoints and descriptors to global model
	// TODO Add only if viewpoint has changed sufficiently
	sceneModel->addNewFramePoints(keypoints);
	sceneModel->addNewFrameDescriptors(descriptors);

	// Now the incoming image keypoints need to be matched to all existing images
	// TODO - maybe not to all? Snavely takes initial images according to matches & homography inliers...
	int idx2 = imageInitializedCnt-1;
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
}

bool ReconstructionHandler::initializeStereoModel(std::vector<cv::Point3d> &points, std::vector<cv::Vec3b> &pointsRGB, int imageInitializedCnt){
		
	// Camera parameters - necessary for reconstruction
	cv::Matx34d P0,P1;
	cv::Mat_<double> t;
	cv::Mat_<double> R;
	cv::Mat_<double> rvec(1,3); 

	// Set arbitrary parameters - will be refined
	P0 = cv::Matx34d(1,0,0,0,
					0,1,0,0,
					0,0,1,0);
	P1 = cv::Matx34d(1,0,0,0,
					0,1,0,0,
					0,0,1,0);

	// Attempt to find baseline traingulation
	LOG(Debug, "Trying to find baseline triangulation...");

	// Find best two images to start with
	std::vector<CloudPoint> tmpPcloud;

	// Sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
	// Take less matches tan Snavely suggests...
	std::list<std::pair<int,std::pair<int,int> > > matchesSizes;
	// For debugging
	std::map<std::pair<int,int> ,std::vector<cv::DMatch> > tempMatches = sceneModel->getMatches();

	for(std::map<std::pair<int,int> ,std::vector<cv::DMatch> >::iterator i = tempMatches.begin(); i != tempMatches.end(); ++i) {
		if((*i).second.size() < 60)
			matchesSizes.push_back(make_pair(100,(*i).first));
		else {
			int Hinliers = findHomographyInliers2Views((*i).first.first,(*i).first.second);
			int percent = (int)(((double)Hinliers) / ((double)(*i).second.size()) * 100.0);
			LOG(Info, "The percentage of inliers for images ", (*i).first.first, "," , (*i).first.second, " = ", percent);
			matchesSizes.push_back(make_pair((int)percent,(*i).first));
		}
	}
	matchesSizes.sort(sortByFirst);

	// Reconstruct from two views
	bool goodF = false;
	int highest_pair = 0;
	m_first_view = m_second_view = 0;
	// Reverse iterate by number of matches
	for(std::list<std::pair<int,std::pair<int,int> > >::iterator highest_pair = matchesSizes.begin(); 
		highest_pair != matchesSizes.end() && !goodF; 
		++highest_pair) 
	{
		m_second_view = (*highest_pair).second.second;
		m_first_view  = (*highest_pair).second.first;

		LOG(Debug, "Attempting reconstruction from view ", m_first_view, " and ", m_second_view);
		// If reconstrcution of first two views is bad, fallback to another pair
		// See if the Fundamental Matrix between these two views is good
		std::vector<cv::KeyPoint> keypoints1Refined;
		std::vector<cv::KeyPoint> keypoints2Refined;
		goodF = findCameraMatrices(sceneModel->K, sceneModel->Kinv, sceneModel->distortionCoefficients, 
				sceneModel->getKeypoints(m_first_view), sceneModel->getKeypoints(m_second_view), 
				keypoints1Refined, keypoints2Refined, P0, P1, sceneModel->getMatches(m_first_view,m_second_view), 
				tmpPcloud);

		if (goodF) {
			std::vector<CloudPoint> new_triangulated;
			std::vector<int> add_to_cloud;

			sceneModel->poseMats[m_first_view] = P0;
			sceneModel->poseMats[m_second_view] = P1;

			// TODO imageInitialzedCnt used to initalize correspondence matching vector
			// Will fail if more images are added later...
			bool good_triangulation = triangulatePointsBetweenViews(m_second_view,m_first_view,new_triangulated,add_to_cloud, imageInitializedCnt);
			if(!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
				LOG(Debug, " Triangulation failed");
				goodF = false;
				sceneModel->poseMats[m_first_view] = 0;
				sceneModel->poseMats[m_second_view] = 0;
				m_second_view++;
			} else {
				assert(new_triangulated[0].imgpt_for_img.size() > 0);
				LOG(Debug, " Points before triangulation: ", (int)sceneModel->reconstructedPts.size());
				for (unsigned int j=0; j<add_to_cloud.size(); j++) {
					if(add_to_cloud[j] == 1) {
						sceneModel->reconstructedPts.push_back(new_triangulated[j]);
					}
				}
				LOG(Debug, " Points after triangulation: ", (int)sceneModel->reconstructedPts.size());
			}				
		}
	}
		
	if (!goodF) {
		LOG(Error, "Cannot find a good pair of images to obtain a baseline triangulation");
		return false;
	}

	LOG(Debug, "===================================================================");
	LOG(Debug, "Taking baseline from " , m_first_view , " and " , m_second_view);
	LOG(Debug, "===================================================================");

	// Adjust bundle for everything so far before display
	cv::Mat temporaryCameraMatrix = sceneModel->K;
	BundleAdjustment BA;
	BA.adjustBundle(sceneModel->reconstructedPts,temporaryCameraMatrix,sceneModel->getKeypoints(),sceneModel->poseMats);
	updateReprojectionErrors();

	#ifdef __DEBUG__DISPLAY__
	// DEBUG - Drawing matches that survived the fundamental matrix
	cv::drawMatches(sceneModel->frames[0], sceneModel->getKeypoints(0), 
					sceneModel->frames[1], sceneModel->getKeypoints(1),
					sceneModel->getMatches(0,1), img_matches, cv::Scalar(0,0,255));
	imshow( "Good Matches After F Refinement", img_matches );
	cv::waitKey(0);
	#endif		

	// Pass reconstructed points to 3D display		
	// TODO unnecessary copying
	for (int i = 0; i < sceneModel->reconstructedPts.size(); ++i) {
		points.push_back(sceneModel->reconstructedPts[i].pt);
	}

	getRGBForPointCloud(sceneModel->reconstructedPts, pointsRGB);
				
	// End of stereo initialization
	LOG(Debug, "Initialized stereo model");

	P1 = sceneModel->poseMats[m_second_view];
	t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
	R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2), 
									P1(1,0), P1(1,1), P1(1,2), 
									P1(2,0), P1(2,1), P1(2,2));
	Rodrigues(R, rvec);

	// Update sets which track what was processed 
	sceneModel->doneViews.clear();
	sceneModel->goodViews.clear();
	sceneModel->doneViews.insert(m_first_view);
	sceneModel->doneViews.insert(m_second_view);
	sceneModel->goodViews.insert(m_first_view);
	sceneModel->goodViews.insert(m_second_view);

	return true;
}

bool ReconstructionHandler::addNextViewToStereoModel(std::vector<cv::Point3d> &points, std::vector<cv::Vec3b> &pointsRGB, int imageInitializedCnt){
	
	// Camera parameters - necessary for reconstruction
	cv::Matx34d P0,P1;
	cv::Mat_<double> t;
	cv::Mat_<double> R;
	cv::Mat_<double> rvec(1,3); 

	// Find image with highest 2d-3d correspondance [Snavely07 4.2]
	int max_2d3d_view = -1;
	int max_2d3d_count = 0;
	std::vector<cv::Point3f> _3dPoints; 
	std::vector<cv::Point2f> _2dPoints;
	for (unsigned int _i=0; _i < imageInitializedCnt; _i++) {
		if(sceneModel->doneViews.find(_i) != sceneModel->doneViews.end()) continue; //already done with this view

		std::vector<cv::Point3f> tmp3d; std::vector<cv::Point2f> tmp2d;
		find2D3DCorrespondences(_i,tmp3d,tmp2d);
		if(tmp3d.size() > max_2d3d_count) {
			max_2d3d_count = tmp3d.size();
			max_2d3d_view = _i;
			_3dPoints = tmp3d; _2dPoints = tmp2d;
		}
	}
	int i = max_2d3d_view; //highest 2d3d matching view
	LOG(Debug, "Image with most 2D-3D correspondences is ", i, " with ", max_2d3d_count);
	sceneModel->doneViews.insert(i);

	// Find pose of camera for new image based on these correspondences
	bool poseEstimated = findPoseEstimation(i,rvec,t,R,_3dPoints,_2dPoints);
	if(!poseEstimated){
		LOG (Warn, "Pose estimation for view ", i , "failed. Continuing.");
		return false;
	}

	// Store estimated pose	
	sceneModel->poseMats[i] = cv::Matx34d	(R(0,0),R(0,1),R(0,2),t(0),
											R(1,0),R(1,1),R(1,2),t(1),
											R(2,0),R(2,1),R(2,2),t(2));

	// Triangulate current view with all previous good views to reconstruct more 3D points
	// Good views are those views for which camera poses were calculated successfully
	//...
	for (std::set<int>::iterator goodView = sceneModel->goodViews.begin(); goodView != sceneModel->goodViews.end(); ++goodView) 
	{
		int view = *goodView;
		if( view == i ) continue; // Skip current
			
		std::vector<CloudPoint> newTriangulated;
		std::vector<int> addToCloud;
		bool goodTriangulation = triangulatePointsBetweenViews(i,view,newTriangulated,addToCloud, imageInitializedCnt);
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
	sceneModel->goodViews.insert(i);

	// Bundle adjustment for all views & points reconstructed so far
	cv::Mat temporaryCameraMatrix = sceneModel->K;
	BundleAdjustment BA;
	BA.adjustBundle(sceneModel->reconstructedPts,temporaryCameraMatrix,sceneModel->getKeypoints(),sceneModel->poseMats);
	updateReprojectionErrors();

	// Pass reconstructed points to 3D display		
	// TODO unnecessary copying
	for (int i = 0; i < sceneModel->reconstructedPts.size(); ++i) {
		points.push_back(sceneModel->reconstructedPts[i].pt);
	}

	getRGBForPointCloud(sceneModel->reconstructedPts, pointsRGB);
	return true;		
}

void ReconstructionHandler::getRGBForPointCloud(
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

void ReconstructionHandler::find2D3DCorrespondences(int currentView, 
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
	LOG(Debug, "Found ", ppcloud.size(), " 3d-2d point correspondences for view ", currentView);
}

bool ReconstructionHandler::findPoseEstimation(
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
						rvec, t, true, 1000, 0.004 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);
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

	// AS - Changed threshold
	if(inliers.size() < (double)(imgPoints.size())/10.0) {
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

bool ReconstructionHandler::triangulatePointsBetweenViews(
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

	if (newTriangulated.size()==0){
		LOG(Warn, "Zero points were triangulated");
		return false;
	}

	std::vector<uchar> trig_status;
	if(!testTriangulation(newTriangulated, P0, trig_status) || 
		!testTriangulation(newTriangulated, P1, trig_status)) {
		LOG(Warn, "Triangulation did not succeed");
		return false;
	}
	if(reprojectionError > 50.0) {
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
	LOG(Debug, "Number of points found in other views: ", foundOtherViewsCount, "/", newTriangulated.size());
	LOG(Debug, "Adding new points from triangulation: ", cv::countNonZero(addToCloud));
	return true;
}

void ReconstructionHandler::updateReprojectionErrors(){
	// Here we want to calculate the total reprojection error for all reconstructed points so far
	double averageTotalReprojectionError = 0.0;

	// For every 3D reconstructed point
	for (int j=sceneModel->reconstructedPts.size()-1; j>=0; j--) {

		// The point in homogenous coordinates
		cv::Mat_<double> X(4,1);
		X(0) = sceneModel->reconstructedPts[j].pt.x; 
		X(1) = sceneModel->reconstructedPts[j].pt.y; 
		X(2) = sceneModel->reconstructedPts[j].pt.z;
		X(3) = 1.0;

		// Total reprojection error for all views
		double averagePointError = 0.0;
		int appearanceCnt = 0;

		// For all views in which this point is visible
		for (int i=0; i<sceneModel->getKeypoints().size(); i++)
		{

			// If point is not visible in this view yet, continue
			if (sceneModel->reconstructedPts[j].imgpt_for_img[i] < 0){
				continue;
			}

			// Projection matrix including internal camera parameters - of current view
			cv::Mat_<double> iKP = sceneModel->K * cv::Mat(sceneModel->poseMats[i]);

			// Reproject point
			cv::Mat_<double> iPtImg = iKP * X;	

			// Get 2D point coordinates in the current view
			cv::Point2f iPt2D(iPtImg(0)/iPtImg(2),iPtImg(1)/iPtImg(2));

			// Get reprojection error
			double reprjErr = cv::norm(iPt2D - sceneModel->getKeypoints()[i][sceneModel->reconstructedPts[j].imgpt_for_img[i]].pt);

			// Add reprojection error for i-th view to total error
			averagePointError += reprjErr;
			appearanceCnt++;

		}

		// Normalize reprojection error for point
		averagePointError = averagePointError / appearanceCnt;

		// If reprojection error too high remove point
		if (averagePointError > 30){
			sceneModel->reconstructedPts.erase(sceneModel->reconstructedPts.begin()+j);
			continue;
		}

		// Add point error to total
		averageTotalReprojectionError += averagePointError;

		// Set point error as its updated error
		sceneModel->reconstructedPts[j].reprojection_error = averagePointError;

	}

	// Normalize total reprojection error
	averageTotalReprojectionError = averageTotalReprojectionError / sceneModel->reconstructedPts.size();

	LOG(Debug, "===================================================================");
	LOG(Debug, "Finished BA, average total reprojection error: ", averageTotalReprojectionError);
	LOG(Debug, "Total number of 3D points: ", (int) sceneModel->reconstructedPts.size());
	LOG(Debug, "Total number of point observations: ", sceneModel->getNumberOfObservations());
	LOG(Debug, "===================================================================");
}

// Following Snavely07 4.2 - find how many inliers are in the Homography between 2 views
int ReconstructionHandler::findHomographyInliers2Views(int vi, int vj) 
{
	std::vector<cv::KeyPoint> ikpts,jkpts; 
	std::vector<cv::Point2f> ipts,jpts;
	getAlignedPointsFromMatch(sceneModel->getKeypoints(vi),sceneModel->getKeypoints(vj),
		sceneModel->getMatches()[std::make_pair(vi,vj)],ikpts,jkpts);
	keyPointsToPoints(jkpts,jpts);
	keyPointsToPoints(ikpts,ipts);

	double minVal,maxVal; cv::minMaxIdx(ipts,&minVal,&maxVal); //TODO flatten point2d?? or it takes max of width and height

	std::vector<uchar> status;
	cv::Mat H = cv::findHomography(ipts,jpts,status,CV_RANSAC, 0.004 * maxVal); //threshold from Snavely07
	return cv::countNonZero(status); //number of inliers
}

void ReconstructionHandler::readMatchesFromFile(){
	QString filePath = sceneModel->folderPath + "matches.init.txt";
	QFile f(filePath);
	f.open(QIODevice::ReadOnly);
	QTextStream in(&f);

	while(!in.atEnd()) {
		std::vector< cv::DMatch > matches;

		QString line = in.readLine();    
		QStringList fields = line.split(" ");    
		int imgIdx1 = fields[0].toInt();
		int imgIdx2 = fields[1].toInt();
		line = in.readLine();  
		int numberMatches = line.toInt();
		for (int i=0; i<numberMatches; ++i){
			QString line = in.readLine();
			QStringList fields = line.split(" ");
			int matchIdx1 = fields[0].toInt();
			int matchIdx2 = fields[1].toInt();
			matches.push_back(cv::DMatch(matchIdx1, matchIdx2, 1.0));
		}
		sceneModel->addMatches(imgIdx1, imgIdx2, matches);

		std::vector<cv::DMatch> matchesFlipped = featureHandler->flipMatches(matches);
		sceneModel->addMatches(imgIdx2, imgIdx1,matchesFlipped);
	}

	f.close();
}

void ReconstructionHandler::readMatchesFromFileAfterRansac(){


	QString filePath1 = sceneModel->folderPath + "nmatches.ransac.txt";
	QFile f1(filePath1);
	f1.open(QIODevice::ReadOnly);
	QTextStream in1(&f1);

	QString filePath2= sceneModel->folderPath + "matches.ransac.txt";
	QFile f2(filePath2);
	f2.open(QIODevice::ReadOnly);
	QTextStream in2(&f2);

	QString line = in1.readLine();    
	int numImages = line.toInt();

	for (int i=0; i<numImages; ++i){
		line = in1.readLine();
		QStringList fields = line.split(" ");
		for (int j=0; j<numImages; ++j){
			
			std::vector< cv::DMatch > matches;
			int numberMatches = fields[j].toInt();

			// If no matches - continue
			if (numberMatches==0)
				continue;

			//We want to read these matches from second file
			while (1){
				QString lineMatches = in2.readLine();   
				if (lineMatches.length()==0)
					continue;
				// We have valid line
				QStringList fieldsMatches = lineMatches.split(" ");
				for (int k=0; k<numberMatches; ++k){
					int matchIdx1 = fieldsMatches[2*k].toInt();
					int matchIdx2 = fieldsMatches[2*k+1].toInt();
					matches.push_back(cv::DMatch(matchIdx1, matchIdx2, 1.0));
				}
				break;
			}
			sceneModel->addMatches(i, j, matches);
			std::vector<cv::DMatch> matchesFlipped = featureHandler->flipMatches(matches);
			sceneModel->addMatches(j, i,matchesFlipped);	
		}
	}

	f1.close();
	f2.close();
}

void ReconstructionHandler::readKeypointsFromFile(int fileIndex)
{
	// Alternatively load keypoints and matches from file
	std::vector<cv::KeyPoint> keypoints;

	QString filePath = sceneModel->folderPath + sceneModel->keypointFileList.at(fileIndex);
	QFile f(filePath);
	f.open(QIODevice::ReadOnly);
	QTextStream in(&f);

	QString line = in.readLine();    
	QStringList fields = line.split(" ");
	int numFeatures = fields[0].toInt();
	int sizeOfFeature = fields[1].toInt();

	// Every feature is one line position, scale and orientation
	// Next seven lines are feature data
	// We only need info from first line
	for (int i=0; i<numFeatures; ++i){
		QString line = in.readLine();    
		QStringList fields = line.split(" ");
		double y = fields[0].toDouble();
		double x = fields[1].toDouble();
		double size = fields[2].toDouble();
		double angle = fields[3].toDouble();
		keypoints.push_back(cv::KeyPoint(x,y,size,angle));
		// Read and discard actual keypoint data
		for (int j=0; j<7; ++j){
			in.readLine();
		}
	}

	f.close();
	sceneModel->addNewFramePoints(keypoints);
}