#include "stdafx.h"

#include "common.h"
#include "triangulation.h"

double triangulatePoints(const std::vector<cv::KeyPoint>& pt_set1, 
					   const std::vector<cv::KeyPoint>& pt_set2, 
					   const cv::Mat& K,
					   const cv::Mat& Kinv,
					   const cv::Mat& distcoeff,
					   const cv::Matx34d& P0,
					   const cv::Matx34d& P1,
					   std::vector<CloudPoint>& reconstructedPts)
{

#ifdef __DEBUG__DISPLAY__
	std::vector<double> depths;
#endif
	
//	pointcloud.clear();
	//correspImg1Pt.clear();
	
	cv::Matx44d P1_(P1(0,0),P1(0,1),P1(0,2),P1(0,3),
				P1(1,0),P1(1,1),P1(1,2),P1(1,3),
				P1(2,0),P1(2,1),P1(2,2),P1(2,3),
				0,		0,		0,		1);
	cv::Matx44d P1inv(P1_.inv());
	
	LOG(Debug, "Triangulating points...");

	double time = cv::getTickCount();
	std::vector<double> reproj_error;
	unsigned int pts_size = pt_set1.size();
	
#if 0
	//Using OpenCV's triangulation
	//convert to Point2f
	vector<Point2f> _pt_set1_pt,_pt_set2_pt;
	KeyPointsToPoints(pt_set1,_pt_set1_pt);
	KeyPointsToPoints(pt_set2,_pt_set2_pt);
	
	//undistort
	Mat pt_set1_pt,pt_set2_pt;
	undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
	undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);
	
	//triangulate
	Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2);
	Mat pt_set2_pt_2r = pt_set2_pt.reshape(1, 2);
	Mat pt_3d_h(1,pts_size,CV_32FC4);
	cv::triangulatePoints(P,P1,pt_set1_pt_2r,pt_set2_pt_2r,pt_3d_h);

	//calculate reprojection
	vector<Point3f> pt_3d;
	convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
	cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P(0,0),P(0,1),P(0,2), P(1,0),P(1,1),P(1,2), P(2,0),P(2,1),P(2,2));
	Vec3d rvec; Rodrigues(R ,rvec);
	Vec3d tvec(P(0,3),P(1,3),P(2,3));
	vector<Point2f> reprojected_pt_set1;
	projectPoints(pt_3d,rvec,tvec,K,distcoeff,reprojected_pt_set1);

	for (unsigned int i=0; i<pts_size; i++) {
		CloudPoint cp; 
		cp.pt = pt_3d[i];
		pointcloud.push_back(cp);
		reproj_error.push_back(norm(_pt_set1_pt[i]-reprojected_pt_set1[i]));
	}
#else
	// Use state-of-the-art Iterative Linear LS Triangulation described by Hartley...
	cv::Mat_<double> KP1 = K * cv::Mat(P1);
	for (int i=0; i<pts_size; i++) {
		cv::Point2f kp = pt_set1[i].pt; 
		cv::Point3d u(kp.x,kp.y,1.0);
		cv::Mat_<double> um = Kinv * cv::Mat_<double>(u); 
		u.x = um(0); u.y = um(1); u.z = um(2);

		cv::Point2f kp1 = pt_set2[i].pt; 
		cv::Point3d u1(kp1.x,kp1.y,1.0);
		cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1); 
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);
		
		// Calculate each 3D point using image positions and camera view matrices
		cv::Mat_<double> X = iterativeLinearLSTriangulation(u,P0,u1,P1);

		cv::Mat_<double> xPt_img = KP1 * X;				//reproject
//		cout <<	"Point * K: " << xPt_img << endl;
		cv::Point2f reprojectedPoint(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));			
		{
			double reprj_err = norm(reprojectedPoint-kp1);
			reproj_error.push_back(reprj_err);

			CloudPoint cp; 
			cp.pt = cv::Point3d(X(0),X(1),X(2));
			cp.reprojection_error = reprj_err;
			
			reconstructedPts.push_back(cp);
			//correspImg1Pt.push_back(pt_set1[i]);
#ifdef __DEBUG__DISPLAY__
			depths.push_back(X(2));
#endif
		}
	}
#endif
	
	cv::Scalar mse = cv::mean(reproj_error);
	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	LOG(Info, "Point cloud size: ", int(reconstructedPts.size()));
	LOG(Info, "Time: ", time);
	LOG(Info, "Mean reprojection error: ", mse[0]);

	//show "range image"
#ifdef __DEBUG__DISPLAY__
	{
		double minVal,maxVal;
		cv::minMaxLoc(depths, &minVal, &maxVal);
		cv::Mat tmp(480,640,CV_8UC3,cv::Scalar(0,0,0));
		for (unsigned int i=0; i<reconstructedPts.size(); i++) {
			double _d = MAX( (MIN((reconstructedPts[i].pt.z-minVal)/(maxVal-minVal),1.0)) , 0.0);
			circle(tmp, pt_set1[i].pt, 1, cv::Scalar(255 * (1.0-(_d)),255,255), CV_FILLED);
		}
		cvtColor(tmp, tmp, CV_HSV2BGR);
		imshow("Depth Map", tmp);
		cv::waitKey(0);
	}	
#endif
	
	LOG(Debug, "Finished triangulating points...");
	return mse[0];
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,		//homogenous image point (u,v,1)
																cv::Matx34d P,			//camera 1 matrix
																cv::Point3d u1,			//homogenous image point in 2nd camera
																cv::Matx34d P1			//camera 2 matrix
																) {
	double wi = 1, wi1 = 1;
	cv::Mat_<double> X(4,1); 
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		cv::Mat_<double> X_ = linearLSTriangulation(u,P,u1,P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
		
		// Recalculate weights
		double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
		double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
		
		// Breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
		
		wi = p2x;
		wi1 = p2x1;
		
		// Reweight equations and solve
		cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,		
				  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,		
				  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,	
				  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1
				  );

		cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<	  -(u.x*P(2,3)	-P(0,3))/wi,
												  -(u.y*P(2,3)	-P(1,3))/wi,
												  -(u1.x*P1(2,3)	-P1(0,3))/wi1,
												  -(u1.y*P1(2,3)	-P1(1,3))/wi1
						  );
		
		solve(A,B,X_,cv::DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}

	return X;
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Mat_<double> linearLSTriangulation(cv::Point3d u,		//homogenous image point (u,v,1)
														cv::Matx34d P,		//camera 1 matrix
														cv::Point3d u1,		//homogenous image point in 2nd camera
														cv::Matx34d P1		//camera 2 matrix
														) 
{
	
	// Build matrix A for homogenous equation system Ax = 0
	// Assume X = (x,y,z,1), for Linear-LS method
	// Which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	//
	//	cout << "u " << u <<", u1 " << u1 << endl;
	//	Matx<double,6,4> A; //this is for the AX=0 case, and with linear dependence..
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u.x*P(1)-u.y*P(0);
	//	A(3) = u1.x*P1(2)-P1(0);
	//	A(4) = u1.y*P1(2)-P1(1);
	//	A(5) = u1.x*P(1)-u1.y*P1(0);
	//	Matx43d A; //not working for some reason...
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u1.x*P1(2)-P1(0);
	//	A(3) = u1.y*P1(2)-P1(1);
	cv::Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),		
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	cv::Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	cv::Mat_<double> X;
	solve(A,B,X,cv::DECOMP_SVD);
	
	return X;
}

// Checks which 3D points are in front of the camera
bool testTriangulation(const std::vector<CloudPoint>& pcloud, 
	const cv::Matx34d& P, 
	std::vector<uchar>& status) {

	std::vector<cv::Point3d> pcloud_pt3d = cloudPointsToCvPoints(pcloud);
	std::vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());
	
	cv::Matx44d P4x4 = cv::Matx44d::eye(); 
	for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];
	
	perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);
	
	status.resize(pcloud.size(),0);
	for (int i=0; i<pcloud.size(); i++) {
		status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
	}
	int count = cv::countNonZero(status);

	double percentage = ((double)count / (double)pcloud.size());
	LOG(Info, "Percentage of points in front of camera: ", percentage*100.0);

	if(percentage < 0.75){
		LOG(Debug, "Too few points in front of camera!");
		return false;
	}

	//check for coplanarity of points
	if(false) //omit
	{
		cv::Mat_<double> cldm(pcloud.size(),3);
		for(unsigned int i=0;i<pcloud.size();i++) {
			cldm.row(i)(0) = pcloud[i].pt.x;
			cldm.row(i)(1) = pcloud[i].pt.y;
			cldm.row(i)(2) = pcloud[i].pt.z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

		int num_inliers = 0;
		cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
		cv::Vec3d x0 = pca.mean;
		double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

		for (int i=0; i<pcloud.size(); i++) {
			cv::Vec3d w = cv::Vec3d(pcloud[i].pt) - x0;
			double D = fabs(nrm.dot(w));
			if(D < p_to_plane_thresh) num_inliers++;
		}

		LOG(Info, "Number of points that are coplanar: ", num_inliers);
		if((double)num_inliers / (double)(pcloud.size()) > 0.85){
			LOG(Debug,"Too manu points are coplanar!");
			return false;
		}
	}

	return true;
}