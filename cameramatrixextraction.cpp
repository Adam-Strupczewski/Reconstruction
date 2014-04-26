#include "stdafx.h"

#include "cameramatrixextraction.h"

#include "logging.h"
#include "triangulation.h"

bool findCameraMatrices(const cv::Mat& K, 
						const cv::Mat& Kinv, 
						const cv::Mat& distcoeff,
						const std::vector<cv::KeyPoint>& keypoints1,
						const std::vector<cv::KeyPoint>& keypoints2,
						std::vector<cv::KeyPoint>& keypoints1_refined,
						std::vector<cv::KeyPoint>& keypoints2_refined,
						cv::Matx34d& P0,
						cv::Matx34d& P1,
						std::vector<cv::DMatch>& matches,
						std::vector<CloudPoint>& reconstructedCloud){

	double time = cv::getTickCount();

	// Matches will get substituted to filtered ones
	cv::Mat F = findFundamentalMatrix(keypoints1,keypoints2,keypoints1_refined,keypoints2_refined,matches);

	if(matches.size() < 15) { // || ((double)keypoints1.size() / (double)keypoints1_refined.size()) < 0.25
		LOG(Debug, "Insufficient number of matches after F refinement: ", (int)matches.size());
		return false;
	}

	// Essential matrix: compute then extract cameras [R|t]
	cv::Mat_<double> E = K.t() * F * K; //according to HZ (Equation 9.12)

	// Check validity according to http://en.wikipedia.org/wiki/Essential_matrix#Properties_of_the_essential_matrix
	if(fabsf(determinant(E)) > 1e-07) {
		LOG(Debug, "Essential matrix does not satisfy internal constraints! Determinant is too large: ", determinant(E));
		P1 = 0;
		return false;
	}

	cv::Mat_<double> R1(3,3);
	cv::Mat_<double> R2(3,3);
	cv::Mat_<double> t1(1,3);
	cv::Mat_<double> t2(1,3);

	// Using the essential matrix, decompose E to P' , HZ (Result 9.19)
	{			
		if (!decomposeEtoRandT(E,R1,R2,t1,t2)){
			LOG(Debug, "Unable to decompose essential matrix");
			return false;
		}

		if(determinant(R1)+1.0 < 1e-09) {
			//according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
			LOG(Info, "Essential matrix should have sign flipped! Determinant is ", determinant(E));
			E = -E;
			decomposeEtoRandT(E,R1,R2,t1,t2);
		}

		if (!checkCoherentRotation(R1)) {
			LOG(Debug, "Resulting rotation is not coherent");
			P1 = 0;
			return false;
		}
			
		P1 = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t1(0),
						R1(1,0),	R1(1,1),	R1(1,2),	t1(1),
						R1(2,0),	R1(2,1),	R1(2,2),	t1(2));
		LOG(Info, "Testing P1: ");
		LOG(Info, cv::Mat(P1));

		// At this point we have the initial camera view matrix (P0 - identity) and second camera view matrix (P1)
		// We will want to check if points are triangulated --in front-- of cameras for all 4 ambiguations
		// Matrix P1 will be comprised in four configurations - R1,t1 ; R1,t2 ; R2,t1 ; R2,t2
		// Once all required conditions are met - the next configurations need not be tried
		std::vector<CloudPoint> pcloud0,pcloud1; 

		double reproj_error1 = triangulatePoints(keypoints1_refined, keypoints2_refined, K, Kinv, distcoeff, P0, P1, pcloud0);
		double reproj_error2 = triangulatePoints(keypoints2_refined, keypoints1_refined, K, Kinv, distcoeff, P1, P0, pcloud1);
		
		std::vector<uchar> tmp_status;
		if (!testTriangulation(pcloud0,P1,tmp_status) || !testTriangulation(pcloud1,P0,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
			P1 = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t2(0),
							R1(1,0),	R1(1,1),	R1(1,2),	t2(1),
							R1(2,0),	R1(2,1),	R1(2,2),	t2(2));
			
			LOG(Info, "Testing P1: ");
			LOG(Info, cv::Mat(P1));

			pcloud0.clear(); pcloud1.clear();
			reproj_error1 = triangulatePoints(keypoints1_refined, keypoints2_refined, K, Kinv, distcoeff, P0, P1, pcloud0);
			reproj_error2 = triangulatePoints(keypoints2_refined, keypoints1_refined, K, Kinv, distcoeff, P1, P0, pcloud1);
				
			if (!testTriangulation(pcloud0,P1,tmp_status) || !testTriangulation(pcloud1,P0,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
				if (!checkCoherentRotation(R2)) {
					LOG(Debug, "Resulting rotation is not coherent");
					P1 = 0;
					return false;
				}
					
				P1 = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t1(0),
								R2(1,0),	R2(1,1),	R2(1,2),	t1(1),
								R2(2,0),	R2(2,1),	R2(2,2),	t1(2));
				LOG(Info, "Testing P1: ");
				LOG(Info, cv::Mat(P1));

				pcloud0.clear(); pcloud1.clear();
				reproj_error1 = triangulatePoints(keypoints1_refined, keypoints2_refined, K, Kinv, distcoeff, P0, P1, pcloud0);
				reproj_error2 = triangulatePoints(keypoints2_refined, keypoints1_refined, K, Kinv, distcoeff, P1, P0, pcloud1);
					
				if (!testTriangulation(pcloud0,P1,tmp_status) || !testTriangulation(pcloud1,P0,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
					P1 = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t2(0),
									R2(1,0),	R2(1,1),	R2(1,2),	t2(1),
									R2(2,0),	R2(2,1),	R2(2,2),	t2(2));
					LOG(Info, "Testing P1: ");
					LOG(Info, cv::Mat(P1));

					pcloud0.clear(); pcloud1.clear();
					reproj_error1 = triangulatePoints(keypoints1_refined, keypoints2_refined, K, Kinv, distcoeff, P0, P1, pcloud0);
					reproj_error2 = triangulatePoints(keypoints2_refined, keypoints1_refined, K, Kinv, distcoeff, P1, P0, pcloud1);
						
					if (!testTriangulation(pcloud0,P1,tmp_status) || !testTriangulation(pcloud1,P0,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
						LOG(Debug, "Triangulation has failed");
						return false;
					}
				}				
			}			
		}
		for (unsigned int i=0; i<pcloud0.size(); i++) {
			reconstructedCloud.push_back(pcloud0[i]);
		}
	}		
		
	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	LOG(Debug, "Finished calculating camera matrices in time of: ", time);
	return true;
}

cv::Mat findFundamentalMatrix(const std::vector<cv::KeyPoint>& keypoints1,
							const std::vector<cv::KeyPoint>& keypoints2,
							std::vector<cv::KeyPoint>& keypoints1_refined,
							std::vector<cv::KeyPoint>& keypoints2_refined,
							std::vector<cv::DMatch>& matches)
{
	// Try to eliminate keypoints based on the fundamental matrix
	std::vector<uchar> status(keypoints1.size());

	keypoints1_refined.clear(); 
	keypoints2_refined.clear();
	
	// Initialize matching point structures
	std::vector<cv::KeyPoint> keypoints1_tmp;
	std::vector<cv::KeyPoint> keypoints2_tmp;
	if (matches.size() <= 0) {
		keypoints1_tmp = keypoints1;
		keypoints2_tmp = keypoints2;
	} else {
		for (unsigned int i=0; i<matches.size(); i++) {
			keypoints1_tmp.push_back(keypoints1[matches[i].queryIdx]);
			keypoints2_tmp.push_back(keypoints2[matches[i].trainIdx]);
		}	
	}
	
	cv::Mat F;
	{
		std::vector<cv::Point2f> pts1,pts2;
		for (unsigned int i=0; i<keypoints1_tmp.size(); i++) pts1.push_back(keypoints1_tmp[i].pt);
		for (unsigned int i=0; i<keypoints2_tmp.size(); i++) pts2.push_back(keypoints2_tmp[i].pt);

		double minVal,maxVal;
		cv::minMaxIdx(pts1,&minVal,&maxVal);
		//F = findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.006 * maxVal, 0.99, status);	//threshold from [Snavely07 4.1]
		F = findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 10, 0.99, status);				// AS
	}
	
	std::vector<cv::DMatch> new_matches;
	LOG(Info, "Number of points kept by Ransac: ", cv::countNonZero(status));

	for (unsigned int i=0; i<status.size(); i++) {
		if (status[i]) 
		{
			keypoints1_refined.push_back(keypoints1_tmp[i]);
			keypoints2_refined.push_back(keypoints2_tmp[i]);
			new_matches.push_back(matches[i]);
		}
	}	
	
	// Keep only those points that "survived" the fundamental matrix
	matches = new_matches; 

	LOG(Debug, "Finished calculating fundamental matrix");
	return F;
}

bool decomposeEtoRandT(cv::Mat_<double>& E,
						cv::Mat_<double>& R1,
						cv::Mat_<double>& R2,
						cv::Mat_<double>& t1,
						cv::Mat_<double>& t2)
{
	// Show E matrix and other matrices on the way...

#ifdef DECOMPOSE_E_USING_SVD

	// Using HZ E decomposition
	cv::Mat svd_u, svd_vt, svd_w;
	takeSVDOfE(E,svd_u,svd_vt,svd_w);

	// Check if first and second singular values are the same (as they should be)
	LOG(Info, "Singular value 1: ", svd_w.at<double>(0));
	LOG(Info, "Singular value 2: ", svd_w.at<double>(1));

	double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
	if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7) {
		LOG(Debug, "Singular values are too far apart, ratio (min 0.7): ", singular_values_ratio);
		return false;
	}

	cv::Matx33d W(0,-1,0,				//HZ 9.13
				1,0,0,
				0,0,1);
	cv::Matx33d Wt(0,1,0,
				-1,0,0,
				0,0,1);
	R1 = svd_u * cv::Mat(W) * svd_vt;	//HZ 9.19
	R2 = svd_u * cv::Mat(Wt) * svd_vt;	//HZ 9.19
	t1 = svd_u.col(2);	//u3
	t2 = -svd_u.col(2); //u3
#else
	//Using Horn E decomposition
	DecomposeEssentialUsingHorn90(E[0],R1[0],R2[0],t1[0],t2[0]);
#endif
	return true;
}

void takeSVDOfE(cv::Mat_<double>& E, 
									cv::Mat& svd_u, 
									cv::Mat& svd_vt, 
									cv::Mat& svd_w) {
#ifdef SVD_USING_OPENCV
	// Using OpenCV's SVD, allow to modify input matrix (speeds up)
	cv::SVD svd(E,cv::SVD::MODIFY_A);
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;
#else
	//Using Eigen's SVD
	cout << "Eigen3 SVD..\n";
	Eigen::Matrix3f  e = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >((double*)E.data).cast<float>();
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(e, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf Esvd_u = svd.matrixU();
	Eigen::MatrixXf Esvd_v = svd.matrixV();
	svd_u = (Mat_<double>(3,3) << Esvd_u(0,0), Esvd_u(0,1), Esvd_u(0,2),
						  Esvd_u(1,0), Esvd_u(1,1), Esvd_u(1,2), 
						  Esvd_u(2,0), Esvd_u(2,1), Esvd_u(2,2)); 
	Mat_<double> svd_v = (Mat_<double>(3,3) << Esvd_v(0,0), Esvd_v(0,1), Esvd_v(0,2),
						  Esvd_v(1,0), Esvd_v(1,1), Esvd_v(1,2), 
						  Esvd_v(2,0), Esvd_v(2,1), Esvd_v(2,2));
	svd_vt = svd_v.t();
	svd_w = (Mat_<double>(1,3) << svd.singularValues()[0] , svd.singularValues()[1] , svd.singularValues()[2]);
#endif
	
	LOG(Info, "--------------------- SVD ---------------------");
	LOG(Info, "Calculated SVD (U,W,Vt): ");
	LOG(Info, svd_u); 	LOG(Info, " ");
	LOG(Info, svd_w);	LOG(Info, " ");
	LOG(Info, svd_vt);	LOG(Info, " ");
	LOG(Info, "-----------------------------------------------");
}

void DecomposeEssentialUsingHorn90(double _E[9], double _R1[9], double _R2[9], double _t1[3], double _t2[3]) {
	//from : http://people.csail.mit.edu/bkph/articles/Essential.pdf
#ifdef USE_EIGEN
	using namespace Eigen;

	Matrix3d E = Map<Matrix<double,3,3,RowMajor> >(_E);
	Matrix3d EEt = E * E.transpose();
	Vector3d e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(0));
	Vector3d b1,b2;

#if 1
	//Method 1
	Matrix3d bbt = 0.5 * EEt.trace() * Matrix3d::Identity() - EEt; //Horn90 (12)
	Vector3d bbt_diag = bbt.diagonal();
	if (bbt_diag(0) > bbt_diag(1) && bbt_diag(0) > bbt_diag(2)) {
		b1 = bbt.row(0) / sqrt(bbt_diag(0));
		b2 = -b1;
	} else if (bbt_diag(1) > bbt_diag(0) && bbt_diag(1) > bbt_diag(2)) {
		b1 = bbt.row(1) / sqrt(bbt_diag(1));
		b2 = -b1;
	} else {
		b1 = bbt.row(2) / sqrt(bbt_diag(2));
		b2 = -b1;
	}
#else
	//Method 2
	if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
		b1 = e0e1.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
		b2 = -b1;
	} else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
		b1 = e1e2.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
		b2 = -b1;
	} else {
		b1 = e2e0.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
		b2 = -b1;
	}
#endif
	
	//Horn90 (19)
	Matrix3d cofactors; cofactors.col(0) = e1e2; cofactors.col(1) = e2e0; cofactors.col(2) = e0e1;
	cofactors.transposeInPlace();
	
	//B = [b]_x , see Horn90 (6) and http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
	Matrix3d B1; B1 <<	0,-b1(2),b1(1),
						b1(2),0,-b1(0),
						-b1(1),b1(0),0;
	Matrix3d B2; B2 <<	0,-b2(2),b2(1),
						b2(2),0,-b2(0),
						-b2(1),b2(0),0;

	Map<Matrix<double,3,3,RowMajor> > R1(_R1),R2(_R2);

	//Horn90 (24)
	R1 = (cofactors.transpose() - B1*E) / b1.dot(b1);
	R2 = (cofactors.transpose() - B2*E) / b2.dot(b2);
	Map<Vector3d> t1(_t1),t2(_t2); 
	t1 = b1; t2 = b2;
	
	cout << "Horn90 provided " << endl << R1 << endl << "and" << endl << R2 << endl;
#endif
}

bool checkCoherentRotation(cv::Mat_<double>& R) {
	std::cout << "R; " << R << std::endl;
	
	if(fabsf(determinant(R))-1.0 > 1e-07) {
		LOG(Info, "Chcecking rotaion : det(R) != +-1.0, this is not a rotation matrix");
		return false;
	}

	return true;
}