#include "stdafx.h"

#include "processingthread.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QDebug>
#include <QTextStream>

#include <opencv2/nonfree/features2d.hpp>

#define DECOMPOSE_SVD
#define SVD_OPENCV

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
					// Get keypoints & descriptors
				std::vector<cv::KeyPoint> keypoints1 = sceneModel->getKeypoints(idx1);
				cv::Mat descriptors1 = sceneModel->getDescriptors(idx1);
				std::vector<cv::KeyPoint> keypoints2 = sceneModel->getKeypoints(idx2);
				cv::Mat descriptors2 = sceneModel->getDescriptors(idx2);
				featureHandler->findMatches(idx1, idx2, keypoints1, descriptors1, keypoints2, descriptors2, goodMatches);

				// Store good matches in model
				sceneModel->addMatches(idx1,idx2,goodMatches);
				LOG(Debug, "Stored matches");
		
				// DEBUG - Drawing matches
				cv::Mat img_matches;
				cv::drawMatches(sceneModel->frames[idx1], sceneModel->getKeypoints(idx1), 
								sceneModel->frames[idx2], sceneModel->getKeypoints(idx2),
								goodMatches, img_matches);
				imshow( "Good Matches", img_matches );
				cv::waitKey(0);

				// Set camera parameters - necessary for reconstruction
				cv::Matx34d P,P1;
				cv::Mat K;
				cv::Mat_<double> Kinv;
				cv::Mat distortion_coeff;
				std::vector<CloudPoint> pointcloud;

				/*cv::FileStorage fs;
				fs.open("../out_camera_data.yml",cv::FileStorage::READ);
				fs["camera_matrix"]>>cam_matrix;
				fs["distortion_coefficients"]>>distortion_coeff;*/

				P = cv::Matx34d(1,0,0,0,
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
									keypoints1Refined, keypoints2Refined, P, P1, goodMatches, pointcloud);
								
				// DEBUG - Drawing matches that survived the fundamental matrix
				cv::drawMatches(sceneModel->frames[idx1], sceneModel->getKeypoints(idx1), 
								sceneModel->frames[idx2], sceneModel->getKeypoints(idx2),
								goodMatches, img_matches, cv::Scalar(0,0,255));
				imshow( "Good Matches After F Refinement", img_matches );
				cv::waitKey(0);

				triangulatePoints();
							
				// Assuming we have two images appropriate for initialization - calculate stereo
				// For this we need the 5-point algorithm and triangulation
				// TODO Move this to the correct place

				// End of stereo initialization
				LOG(Debug, "Initialized stereo model");


			}

			
			/* 
			Create online map (PTAM):
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

bool ProcessingThread::findCameraMatrices(const cv::Mat& K, 
						const cv::Mat& Kinv, 
						const cv::Mat& distcoeff,
						const std::vector<cv::KeyPoint>& keypoints1,
						const std::vector<cv::KeyPoint>& keypoints2,
						std::vector<cv::KeyPoint>& keypoints1_refined,
						std::vector<cv::KeyPoint>& keypoints2_refined,
						cv::Matx34d& P,
						cv::Matx34d& P1,
						std::vector<cv::DMatch>& matches,
						std::vector<CloudPoint>& outCloud){

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
	}

	LOG(Debug, "Finished calculating camera matrices");
	return true;
}

cv::Mat ProcessingThread::findFundamentalMatrix(const std::vector<cv::KeyPoint>& keypoints1,
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
		//F = findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
		F = findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 10, 0.99, status);
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

bool ProcessingThread::decomposeEtoRandT(cv::Mat_<double>& E,
						cv::Mat_<double>& R1,
						cv::Mat_<double>& R2,
						cv::Mat_<double>& t1,
						cv::Mat_<double>& t2)
{
	// Show E matrix and other matrices on the way...

#ifdef DECOMPOSE_SVD

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

void ProcessingThread::takeSVDOfE(cv::Mat_<double>& E, 
									cv::Mat& svd_u, 
									cv::Mat& svd_vt, 
									cv::Mat& svd_w) {
#ifdef SVD_OPENCV
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

bool ProcessingThread::checkCoherentRotation(cv::Mat_<double>& R) {
	std::cout << "R; " << R << std::endl;
	
	if(fabsf(determinant(R))-1.0 > 1e-07) {
		LOG(Info, "Chcecking rotaion : det(R) != +-1.0, this is not a rotation matrix");
		return false;
	}

	return true;
}

bool ProcessingThread::triangulatePoints(){
	return true;
}