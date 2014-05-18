#ifndef RECONSTRUCTIONHANDLER_H
#define RECONSTRUCTIONHANDLER_H

#include "scenemodel.h"
#include "featurehandler.h"

class ReconstructionHandler
{

public:
    explicit ReconstructionHandler();
    virtual ~ReconstructionHandler();

	void initialize(SceneModel *sceneModel);

	void initializeImage(cv::Mat img, int imageInitializedCnt);

	bool initializeStereoModel(std::vector<cv::Point3d> &points, std::vector<cv::Vec3b> &pointsRGB, int imageInitializedCnt);
	bool addNextViewToStereoModel(std::vector<cv::Point3d> &points, std::vector<cv::Vec3b> &pointsRGB, int imageInitializedCnt);

private:
	SceneModel *sceneModel;
	FeatureHandler *featureHandler;

private:

	void getRGBForPointCloud(const std::vector<struct CloudPoint>& pcloud,
							std::vector<cv::Vec3b>& RGBCloud);

	void find2D3DCorrespondences(int working_view, 
								std::vector<cv::Point3f>& ppcloud, 
								std::vector<cv::Point2f>& imgPoints);

	bool findPoseEstimation(int working_view,
							cv::Mat_<double>& rvec,
							cv::Mat_<double>& t,
							cv::Mat_<double>& R,
							std::vector<cv::Point3f> ppcloud,
							std::vector<cv::Point2f> imgPoints); 

	bool triangulatePointsBetweenViews(int workingView, 
										int olderView,
										std::vector<struct CloudPoint>& newTriangulated,
										std::vector<int>& addToCloud,
										int numViews); 

	void updateReprojectionErrors();

	int findHomographyInliers2Views(int vi, int vj);

	int m_first_view;
	int m_second_view; //baseline's second view other to 0
};

#endif // PROCESSINGTHREAD_H
