#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

#include "sfmupdatelistener.h"
#include "scenemodel.h"
#include "featurehandler.h"

class ProcessingThread : public QThread
{
    Q_OBJECT
public:
    explicit ProcessingThread(QObject *parent = 0);
    virtual ~ProcessingThread();

	void initialize(SceneModel *sceneModel);

	QImage* getCurrentFrame(){return &currentFrameWithKeypoints;}

signals:
    void frameProcessed();
    void queueFull();

public:
    void stop();
    void addFrameToProcessingQueue(QImage frame);

private:
    virtual void run();

private:
    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;

	QImage currentFrame;
	QImage currentFrameWithKeypoints;
	SceneModel *sceneModel;
	FeatureHandler *featureHandler;

	SfMUpdateListener* listener;

public:
    void setUpdateListener(SfMUpdateListener *ul)
    {
        listener = ul;
    }
private:
    void update(std::vector<cv::Point3d> points, 
				std::vector<cv::Vec3b> pointsRGB,
				std::vector<cv::Matx34d> cameras)
    {
		listener->update(points, pointsRGB, cameras);
    }

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
