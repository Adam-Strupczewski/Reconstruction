#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

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

	bool findCameraMatrices(const cv::Mat& K, 
						const cv::Mat& Kinv, 
						const cv::Mat& distcoeff,
						const std::vector<cv::KeyPoint>& keypoints1,
						const std::vector<cv::KeyPoint>& keypoints2,
						std::vector<cv::KeyPoint>& keypoints1_refined,
						std::vector<cv::KeyPoint>& keypoints2_refined,
						cv::Matx34d& P,
						cv::Matx34d& P1,
						std::vector<cv::DMatch>& matches,
						std::vector<CloudPoint>& outCloud);

	// Not called directly from reconstruction loop
	cv::Mat findFundamentalMatrix(const std::vector<cv::KeyPoint>& keypoints1,
							const std::vector<cv::KeyPoint>& keypoints2,
							std::vector<cv::KeyPoint>& keypoints1_refined,
							std::vector<cv::KeyPoint>& keypoints2_refined,
							std::vector<cv::DMatch>& matches);
	
	bool decomposeEtoRandT(cv::Mat_<double>& E,
							cv::Mat_<double>& R1,
							cv::Mat_<double>& R2,
							cv::Mat_<double>& t1,
							cv::Mat_<double>& t2);

	void takeSVDOfE(cv::Mat_<double>& E, 
					cv::Mat& svd_u, 
					cv::Mat& svd_vt, 
					cv::Mat& svd_w);

	bool checkCoherentRotation(cv::Mat_<double>& R);

	bool triangulatePoints();

    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;

	QImage currentFrame;
	QImage currentFrameWithKeypoints;
	SceneModel *sceneModel;
	FeatureHandler *featureHandler;
};

#endif // PROCESSINGTHREAD_H
