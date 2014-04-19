#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

#include "scenemodel.h"

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
	bool extractKeypoints(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints);

	bool extractDescriptors(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

	bool findMatches(int idx1, int idx2, std::vector< cv::DMatch > &matches);

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

	bool triangulatePoints();

	// Not called directly from reconstruction loop
	cv::Mat findFundamentalMatrix(const std::vector<cv::KeyPoint>& keypoints1,
							const std::vector<cv::KeyPoint>& keypoints2,
							std::vector<cv::KeyPoint>& keypoints1_refined,
							std::vector<cv::KeyPoint>& keypoints2_refined,
							std::vector<cv::DMatch>& matches);

    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;

	QImage currentFrame;
	QImage currentFrameWithKeypoints;
	SceneModel *sceneModel;
};

#endif // PROCESSINGTHREAD_H
