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
	bool findCameraMatrices();
	bool triangulatePoints();

    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;

	QImage currentFrame;
	QImage currentFrameWithKeypoints;
	SceneModel *sceneModel;
};

#endif // PROCESSINGTHREAD_H
