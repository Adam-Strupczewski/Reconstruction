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
				std::vector<cv::Vec3b> pointsRGB)
    {
		listener->update(points, pointsRGB);
    }

	void getRGBForPointCloud(const std::vector<struct CloudPoint>& pcloud,
							std::vector<cv::Vec3b>& RGBCloud);
};

#endif // PROCESSINGTHREAD_H
