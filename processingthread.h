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
    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;

	QImage currentFrame;
	QImage currentFrameWithKeypoints;
	SceneModel *sceneModel;
	FeatureHandler *featureHandler;
};

#endif // PROCESSINGTHREAD_H
