#ifndef IMAGEREADINGTHREAD_H
#define IMAGEREADINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

#include "scenemodel.h"
#include "imagereader.h"

class ImageReadingThread : public QThread
{
    Q_OBJECT
public:
    explicit ImageReadingThread(QObject *parent = 0);
    virtual ~ImageReadingThread();

	void initialize(SceneModel * sceneModel);

    void setImageBuffers(QImage *im1, QImage *im2);

signals:
    void imageReady();
public:
    void stop();
private:
    virtual void run();
private:
    ImageReader imageReader;

    bool stopped;
};

#endif // IMAGEREADINGTHREAD_H
