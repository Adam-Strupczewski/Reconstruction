#include "stdafx.h"

#include "imagereadingthread.h"
#include "imagereader.h"
#include "logging.h"

#include <QDebug>

static const int THREAD_SLEEP_MS = 250;

ImageReadingThread::ImageReadingThread(QObject *parent) :
    QThread(parent), stopped(false)
{
}

ImageReadingThread::~ImageReadingThread()
{
    stop();
}

void ImageReadingThread::initialize(SceneModel * sceneModel){
	imageReader.initialize(sceneModel);
}

void ImageReadingThread::setImageBuffers(QImage *im1, QImage *im2){
    imageReader.setImageBuffers(im1, im2);
}

void ImageReadingThread::stop()
{
    stopped = true;
}

void ImageReadingThread::run()
{
    // Process until stop() called
    while (!stopped)
    {
        if (imageReader.getNextImage()){
            emit imageReady();
        }else{
			LOG(Debug, "Read all images");
           break;
        }

        // No frames in queue, sleep for a short while
        msleep(THREAD_SLEEP_MS);
    }
}
