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

void ImageReadingThread::stop()
{
    stopped = true;
}

void ImageReadingThread::run()
{
    ImageReader imageReader;

    // Process until stop() called
    while (!stopped)
    {
        QImage *currentFrame = new QImage();
        *currentFrame = imageReader.getNextImage();

        emit imageReady(currentFrame);

        // No frames in queue, sleep for a short while
        msleep(THREAD_SLEEP_MS);
    }

    exit(0);
}
