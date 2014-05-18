#include "stdafx.h"

#include "videosurface.h"

#include "qtcvconversion.h"
#include "logging.h"

#include <QVideoSurfaceFormat>
#include <QDebug>

#include <opencv2/opencv.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

const int FILLCOLOR = 0xFF0000;

VideoSurface::VideoSurface(QObject *parent) :
    QAbstractVideoSurface(parent)
{
    lastFrame.fill(FILLCOLOR);
}

VideoSurface::~VideoSurface()
{
    stop();
}

QImage VideoSurface::frame() const
{
    return lastFrame;
}

QList<QVideoFrame::PixelFormat> VideoSurface::supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType) const
{
    return QList<QVideoFrame::PixelFormat>() << QVideoFrame::Format_ARGB32
                                             << QVideoFrame::Format_RGB24
                                             << QVideoFrame::Format_RGB32;
}

bool VideoSurface::present(const QVideoFrame &frame)
{
    if (frame.isValid())
    {
        QVideoFrame videoFrame(frame);
        if( videoFrame.map(QAbstractVideoBuffer::ReadOnly) )
        {
            lastFrame = QImage(videoFrame.width(), videoFrame.height(), QImage::Format_RGB888);
            memcpy(lastFrame.bits(), videoFrame.bits(), videoFrame.mappedBytes());

            videoFrame.unmap();

            // Use thread for processing
            emit frameAvailable();
            return true;
        }
    }
    return false;
}

bool VideoSurface::start(const QVideoSurfaceFormat &format)
{
    if (isActive()) {
        stop();
    } else if (!format.frameSize().isEmpty()) {
        return QAbstractVideoSurface::start(format);
    }
    return false;
}

void VideoSurface::stop()
{
    lastFrame.fill(FILLCOLOR);
    QAbstractVideoSurface::stop();
}
