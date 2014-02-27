#include "videosurface.h"

#include <QVideoSurfaceFormat>
#include <QDebug>

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
