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

            /***************************************************************/
            // Process last frame
        /*    IplImage iplImage = *(cvtQImage2IplImage(lastFrame));

            //Convert to grayscale
            IplImage * imageGrayScale = 0;
            if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
            {
                LOG(Debug, "Created grascale");
                imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
                cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
            }
            cv::Mat img;
            if(imageGrayScale)
            {
                img = cv::Mat(imageGrayScale);
            }
            else
            {
                img =  cv::Mat(&iplImage);
            }
*/


            // TODO Use thread in future
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
