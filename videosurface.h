#ifndef VIDEOSURFACE_H
#define VIDEOSURFACE_H

#include <QAbstractVideoSurface>
#include <QImage>

class VideoSurface : public QAbstractVideoSurface
{
    Q_OBJECT
public:
    explicit VideoSurface(QObject *parent = 0);
    virtual ~VideoSurface();
signals:
    void frameAvailable();

public:
    QImage frame() const;
    virtual void stop();

    //void updateVideoRect();

private:
    virtual QList<QVideoFrame::PixelFormat> supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const;
    virtual bool present(const QVideoFrame &frame);
    virtual bool start(const QVideoSurfaceFormat &format);

    QImage lastFrame;
};

#endif // VIDEOSURFACE_H
