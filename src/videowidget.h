#ifndef VIDEOWIDGET_H
#define VIDEOWIDGET_H

#include <QWidget>
#include <QAbstractVideoSurface>

#include "sfmviewer.h"

#include "videosurface.h"
#include "processingthread.h"
#include "scenemodel.h"

class VideoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VideoWidget(QWidget *parent = 0);
    virtual ~VideoWidget();

	void initialize(SFMViewer *sfmViewer, SceneModel * sceneModel);

    void setImageBuffers(QImage *im1, QImage *im2);

    QAbstractVideoSurface *videoSurface() { return surface; }

    QSize sizeHint() const;

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

private slots:

    // Surface
    void frameReady();

    // From file
    // Image should be deleted after processing is finished
    void imageReady();

    // ProcessingThread
    void onFrameProcessed();
    void onThreadCongested();

private:
    VideoSurface *surface;
    QPixmap currentFrame;

    ProcessingThread* processor;

    long receivedFrameCounter;
    long processedFrameCounter;

    // These buffers are set and managed externally
    QImage * imageBuffer1;
    QImage * imageBuffer2;
};

#endif
